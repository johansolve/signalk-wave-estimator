'use strict'

const { analyze } = require('./lib/wave-estimator')

module.exports = function (app) {
  let timer = null
  let unsubscribes = []
  let buffer = [] // time-sorted [{ t, pitch, roll }]
  // Each scalar input keeps its arrival time so compute() can ignore a source
  // that has gone silent instead of trusting a stale value forever.
  let latest = emptyLatest()
  // Recent valid estimates [{ period, directionRelative }] for the temporal
  // stability gate; an estimate that jitters between cycles is downweighted.
  let history = []
  let opts = {}

  const STALE_MS = 20000
  const HISTORY_MAX = 6

  function emptyLatest () {
    return {
      stw: { value: null, t: 0 },
      headingTrue: { value: null, t: 0 },
      headingMagnetic: { value: null, t: 0 },
      windDir: { value: null, t: 0 }
    }
  }

  function fresh (f) {
    return (f.value != null && (Date.now() - f.t) < STALE_MS) ? f.value : null
  }

  const plugin = {
    id: 'signalk-wave-estimator',
    name: 'Wave estimator (from boat motion)',
    description:
      'Derives sea state from boat motion: estimates the wave encounter period ' +
      'from pitch/roll, corrects for speed through water via the deep-water ' +
      'encounter relation, and publishes true wave period, length, celerity and ' +
      'a slope-inversion height proxy under environment.wave.*. The height is a ' +
      'PROXY (no heave sensor on board) and carries a confidence value — see README.'
  }

  plugin.schema = () => ({
    type: 'object',
    properties: {
      windowSeconds: {
        type: 'number',
        title: 'Analysis window (s)',
        description: 'Rolling buffer length the FFT runs over. Longer = better low-frequency resolution, slower to react.',
        default: 90,
        minimum: 20
      },
      updateSeconds: {
        type: 'number',
        title: 'Update interval (s)',
        description: 'How often a new estimate is computed and published.',
        default: 5,
        minimum: 1
      },
      fsTarget: {
        type: 'number',
        title: 'Resample rate (Hz)',
        description: 'Attitude arrives irregularly; it is resampled to this uniform rate before the FFT. 4 Hz resolves all wave frequencies.',
        default: 4,
        minimum: 2
      },
      periodMin: {
        type: 'number',
        title: 'Shortest wave period considered (s)',
        description: 'Lower bound of the analysis band. Below ~2 s the hull no longer follows the wave slope.',
        default: 2,
        minimum: 1
      },
      periodMax: {
        type: 'number',
        title: 'Longest wave period considered (s)',
        description: 'Upper bound of the analysis band.',
        default: 20,
        minimum: 5
      },
      boatLength: {
        type: 'number',
        title: 'Waterline length (m)',
        description: 'Used for the confidence flag: the slope/height estimate is only trustworthy when the wavelength clearly exceeds the hull. Elan 333 LWL ≈ 8.4 m.',
        default: 8.4,
        minimum: 1
      },
      defaultRegime: {
        type: 'string',
        title: 'Default sea regime when wind is unknown',
        description: 'Head vs following sets the sign of the speed correction. Used only when wind direction is unavailable to discriminate.',
        enum: ['head', 'following'],
        default: 'head'
      },
      minConfidence: {
        type: 'number',
        title: 'Minimum confidence to publish',
        description: 'Estimates below this confidence (0–1) are suppressed rather than published as noise. 0 publishes everything.',
        default: 0.1,
        minimum: 0,
        maximum: 1
      },
      minSlopeDeg: {
        type: 'number',
        title: 'Minimum motion to report (° RMS slope)',
        description: 'Amplitude gate: suppress estimates when the RMS pitch/roll slope is below this. At the dock the boat barely moves (~0.04° RMS), so a small floor rejects noise while any real sea (degrees of slope) passes. Set 0 to disable.',
        default: 0.5,
        minimum: 0
      },
      flipSide: {
        type: 'boolean',
        title: 'Flip port/starboard side',
        description: 'Port/starboard of the wave direction comes from the pitch/roll cross-spectrum sign, which depends on the IMU mounting and attitude convention. If the reported side is mirrored at sea trial, enable this to invert it.',
        default: false
      }
    }
  })

  plugin.start = function (settings) {
    opts = Object.assign(
      {
        windowSeconds: 90,
        updateSeconds: 5,
        fsTarget: 4,
        periodMin: 2,
        periodMax: 20,
        boatLength: 8.4,
        defaultRegime: 'head',
        minConfidence: 0.1,
        minSlopeDeg: 0.5,
        flipSide: false
      },
      settings || {}
    )
    buffer = []
    latest = emptyLatest()
    history = []

    subscribe()
    publishMeta()
    timer = setInterval(compute, Math.max(1, opts.updateSeconds) * 1000)
    app.setPluginStatus('Collecting motion data…')
  }

  plugin.stop = function () {
    if (timer) { clearInterval(timer); timer = null }
    unsubscribes.forEach((f) => { try { f() } catch (e) { /* ignore */ } })
    unsubscribes = []
    buffer = []
  }

  // ---- Input ---------------------------------------------------------------

  function subscribe () {
    const subscription = {
      context: 'vessels.self',
      subscribe: [
        { path: 'navigation.attitude', policy: 'instant', minPeriod: 0 },
        { path: 'navigation.speedThroughWater', period: 1000 },
        { path: 'navigation.headingTrue', period: 1000 },
        { path: 'navigation.headingMagnetic', period: 1000 },
        { path: 'environment.wind.directionTrue', period: 2000 }
      ]
    }
    app.subscriptionmanager.subscribe(
      subscription,
      unsubscribes,
      (err) => app.error('subscription error: ' + err),
      onDelta
    )
  }

  function onDelta (delta) {
    if (!delta.updates) { return }
    const now = Date.now()
    for (const u of delta.updates) {
      if (!u.values) { continue }
      // Buffer samples carry the delta's own (signal) time; the scalar inputs are
      // stamped with arrival time, which is the right reference for staleness.
      const t = u.timestamp ? Date.parse(u.timestamp) : now
      for (const v of u.values) {
        switch (v.path) {
          case 'navigation.attitude':
            if (v.value && Number.isFinite(v.value.pitch) && Number.isFinite(v.value.roll)) {
              buffer.push({ t, pitch: v.value.pitch, roll: v.value.roll })
            }
            break
          case 'navigation.speedThroughWater':
            if (Number.isFinite(v.value)) { latest.stw = { value: v.value, t: now } }
            break
          case 'navigation.headingTrue':
            if (Number.isFinite(v.value)) { latest.headingTrue = { value: v.value, t: now } }
            break
          case 'navigation.headingMagnetic':
            if (Number.isFinite(v.value)) { latest.headingMagnetic = { value: v.value, t: now } }
            break
          case 'environment.wind.directionTrue':
            if (Number.isFinite(v.value)) { latest.windDir = { value: v.value, t: now } }
            break
        }
      }
    }
  }

  function trimBuffer () {
    if (buffer.length === 0) { return }
    // Trim relative to the newest sample's own timestamp, not the wall clock —
    // delta timestamps can be offset from server time (clock skew, playback), and
    // mixing the two would size the window wrong or let the buffer grow unbounded.
    const cutoff = buffer[buffer.length - 1].t - opts.windowSeconds * 1000
    let i = 0
    while (i < buffer.length && buffer[i].t < cutoff) { i++ }
    if (i > 0) { buffer = buffer.slice(i) }
  }

  // ---- Compute & publish ---------------------------------------------------

  function compute () {
    trimBuffer()
    if (buffer.length < 8) {
      app.setPluginStatus(`Collecting motion data… (${buffer.length} samples)`)
      return
    }

    const headingTrue = fresh(latest.headingTrue)
    const headingMagnetic = fresh(latest.headingMagnetic)
    const ctx = {
      fsTarget: opts.fsTarget,
      minSamples: 64,
      periodMin: opts.periodMin,
      periodMax: opts.periodMax,
      boatLength: opts.boatLength,
      defaultRegime: opts.defaultRegime,
      stw: fresh(latest.stw),
      heading: headingTrue != null ? headingTrue : headingMagnetic,
      windDir: fresh(latest.windDir),
      flipSide: opts.flipSide,
      history
    }

    let r
    try {
      r = analyze(buffer, ctx)
    } catch (e) {
      app.error('analyze failed: ' + (e && e.message))
      return
    }
    if (!r) {
      app.setPluginStatus(`Buffering… (${buffer.length} samples, need a longer/cleaner window)`)
      return
    }

    // A heartbeat (state + rmsSlope + gate) is published every cycle the
    // analysis succeeds, even when gated, so the webapp can tell "too calm" and
    // "low confidence" apart from "no data / plugin off" (no heartbeat at all).
    const rmsSlopeDeg = r.rmsSlope * 180 / Math.PI
    const slopeGate = opts.minSlopeDeg * Math.PI / 180

    if (rmsSlopeDeg < opts.minSlopeDeg) {
      emit([
        { path: 'environment.wave.state', value: 'calm' },
        { path: 'environment.wave.rmsSlope', value: r.rmsSlope },
        { path: 'environment.wave.slopeGate', value: slopeGate }
      ])
      app.setPluginStatus(
        `Too calm — ${rmsSlopeDeg.toFixed(2)}° RMS slope < ${opts.minSlopeDeg}° gate. Not publishing waves.`
      )
      return
    }

    if (!r._solveOk) {
      // The encounter solve failed or was ill-conditioned (fast following sea):
      // the corrected period would be systematically wrong, not merely uncertain,
      // so publish only the heartbeat and withhold the wave parameters.
      emit([
        { path: 'environment.wave.state', value: 'lowConfidence' },
        { path: 'environment.wave.rmsSlope', value: r.rmsSlope },
        { path: 'environment.wave.slopeGate', value: slopeGate },
        { path: 'environment.wave.confidence', value: r.confidence }
      ])
      app.setPluginStatus(
        `Encounter solve unreliable (fast following sea) — not publishing. ` +
        `Te≈${r.encounterPeriod.toFixed(1)}s`
      )
      return
    }

    // Valid solve: record it for the temporal-stability gate (whether or not it
    // clears the confidence threshold) so jitter is detectable across cycles.
    history.push({ period: r.period, directionRelative: r.directionRelative })
    if (history.length > HISTORY_MAX) { history = history.slice(history.length - HISTORY_MAX) }

    if (r.confidence < opts.minConfidence) {
      emit([
        { path: 'environment.wave.state', value: 'lowConfidence' },
        { path: 'environment.wave.rmsSlope', value: r.rmsSlope },
        { path: 'environment.wave.slopeGate', value: slopeGate },
        { path: 'environment.wave.confidence', value: r.confidence }
      ])
      app.setPluginStatus(
        `Low confidence (${r.confidence.toFixed(2)}) — not publishing. ` +
        `T≈${r.period.toFixed(1)}s λ≈${r.length.toFixed(0)}m`
      )
      return
    }

    publish(r, slopeGate)
    app.setPluginStatus(
      `T ${r.period.toFixed(1)}s · λ ${r.length.toFixed(0)}m · ` +
      `Hs≈${r.significantHeight != null ? r.significantHeight.toFixed(2) : '?'}m · ` +
      `conf ${r.confidence.toFixed(2)} (${r._following ? 'following' : 'head'}, ` +
      `STW ${r._stw != null ? r._stw.toFixed(1) : '?'})` +
      (r.secondary != null ? ` · 2nd T ${r.secondary.period.toFixed(1)}s` : '')
    )
  }

  function emit (values) {
    app.handleMessage(plugin.id, {
      updates: [
        {
          source: { label: plugin.id },
          timestamp: new Date().toISOString(),
          values
        }
      ]
    })
  }

  function publish (r, slopeGate) {
    const values = [
      { path: 'environment.wave.state', value: 'ok' },
      { path: 'environment.wave.rmsSlope', value: r.rmsSlope },
      { path: 'environment.wave.slopeGate', value: slopeGate },
      { path: 'environment.wave.period', value: r.period },
      { path: 'environment.wave.encounterPeriod', value: r.encounterPeriod },
      { path: 'environment.wave.length', value: r.length },
      { path: 'environment.wave.celerity', value: r.celerity },
      { path: 'environment.wave.groupSpeed', value: r.groupSpeed },
      { path: 'environment.wave.confidence', value: r.confidence }
    ]
    if (r.significantHeight != null) {
      values.push({ path: 'environment.wave.significantHeight', value: r.significantHeight })
      values.push({ path: 'environment.wave.heightConfidence', value: r.heightConfidence })
    }
    if (r.directionTrue != null) {
      values.push({ path: 'environment.wave.directionTrue', value: r.directionTrue })
    }
    values.push({ path: 'environment.wave.directionRelative', value: r.directionRelative })

    // Secondary wave system (bimodal sea), when a distinct second peak is found.
    const s = r.secondary
    values.push({ path: 'environment.wave.secondary.present', value: s != null })
    if (s != null) {
      values.push({ path: 'environment.wave.secondary.period', value: s.period })
      values.push({ path: 'environment.wave.secondary.length', value: s.length })
      values.push({ path: 'environment.wave.secondary.directionRelative', value: s.directionRelative })
      if (s.directionTrue != null) {
        values.push({ path: 'environment.wave.secondary.directionTrue', value: s.directionTrue })
      }
      values.push({ path: 'environment.wave.secondary.confidence', value: s.confidence })
    }

    emit(values)
  }

  function publishMeta () {
    const meta = [
      { path: 'environment.wave.period', value: { units: 's', displayName: 'Wave period', shortName: 'Tw', description: 'True wave period (encounter period corrected for STW)' } },
      { path: 'environment.wave.encounterPeriod', value: { units: 's', displayName: 'Wave encounter period', shortName: 'Te' } },
      { path: 'environment.wave.length', value: { units: 'm', displayName: 'Wavelength', shortName: 'λ' } },
      { path: 'environment.wave.celerity', value: { units: 'm/s', displayName: 'Wave celerity', shortName: 'c' } },
      { path: 'environment.wave.groupSpeed', value: { units: 'm/s', displayName: 'Wave group speed', shortName: 'cg' } },
      { path: 'environment.wave.significantHeight', value: { units: 'm', displayName: 'Sig. wave height (proxy)', shortName: 'Hs~', description: 'Slope-inversion proxy from pitch/roll — no heave sensor; trust only with high height confidence' } },
      { path: 'environment.wave.heightConfidence', value: { units: 'ratio', displayName: 'Height confidence', shortName: 'confH', description: 'Confidence in the height proxy specifically; <= overall confidence, also folds in wavelength-vs-hull contouring validity' } },
      { path: 'environment.wave.directionTrue', value: { units: 'rad', displayName: 'Wave direction (from)', shortName: 'Dir' } },
      { path: 'environment.wave.directionRelative', value: { units: 'rad', displayName: 'Wave dir. rel. bow (from)', shortName: 'DirRel', description: 'Signed angle off the bow axis, +ve starboard / -ve port (side from pitch/roll cross-spectrum)' } },
      { path: 'environment.wave.confidence', value: { units: 'ratio', displayName: 'Estimate confidence', shortName: 'conf', description: 'Confidence in period/direction (spectral narrowness, encounter-solve, roll-resonance and temporal stability)' } },
      { path: 'environment.wave.secondary.present', value: { displayName: 'Secondary wave present', description: 'Whether a distinct second wave system (e.g. swell under a wind sea) was detected' } },
      { path: 'environment.wave.secondary.period', value: { units: 's', displayName: 'Secondary wave period', shortName: 'Tw2' } },
      { path: 'environment.wave.secondary.length', value: { units: 'm', displayName: 'Secondary wavelength', shortName: 'λ2' } },
      { path: 'environment.wave.secondary.directionTrue', value: { units: 'rad', displayName: 'Secondary direction (from)', shortName: 'Dir2' } },
      { path: 'environment.wave.secondary.directionRelative', value: { units: 'rad', displayName: 'Secondary dir. rel. bow (from)', shortName: 'DirRel2', description: 'Signed angle off the bow axis, +ve starboard / -ve port' } },
      { path: 'environment.wave.secondary.confidence', value: { units: 'ratio', displayName: 'Secondary confidence', shortName: 'conf2', description: 'Prominence of the secondary peak relative to the primary' } },
      { path: 'environment.wave.state', value: { displayName: 'Estimator state', description: 'ok | calm (below the motion gate) | lowConfidence' } },
      { path: 'environment.wave.rmsSlope', value: { units: 'rad', displayName: 'RMS pitch/roll slope', shortName: 'slope', description: 'Raw wave-band motion energy; the amplitude gate acts on this' } },
      { path: 'environment.wave.slopeGate', value: { units: 'rad', displayName: 'Motion gate threshold', shortName: 'gate' } }
    ]
    app.handleMessage(plugin.id, { updates: [{ meta }] })
  }

  return plugin
}
