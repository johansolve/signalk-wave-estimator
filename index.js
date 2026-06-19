'use strict'

const { analyze } = require('./lib/wave-estimator')

module.exports = function (app) {
  let timer = null
  let unsubscribes = []
  let buffer = [] // time-sorted [{ t, pitch, roll }]
  let latest = { stw: null, headingTrue: null, headingMagnetic: null, windDir: null }
  let opts = {}

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
        minSlopeDeg: 0.5
      },
      settings || {}
    )
    buffer = []
    latest = { stw: null, headingTrue: null, headingMagnetic: null, windDir: null }

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
    for (const u of delta.updates) {
      if (!u.values) { continue }
      const t = u.timestamp ? Date.parse(u.timestamp) : Date.now()
      for (const v of u.values) {
        switch (v.path) {
          case 'navigation.attitude':
            if (v.value && Number.isFinite(v.value.pitch) && Number.isFinite(v.value.roll)) {
              buffer.push({ t, pitch: v.value.pitch, roll: v.value.roll })
            }
            break
          case 'navigation.speedThroughWater':
            if (Number.isFinite(v.value)) { latest.stw = v.value }
            break
          case 'navigation.headingTrue':
            if (Number.isFinite(v.value)) { latest.headingTrue = v.value }
            break
          case 'navigation.headingMagnetic':
            if (Number.isFinite(v.value)) { latest.headingMagnetic = v.value }
            break
          case 'environment.wind.directionTrue':
            if (Number.isFinite(v.value)) { latest.windDir = v.value }
            break
        }
      }
    }
  }

  function trimBuffer () {
    const cutoff = Date.now() - opts.windowSeconds * 1000
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

    const ctx = {
      fsTarget: opts.fsTarget,
      minSamples: 64,
      periodMin: opts.periodMin,
      periodMax: opts.periodMax,
      boatLength: opts.boatLength,
      defaultRegime: opts.defaultRegime,
      stw: latest.stw,
      heading: latest.headingTrue != null ? latest.headingTrue : latest.headingMagnetic,
      windDir: latest.windDir
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
      `conf ${r.confidence.toFixed(2)} (${r._following ? 'following' : 'head'}, STW ${r._stw.toFixed(1)})`
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
    }
    if (r.directionTrue != null) {
      values.push({ path: 'environment.wave.directionTrue', value: r.directionTrue })
    }
    values.push({ path: 'environment.wave.directionRelative', value: r.directionRelative })

    emit(values)
  }

  function publishMeta () {
    const meta = [
      { path: 'environment.wave.period', value: { units: 's', displayName: 'Wave period', shortName: 'Tw', description: 'True wave period (encounter period corrected for STW)' } },
      { path: 'environment.wave.encounterPeriod', value: { units: 's', displayName: 'Wave encounter period', shortName: 'Te' } },
      { path: 'environment.wave.length', value: { units: 'm', displayName: 'Wavelength', shortName: 'λ' } },
      { path: 'environment.wave.celerity', value: { units: 'm/s', displayName: 'Wave celerity', shortName: 'c' } },
      { path: 'environment.wave.groupSpeed', value: { units: 'm/s', displayName: 'Wave group speed', shortName: 'cg' } },
      { path: 'environment.wave.significantHeight', value: { units: 'm', displayName: 'Sig. wave height (proxy)', shortName: 'Hs~', description: 'Slope-inversion proxy from pitch/roll — no heave sensor; trust only with high confidence' } },
      { path: 'environment.wave.directionTrue', value: { units: 'rad', displayName: 'Wave direction (from)', shortName: 'Dir' } },
      { path: 'environment.wave.directionRelative', value: { units: 'rad', displayName: 'Wave dir. rel. bow (from)', shortName: 'DirRel', description: 'Magnitude off the bow axis; port/starboard side is unresolved' } },
      { path: 'environment.wave.confidence', value: { units: 'ratio', displayName: 'Estimate confidence', shortName: 'conf' } },
      { path: 'environment.wave.state', value: { displayName: 'Estimator state', description: 'ok | calm (below the motion gate) | lowConfidence' } },
      { path: 'environment.wave.rmsSlope', value: { units: 'rad', displayName: 'RMS pitch/roll slope', shortName: 'slope', description: 'Raw wave-band motion energy; the amplitude gate acts on this' } },
      { path: 'environment.wave.slopeGate', value: { units: 'rad', displayName: 'Motion gate threshold', shortName: 'gate' } }
    ]
    app.handleMessage(plugin.id, { updates: [{ meta }] })
  }

  return plugin
}
