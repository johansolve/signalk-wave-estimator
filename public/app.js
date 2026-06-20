'use strict'

// Self-contained webapp for signalk-wave-estimator. Connects to the Signal K
// delta stream (served from the same host), subscribes to environment.wave.*
// and renders the values in a mobile-friendly layout. No external resources.

(function () {
  var MS_TO_KN = 1.94384
  var RAD_TO_DEG = 180 / Math.PI
  var STALE_MS = 20000

  var PATHS = [
    'environment.wave.period',
    'environment.wave.encounterPeriod',
    'environment.wave.length',
    'environment.wave.celerity',
    'environment.wave.groupSpeed',
    'environment.wave.significantHeight',
    'environment.wave.directionTrue',
    'environment.wave.directionRelative',
    'environment.wave.confidence',
    'environment.wave.state',
    'environment.wave.rmsSlope',
    'environment.wave.slopeGate'
  ]

  var el = {}
  ;['link', 'banner', 'bannerLabel', 'confValue', 'bannerNote', 'height', 'heightSub', 'period',
    'encPeriod', 'length', 'celerity', 'groupSpeed', 'dirTrue', 'dirRel',
    'waveArrow', 'age', 'regime'].forEach(function (id) { el[id] = document.getElementById(id) })

  var state = {}
  var lastUpdate = 0

  function fmt (v, digits) {
    return (v === undefined || v === null || !isFinite(v)) ? '–' : v.toFixed(digits)
  }

  function deg360 (d) {
    return ((Math.round(d) % 360) + 360) % 360
  }

  function render () {
    // Wave parameters are only meaningful when not gated; blank them when calm.
    var calm = state['environment.wave.state'] === 'calm'

    el.height.textContent = fmt(calm ? null : state['environment.wave.significantHeight'], 2)
    el.period.textContent = fmt(calm ? null : state['environment.wave.period'], 1)
    el.encPeriod.textContent = fmt(calm ? null : state['environment.wave.encounterPeriod'], 1)
    el.length.textContent = fmt(calm ? null : state['environment.wave.length'], 0)

    var c = calm ? null : state['environment.wave.celerity']
    var g = calm ? null : state['environment.wave.groupSpeed']
    el.celerity.textContent = (c == null) ? '–' : fmt(c * MS_TO_KN, 1)
    el.groupSpeed.textContent = (g == null) ? '–' : fmt(g * MS_TO_KN, 1)

    var dt = calm ? null : state['environment.wave.directionTrue']
    var dr = calm ? null : state['environment.wave.directionRelative']
    el.dirTrue.textContent = (dt == null || !isFinite(dt)) ? '–' : deg360(dt * RAD_TO_DEG)
    el.dirRel.textContent = (dr == null || !isFinite(dr))
      ? '–'
      : Math.abs(Math.round(dr * RAD_TO_DEG)) + '° ' + (dr < 0 ? 'P' : 'S')
    if (dr != null && isFinite(dr)) {
      // Arrow points to where the waves come FROM, relative to the bow (up).
      el.waveArrow.setAttribute('transform', 'rotate(' + (dr * RAD_TO_DEG) + ' 60 60)')
    }
    el.heightSub.textContent = calm ? 'too calm — below the motion gate' : 'slope inversion · no heave sensor'

    setBanner()
  }

  function setBanner () {
    var st = state['environment.wave.state']
    var rms = state['environment.wave.rmsSlope']
    var gate = state['environment.wave.slopeGate']
    var conf = state['environment.wave.confidence']

    var cls = 'unknown'
    var label = 'Confidence'
    var value = '–'
    var note = 'waiting for data…'

    if (st === 'calm') {
      cls = 'calm'
      label = 'Motion'
      value = (rms == null) ? '–' : (rms * RAD_TO_DEG).toFixed(2) + '°'
      note = 'Too calm — RMS slope below the ' +
        (gate == null ? '' : (gate * RAD_TO_DEG).toFixed(1) + '° ') +
        'gate. No waves to measure.'
    } else if (st === 'lowConfidence') {
      cls = 'bad'
      value = (conf == null) ? '–' : Math.round(conf * 100) + '%'
      note = 'Low — unreliable conditions (short / confused / beam / fast following sea). Waves not published.'
    } else if (conf != null) {
      value = Math.round(conf * 100) + '%'
      if (conf >= 0.6) { cls = 'good'; note = 'Estimate is reliable.' } else if (conf >= 0.3) { cls = 'warn'; note = 'Marginal — treat height with care.' } else { cls = 'bad'; note = 'Low — height unreliable (short/confused/beam seas).' }
    }

    el.banner.className = 'banner ' + cls
    el.bannerLabel.textContent = label
    el.confValue.textContent = value
    el.bannerNote.textContent = note
  }

  function tick () {
    if (!lastUpdate) { el.age.textContent = '–'; return }
    var ageMs = Date.now() - lastUpdate
    var secs = Math.round(ageMs / 1000)
    el.age.textContent = secs < 60 ? secs + 's ago' : Math.round(secs / 60) + 'm ago'
    document.body.classList.toggle('stale', ageMs > STALE_MS)
    el.regime.textContent = ageMs > STALE_MS ? 'no recent estimate' : 'live'
  }

  function onDelta (delta) {
    if (!delta.updates) { return }
    var touched = false
    delta.updates.forEach(function (u) {
      if (!u.values) { return }
      u.values.forEach(function (v) {
        if (v.path && v.path.indexOf('environment.wave.') === 0) {
          state[v.path] = v.value
          touched = true
        }
      })
    })
    if (touched) {
      lastUpdate = Date.now()
      render()
      tick()
    }
  }

  var ws = null
  function connect () {
    var proto = location.protocol === 'https:' ? 'wss:' : 'ws:'
    var url = proto + '//' + location.host + '/signalk/v1/stream?subscribe=none'
    ws = new WebSocket(url)

    ws.onopen = function () {
      el.link.className = 'link up'
      ws.send(JSON.stringify({
        context: 'vessels.self',
        subscribe: PATHS.map(function (p) {
          return { path: p, period: 1000, policy: 'instant' }
        })
      }))
    }
    ws.onmessage = function (ev) {
      try { onDelta(JSON.parse(ev.data)) } catch (e) { /* ignore non-delta frames */ }
    }
    ws.onclose = function () {
      el.link.className = 'link down'
      setTimeout(connect, 3000)
    }
    ws.onerror = function () { try { ws.close() } catch (e) { /* ignore */ } }
  }

  setInterval(tick, 1000)
  render()
  connect()
}())
