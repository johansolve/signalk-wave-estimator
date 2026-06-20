'use strict'

// Synthetic sanity check: build pitch/roll from a known wave and verify the
// estimator recovers period, wavelength and the encounter-speed correction.
// No hardware needed — this only exercises the math (FFT + encounter solve).

const assert = require('assert')
const { analyze, solveTrueOmega, narrowness, G } = require('../lib/wave-estimator')

// rollPhase (rad) sets the pitch->roll phase: 0 = in phase (resolves to one
// side), Math.PI = anti-phase (the other side), Math.PI/2 = quadrature (the
// degenerate beam case where the side is genuinely ambiguous).
// rollOmegaE lets roll oscillate at a different frequency than pitch (hull roll
// resonance). second = { omegaE, slopeAmp } adds a second pitch wave system.
function buildSamples ({ omegaE, slopeAmp, rollAmp = 0, rollPhase = 0, rollOmegaE = null, second = null, windowSec = 120, rateHz = 11.5 }) {
  const samples = []
  const dt = 1000 / rateHz
  const n = Math.floor(windowSec * rateHz)
  const t0 = 1700000000000
  const ro = rollOmegaE != null ? rollOmegaE : omegaE
  for (let i = 0; i < n; i++) {
    // Irregular arrival: jitter the timestamp a little, like a real bus.
    const jitter = ((i * 2654435761) % 1000) / 1000 - 0.5 // deterministic pseudo-jitter
    const t = t0 + i * dt + jitter * dt * 0.3
    const s = (t - t0) / 1000
    let pitch = slopeAmp * Math.cos(omegaE * s)
    if (second) { pitch += second.slopeAmp * Math.cos(second.omegaE * s) }
    samples.push({ t, pitch, roll: rollAmp * Math.cos(ro * s + rollPhase) })
  }
  return samples
}

const baseCtx = {
  fsTarget: 4,
  minSamples: 64,
  periodMin: 2,
  periodMax: 20,
  boatLength: 8.4,
  defaultRegime: 'head',
  stw: null,
  heading: null,
  windDir: null
}

let failed = 0
function check (name, cond, detail) {
  if (cond) {
    console.log(`  ok   ${name}${detail ? ' — ' + detail : ''}`)
  } else {
    failed++
    console.log(`  FAIL ${name}${detail ? ' — ' + detail : ''}`)
  }
}

// 1) At rest: encounter period == true period. T0 = 6 s, 5° slope amplitude.
{
  const T0 = 6
  const omegaE = 2 * Math.PI / T0
  const samples = buildSamples({ omegaE, slopeAmp: 5 * Math.PI / 180 })
  const r = analyze(samples, Object.assign({}, baseCtx, { stw: 0 }))
  assert(r, 'expected a result at rest')
  const lambda = 1.56 * T0 * T0
  check('rest: period', Math.abs(r.period - T0) < 0.4, `got ${r.period.toFixed(2)}s want ${T0}s`)
  check('rest: wavelength', Math.abs(r.length - lambda) < 6, `got ${r.length.toFixed(1)}m want ${lambda.toFixed(1)}m`)
  check('rest: height present', r.significantHeight > 0, `Hs≈${r.significantHeight.toFixed(2)}m`)
  check('rest: confidence high', r.confidence > 0.4, `conf ${r.confidence.toFixed(2)}`)
  // 5° amplitude sinusoid -> RMS slope = 5/sqrt(2) ≈ 3.54° (0.0617 rad).
  const rmsDeg = r.rmsSlope * 180 / Math.PI
  check('rest: rmsSlope matches amplitude', Math.abs(rmsDeg - 3.54) < 0.4, `got ${rmsDeg.toFixed(2)}° want ≈3.54°`)
}

// 2) Head seas, STW 3 m/s. Generate the SHIFTED encounter signal from a known
//    true period and verify analyze() solves back to the true period.
{
  const T0 = 9
  const omegaTrue = 2 * Math.PI / T0
  const U = 3
  // Head seas cos(mu) = -1: omega_e = omega - (omega^2/g)*U*cos(mu) = omega + ...
  const omegaE = omegaTrue - (omegaTrue * omegaTrue / G) * U * (-1)
  const samples = buildSamples({ omegaE, slopeAmp: 4 * Math.PI / 180 })
  const r = analyze(samples, Object.assign({}, baseCtx, { stw: U, defaultRegime: 'head' }))
  assert(r, 'expected a result in head seas')
  check('head: encounter < true period', r.encounterPeriod < r.period, `Te ${r.encounterPeriod.toFixed(2)} < T ${r.period.toFixed(2)}`)
  check('head: recovers true period', Math.abs(r.period - T0) < 0.7, `got ${r.period.toFixed(2)}s want ${T0}s`)
}

// 3) Beam-dominant motion (roll >> pitch) should bias the angle toward beam.
{
  const T0 = 7
  const omegaE = 2 * Math.PI / T0
  const samples = buildSamples({ omegaE, slopeAmp: 1 * Math.PI / 180, rollAmp: 6 * Math.PI / 180 })
  const r = analyze(samples, Object.assign({}, baseCtx, { stw: 0 }))
  assert(r, 'expected a result in beam seas')
  // In-phase roll resolves to starboard; the magnitude is what this case tests.
  check('beam: |directionRelative| near 90°', Math.abs(Math.abs(r.directionRelative) - Math.PI / 2) < 0.5,
    `got ${(r.directionRelative * 180 / Math.PI).toFixed(0)}°`)
}

// 3b) Port vs starboard from the pitch/roll phase. In-phase roll -> one side,
//     anti-phase -> the other; flipSide inverts the mapping.
{
  const T0 = 7
  const omegaE = 2 * Math.PI / T0
  const amp = 4 * Math.PI / 180
  const stbd = analyze(buildSamples({ omegaE, slopeAmp: amp, rollAmp: amp, rollPhase: 0 }),
    Object.assign({}, baseCtx, { stw: 0 }))
  const port = analyze(buildSamples({ omegaE, slopeAmp: amp, rollAmp: amp, rollPhase: Math.PI }),
    Object.assign({}, baseCtx, { stw: 0 }))
  check('starboard: directionRelative > 0', stbd.directionRelative > 0,
    `got ${(stbd.directionRelative * 180 / Math.PI).toFixed(0)}°`)
  check('port: directionRelative < 0', port.directionRelative < 0,
    `got ${(port.directionRelative * 180 / Math.PI).toFixed(0)}°`)
  check('starboard and port are opposite signs',
    Math.sign(stbd.directionRelative) === -Math.sign(port.directionRelative))
  const flipped = analyze(buildSamples({ omegaE, slopeAmp: amp, rollAmp: amp, rollPhase: 0 }),
    Object.assign({}, baseCtx, { stw: 0, flipSide: true }))
  check('flipSide inverts the side', flipped.directionRelative < 0,
    `got ${(flipped.directionRelative * 180 / Math.PI).toFixed(0)}°`)
}

// 4) solveTrueOmega unit behaviour.
{
  check('solve: U=0 identity', solveTrueOmega(1.0, 0, -1).omega === 1.0)
  const head = solveTrueOmega(1.0, 3, -1)
  check('solve: head root < encounter', head.ok && head.omega < 1.0, `omega ${head.omega.toFixed(3)}`)
  const bad = solveTrueOmega(2.0, 5, 1) // following, boat outruns -> no real root
  check('solve: following no-root falls back', !bad.ok)
  const nearMax = solveTrueOmega(0.78, 3, 1) // following, near encounter-freq max -> ill-conditioned
  check('solve: following near-max flagged unreliable', !nearMax.ok)
  const mildFollow = solveTrueOmega(0.3, 3, 1) // following, well below max -> solves
  check('solve: mild following solves', mildFollow.ok && mildFollow.omega > 0, `omega ${mildFollow.omega.toFixed(3)}`)
}

// 5) Narrowness (A): zero for a single tone, positive for a two-tone spectrum.
{
  // Single tone at f=0.1: m0=1, m1=f*m0, m2=f^2*m0 -> ratio exactly 1, nu=0.
  check('narrowness: single tone ~0', narrowness(1, 0.1, 0.01) < 1e-9,
    `nu=${narrowness(1, 0.1, 0.01).toExponential(1)}`)
  // Two equal tones at 0.1 and 0.2: m0=2, m1=0.3, m2=0.05 -> nu≈0.33.
  const nu2 = narrowness(2, 0.3, 0.05)
  check('narrowness: two tones > single tone', nu2 > 0.1, `nu=${nu2.toFixed(3)}`)
}

// 6) Roll-resonance discriminator (B): a beam sea (roll at the wave frequency)
//    is trusted; roll energy at a different (resonant) frequency is distrusted.
{
  const T0 = 7
  const omegaE = 2 * Math.PI / T0
  const amp = 4 * Math.PI / 180
  const ctx = Object.assign({}, baseCtx, { stw: 0 })
  const beam = analyze(buildSamples({ omegaE, slopeAmp: amp, rollAmp: amp }), ctx)
  const reso = analyze(buildSamples({ omegaE, slopeAmp: amp, rollAmp: amp, rollOmegaE: omegaE * 0.6 }), ctx)
  check('roll: beam sea trusted (cRoll high)', beam._cRoll > 0.9, `cRoll ${beam._cRoll.toFixed(2)}`)
  check('roll: off-frequency resonance distrusted', reso._cRoll < beam._cRoll - 0.2,
    `beam ${beam._cRoll.toFixed(2)} vs reso ${reso._cRoll.toFixed(2)}`)
}

// 7) Temporal stability (C): a consistent history scores higher than a jittery one.
{
  const T0 = 8
  const omegaE = 2 * Math.PI / T0
  const samples = buildSamples({ omegaE, slopeAmp: 4 * Math.PI / 180 })
  const base = analyze(samples, Object.assign({}, baseCtx, { stw: 0 }))
  const steady = []
  const jumpy = []
  for (let i = 0; i < 5; i++) {
    steady.push({ period: base.period, directionRelative: base.directionRelative })
    jumpy.push({ period: base.period * (i % 2 ? 1.4 : 0.6), directionRelative: base.directionRelative + (i % 2 ? 1 : -1) })
  }
  const stable = analyze(samples, Object.assign({}, baseCtx, { stw: 0, history: steady }))
  const unstable = analyze(samples, Object.assign({}, baseCtx, { stw: 0, history: jumpy }))
  check('stability: steady history near 1', stable._cStable > 0.9, `cStable ${stable._cStable.toFixed(2)}`)
  check('stability: jittery history penalised', unstable._cStable < 0.5, `cStable ${unstable._cStable.toFixed(2)}`)
  check('stability: steady > jittery confidence', stable.confidence > unstable.confidence)
}

// 8) Per-output confidence (D): height confidence <= overall, and strictly less
//    for a short wave (wavelength not clearly above the hull).
{
  const T0 = 3.5 // λ ≈ 19 m vs 8.4 m hull -> contouring gate < 1
  const omegaE = 2 * Math.PI / T0
  const r = analyze(buildSamples({ omegaE, slopeAmp: 4 * Math.PI / 180 }), Object.assign({}, baseCtx, { stw: 0 }))
  assert(r, 'expected a result for the short-wave case')
  check('height conf <= overall conf', r.heightConfidence <= r.confidence + 1e-9,
    `confH ${r.heightConfidence.toFixed(2)} vs conf ${r.confidence.toFixed(2)}`)
  check('short wave: height conf < overall conf', r.heightConfidence < r.confidence,
    `confH ${r.heightConfidence.toFixed(2)} vs conf ${r.confidence.toFixed(2)}`)
}

// 9) Secondary peak (E): a bimodal pitch spectrum (swell + wind sea) yields a
//    secondary system at the weaker peak's period.
{
  const T1 = 10 // dominant swell
  const T2 = 5 // weaker wind sea
  const r = analyze(buildSamples({
    omegaE: 2 * Math.PI / T1,
    slopeAmp: 4 * Math.PI / 180,
    second: { omegaE: 2 * Math.PI / T2, slopeAmp: 3 * Math.PI / 180 }
  }), Object.assign({}, baseCtx, { stw: 0 }))
  assert(r, 'expected a result for the bimodal case')
  check('secondary: present', r.secondary != null)
  check('secondary: primary near T1', Math.abs(r.period - T1) < 0.6, `got ${r.period.toFixed(2)}s want ${T1}s`)
  if (r.secondary) {
    check('secondary: period near T2', Math.abs(r.secondary.period - T2) < 0.6,
      `got ${r.secondary.period.toFixed(2)}s want ${T2}s`)
  }
  // A single-wave spectrum must NOT report a secondary.
  const single = analyze(buildSamples({ omegaE: 2 * Math.PI / T1, slopeAmp: 4 * Math.PI / 180 }),
    Object.assign({}, baseCtx, { stw: 0 }))
  check('secondary: absent for single wave', single.secondary == null)
}

console.log(failed === 0 ? '\nAll sanity checks passed.' : `\n${failed} check(s) FAILED.`)
process.exit(failed === 0 ? 0 : 1)
