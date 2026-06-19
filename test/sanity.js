'use strict'

// Synthetic sanity check: build pitch/roll from a known wave and verify the
// estimator recovers period, wavelength and the encounter-speed correction.
// No hardware needed — this only exercises the math (FFT + encounter solve).

const assert = require('assert')
const { analyze, solveTrueOmega, G } = require('../lib/wave-estimator')

function buildSamples ({ omegaE, slopeAmp, rollAmp = 0, windowSec = 120, rateHz = 11.5 }) {
  const samples = []
  const dt = 1000 / rateHz
  const n = Math.floor(windowSec * rateHz)
  const t0 = 1700000000000
  for (let i = 0; i < n; i++) {
    // Irregular arrival: jitter the timestamp a little, like a real bus.
    const jitter = ((i * 2654435761) % 1000) / 1000 - 0.5 // deterministic pseudo-jitter
    const t = t0 + i * dt + jitter * dt * 0.3
    const ph = omegaE * (t - t0) / 1000
    samples.push({ t, pitch: slopeAmp * Math.cos(ph), roll: rollAmp * Math.sin(ph) })
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
  check('beam: directionRelative near 90°', Math.abs(r.directionRelative - Math.PI / 2) < 0.5,
    `got ${(r.directionRelative * 180 / Math.PI).toFixed(0)}°`)
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

console.log(failed === 0 ? '\nAll sanity checks passed.' : `\n${failed} check(s) FAILED.`)
process.exit(failed === 0 ? 0 : 1)
