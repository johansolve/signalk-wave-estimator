'use strict'

const { prevPow2, windowedSpectrum, psdFromSpectrum } = require('./fft')

const G = 9.81

// Solve the deep-water encounter relation for the true wave angular frequency.
//   omega_e = omega - (omega^2 / g) * U * cos(mu)
// Rearranged: a*omega^2 - omega + omega_e = 0  with a = U*cos(mu)/g.
// Returns { omega, ok }. ok=false means no physical real root (e.g. following
// seas where the boat outruns the energy) and the caller should fall back.
function solveTrueOmega (omegaE, U, cosMu) {
  const a = (U * cosMu) / G
  // a -> 0 (no speed / beam seas): the correction vanishes, omega == omega_e.
  if (Math.abs(a) < 1e-9) { return { omega: omegaE, ok: true } }
  const disc = 1 - 4 * a * omegaE
  if (disc < 0) { return { omega: omegaE, ok: false } }
  // In following seas (a > 0) the encounter frequency has a maximum at
  // omega = 1/(2a); above it a measured omega_e maps to two true omegas and the
  // two roots converge as disc -> 0. There the root choice is ambiguous and the
  // solve is ill-conditioned (a small omega_e error maps to a large omega error),
  // so flag it unreliable rather than emit a confident but arbitrary period.
  if (a > 0 && disc < 0.05) { return { omega: omegaE, ok: false } }
  // Physical root: the one that -> omega_e as a -> 0 is (1 - sqrt(disc)) / (2a).
  const omega = (1 - Math.sqrt(disc)) / (2 * a)
  if (!Number.isFinite(omega) || omega <= 0) { return { omega: omegaE, ok: false } }
  return { omega, ok: true }
}

// Linear interpolation of a field over time-sorted samples at time t (ms).
function interpAt (samples, field, t) {
  // Callers walk t monotonically, but a small local scan keeps this independent.
  let lo = 0
  let hi = samples.length - 1
  if (t <= samples[lo].t) { return samples[lo][field] }
  if (t >= samples[hi].t) { return samples[hi][field] }
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1
    if (samples[mid].t <= t) { lo = mid } else { hi = mid }
  }
  const span = samples[hi].t - samples[lo].t
  const f = span > 0 ? (t - samples[lo].t) / span : 0
  return samples[lo][field] + f * (samples[hi][field] - samples[lo][field])
}

// Resample irregular samples onto a uniform power-of-two grid. Returns
// { pitch, roll, fs } or null if the span is too short.
function resample (samples, fsTarget, minN) {
  const span = samples[samples.length - 1].t - samples[0].t // ms
  const spanSec = span / 1000
  const n = prevPow2(Math.floor(spanSec * fsTarget))
  if (n < minN) { return null }
  const dt = span / (n - 1) // ms
  const pitch = new Float64Array(n)
  const roll = new Float64Array(n)
  const t0 = samples[0].t
  for (let i = 0; i < n; i++) {
    const t = t0 + i * dt
    pitch[i] = interpAt(samples, 'pitch', t)
    roll[i] = interpAt(samples, 'roll', t)
  }
  return { pitch, roll, fs: 1000 / dt }
}

// Integrate PSD over [fMin, fMax] -> variance (m0). Also returns the peak bin.
function bandStats (freqs, psd, df, fMin, fMax) {
  let m0 = 0
  let peakIdx = -1
  let peakVal = -1
  for (let k = 0; k < freqs.length; k++) {
    const f = freqs[k]
    if (f < fMin || f > fMax) { continue }
    m0 += psd[k] * df
    if (psd[k] > peakVal) { peakVal = psd[k]; peakIdx = k }
  }
  return { m0, peakIdx, peakVal }
}

// Parabolic interpolation of the peak frequency from three bins around peakIdx.
function refinePeak (freqs, psd, peakIdx, df) {
  if (peakIdx <= 0 || peakIdx >= psd.length - 1) { return freqs[peakIdx] }
  const a = psd[peakIdx - 1]
  const b = psd[peakIdx]
  const c = psd[peakIdx + 1]
  const denom = a - 2 * b + c
  const delta = denom !== 0 ? 0.5 * (a - c) / denom : 0
  return freqs[peakIdx] + delta * df
}

function clamp (x, lo, hi) { return Math.max(lo, Math.min(hi, x)) }
function norm2pi (a) {
  let x = a % (2 * Math.PI)
  if (x < 0) { x += 2 * Math.PI }
  return x
}

// Core analysis. `samples` is time-sorted [{t, pitch, roll}] (radians, ms).
// `ctx` carries the latest scalar inputs and the configuration. Returns a result
// object or null when there is not enough clean data.
function analyze (samples, ctx) {
  if (samples.length < 8) { return null }
  const rs = resample(samples, ctx.fsTarget, ctx.minSamples)
  if (!rs) { return null }

  // Keep the complex spectra: the PSDs drive the moments, and the pitch/roll
  // cross-spectrum (same window) resolves the port/starboard side below.
  const pSpec = windowedSpectrum(rs.pitch)
  const rSpec = windowedSpectrum(rs.roll)
  const pp = psdFromSpectrum(pSpec, rs.fs)
  const rp = psdFromSpectrum(rSpec, rs.fs)

  const fMin = 1 / ctx.periodMax
  const fMax = 1 / ctx.periodMin
  const pStat = bandStats(pp.freqs, pp.psd, pp.df, fMin, fMax)
  const rStat = bandStats(rp.freqs, rp.psd, rp.df, fMin, fMax)
  if (pStat.peakIdx < 0 || pStat.m0 <= 0) { return null }

  // Dominant ENCOUNTER frequency from the pitch spectrum.
  const fE = refinePeak(pp.freqs, pp.psd, pStat.peakIdx, pp.df)
  const omegaE = 2 * Math.PI * fE
  const Te = 1 / fE

  // Encounter angle magnitude off the bow axis from the slope-energy ratio:
  //   along-track slope (pitch) ~ cos(alpha),  athwart slope (roll) ~ sin(alpha)
  // so tan(alpha) = sqrt(m0_roll / m0_pitch). alpha in [0, pi/2]; sign/quadrant
  // is unknown from amplitude alone (needs roll/pitch phase — see README).
  const alpha = Math.atan2(Math.sqrt(Math.max(0, rStat.m0)), Math.sqrt(pStat.m0))
  const cosAlpha = Math.cos(alpha)

  // Port vs starboard from the pitch/roll cross-spectrum at the dominant bin.
  // For a single wave both pitch and roll oscillate co-linearly in time, so the
  // cross-spectrum is real at the peak; its SIGN flips with the side the wave
  // comes from (Re(S_pr) ~ sin(2*beta), + for one side, - for the other). Summed
  // over the peak +/-1 bins for robustness. Which sign means starboard depends on
  // the IMU mounting and the attitude sign convention, so it is calibrated by the
  // `flipSide` config after a sea trial — default: positive Re -> starboard.
  let crossRe = 0
  for (let d = -1; d <= 1; d++) {
    const idx = pStat.peakIdx + d
    if (idx >= 0 && idx < pp.psd.length) {
      crossRe += pSpec.re[idx] * rSpec.re[idx] + pSpec.im[idx] * rSpec.im[idx]
    }
  }
  let starboard = crossRe >= 0
  if (ctx.flipSide) { starboard = !starboard }

  // Head vs following: pick the sign of cos(mu). Wind direction is the fallback
  // discriminator; otherwise use the configured default regime.
  let following = ctx.defaultRegime === 'following'
  if (ctx.heading != null && ctx.windDir != null) {
    // Angle between where the wind/waves come from and the bow.
    const rel = Math.abs(((ctx.windDir - ctx.heading + Math.PI) % (2 * Math.PI)) - Math.PI)
    following = rel < Math.PI / 2 ? false : true // wind from ahead -> head seas
  }
  const cosMu = following ? cosAlpha : -cosAlpha

  const U = (ctx.stw != null && ctx.stw >= 0) ? ctx.stw : 0
  const sol = solveTrueOmega(omegaE, U, cosMu)
  const omega = sol.omega

  const T = 2 * Math.PI / omega
  const length = G * T * T / (2 * Math.PI)
  const celerity = G * T / (2 * Math.PI)
  const groupSpeed = celerity / 2
  const k = omega * omega / G // deep-water wavenumber

  // Height by slope inversion. The surface-slope vector variance equals
  // k^2 * elevation variance, so sigma_eta = sqrt(m0_pitch + m0_roll) / k and
  // the significant-height analogue is Hs = 4 * sigma_eta. Narrowband: a single
  // k (the dominant peak) is applied to the whole band — a proxy, not a measure.
  const slopeVar = pStat.m0 + Math.max(0, rStat.m0)
  const significantHeight = k > 0 ? 4 * Math.sqrt(slopeVar) / k : null

  // Relative bearing the waves come FROM (0 = dead ahead). Magnitude off the bow
  // axis is in [0, pi]; the cross-spectrum sign puts it on the correct side,
  // giving a signed angle in [-pi, pi] (+ starboard, - port), as for other SK
  // relative angles (e.g. apparent wind).
  const offBow = following ? (Math.PI - alpha) : alpha
  const directionRelative = starboard ? offBow : -offBow
  const directionTrue = ctx.heading != null ? norm2pi(ctx.heading + directionRelative) : null

  // Confidence: contouring validity (lambda vs hull), spectral sharpness,
  // encounter-solve validity, and roll-resonance risk in beam seas.
  const cLambda = clamp((length / ctx.boatLength - 1) / 2, 0, 1)
  let peakBand = 0
  for (let d = -1; d <= 1; d++) {
    const idx = pStat.peakIdx + d
    if (idx >= 0 && idx < pp.psd.length) { peakBand += pp.psd[idx] * pp.df }
  }
  const cPeak = clamp(peakBand / pStat.m0, 0, 1)
  const cReg = sol.ok ? 1 : 0.3
  const cRoll = clamp((2 * pStat.m0) / (pStat.m0 + Math.max(0, rStat.m0)), 0, 1)
  const confidence = clamp(cLambda * cPeak * cReg * cRoll, 0, 1)

  return {
    encounterPeriod: Te,
    period: T,
    length,
    celerity,
    groupSpeed,
    significantHeight,
    rmsSlope: Math.sqrt(slopeVar), // rad; raw motion energy, k-independent gate
    directionRelative,
    directionTrue,
    confidence,
    // diagnostics (not published as SK paths, handy for the status line/log)
    _fs: rs.fs,
    _samples: samples.length,
    _following: following,
    _stw: U,
    _solveOk: sol.ok
  }
}

module.exports = { analyze, solveTrueOmega, resample, G }
