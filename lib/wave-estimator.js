'use strict'

const { prevPow2, windowedSpectrum, psdFromSpectrum } = require('./fft')

const G = 9.81

// --- Tuning constants -------------------------------------------------------
// Narrowness -> confidence mapping. nu = sqrt(m0*m2/m1^2 - 1) is 0 for a pure
// tone and grows for a broadband (confused/anchor) spectrum; exp(-nu) maps it to
// (0,1]. A clean swell sits around nu ~ 0.2-0.4, confused chop nu >~ 0.8.
const NARROW_DECAY = 1.0
// Roll-resonance discriminator: how sharply a roll peak at a frequency away from
// the pitch peak is treated as resonance rather than real beam-sea signal. The
// match weight is exp(-(df/FREQ)^2) with df the relative frequency offset.
const ROLL_FREQ_TOL = 0.25
// Temporal stability references: a period coefficient-of-variation of CV_REF or a
// direction circular std of DIR_STD_REF (rad) drops that factor to 1/e.
const CV_REF = 0.25
const DIR_STD_REF = 0.5
// Secondary (bimodal) peak: a candidate must reach SEC_MIN_RATIO of the primary
// peak density, sit at least SEC_MIN_SEP away in relative frequency, and its slope
// ratio is measured in a +/- SEC_BAND_HALF relative-frequency window around it.
const SEC_MIN_RATIO = 0.25
const SEC_MIN_SEP = 0.4
const SEC_BAND_HALF = 0.2

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

// Integrate PSD over [fMin, fMax] -> spectral moments m0..m2 and the peak bin.
// m0 is the in-band variance; m1/m2 are the frequency-weighted moments used for
// the spectral-width (narrowness) confidence.
function bandStats (freqs, psd, df, fMin, fMax) {
  let m0 = 0
  let m1 = 0
  let m2 = 0
  let peakIdx = -1
  let peakVal = -1
  for (let k = 0; k < freqs.length; k++) {
    const f = freqs[k]
    if (f < fMin || f > fMax) { continue }
    const e = psd[k] * df
    m0 += e
    m1 += f * e
    m2 += f * f * e
    if (psd[k] > peakVal) { peakVal = psd[k]; peakIdx = k }
  }
  return { m0, m1, m2, peakIdx, peakVal }
}

// Spectral narrowness nu = sqrt(m0*m2/m1^2 - 1): 0 for a single tone, growing
// with bandwidth. Returns Infinity when the moments are degenerate.
function narrowness (m0, m1, m2) {
  if (m0 <= 0 || m1 <= 0) { return Infinity }
  const r = (m0 * m2) / (m1 * m1) - 1
  return Math.sqrt(Math.max(0, r))
}

// Real part of the pitch/roll cross-spectrum summed over the peak +/-1 bins. Its
// sign resolves the port/starboard side (see analyze).
function crossReAt (pSpec, rSpec, idx, half) {
  let s = 0
  for (let d = -1; d <= 1; d++) {
    const i = idx + d
    if (i >= 0 && i < half) {
      s += pSpec.re[i] * rSpec.re[i] + pSpec.im[i] * rSpec.im[i]
    }
  }
  return s
}

// Temporal stability over the recent history of estimates: low when the period
// or direction jitters between cycles. history is [{period, directionRelative}]
// (most recent excluded current); returns 1 (neutral) until enough history.
function stability (history, period, dirRel) {
  if (!history || history.length < 2) { return 1 }
  const periods = history.map((h) => h.period).concat(period)
  let mean = 0
  for (const p of periods) { mean += p }
  mean /= periods.length
  let varSum = 0
  for (const p of periods) { varSum += (p - mean) * (p - mean) }
  const cv = mean > 0 ? Math.sqrt(varSum / periods.length) / mean : 1
  const cStableT = Math.exp(-cv / CV_REF)

  const dirs = history.map((h) => h.directionRelative).concat(dirRel)
  let sx = 0
  let sy = 0
  for (const a of dirs) { sx += Math.cos(a); sy += Math.sin(a) }
  const R = Math.sqrt(sx * sx + sy * sy) / dirs.length
  const circStd = Math.sqrt(Math.max(0, -2 * Math.log(Math.max(R, 1e-9))))
  const cStableD = Math.exp(-circStd / DIR_STD_REF)

  return clamp(cStableT * cStableD, 0, 1)
}

// Full wave parameters for one spectral peak: invert encounter->true frequency
// for boat speed and derive period/length/celerity and the signed direction.
// `alpha` is the encounter-angle magnitude off the bow, `crossRe` the cross-
// spectrum sign at the peak. Returns ok=false when the encounter solve is
// unreliable (see solveTrueOmega).
function wavesFromPeak (peakF, alpha, crossRe, following, U, ctx) {
  const omegaE = 2 * Math.PI * peakF
  const cosMu = following ? Math.cos(alpha) : -Math.cos(alpha)
  const sol = solveTrueOmega(omegaE, U, cosMu)
  const omega = sol.omega
  const T = 2 * Math.PI / omega
  const length = G * T * T / (2 * Math.PI)
  const celerity = G * T / (2 * Math.PI)
  const groupSpeed = celerity / 2
  const k = omega * omega / G
  let starboard = crossRe >= 0
  if (ctx.flipSide) { starboard = !starboard }
  const offBow = following ? (Math.PI - alpha) : alpha
  const directionRelative = starboard ? offBow : -offBow
  const directionTrue = ctx.heading != null ? norm2pi(ctx.heading + directionRelative) : null
  return { period: T, length, celerity, groupSpeed, k, directionRelative, directionTrue, ok: sol.ok }
}

// Strongest local maximum in the pitch PSD that is clearly distinct from the
// primary peak (a second wave system: swell vs wind sea). Returns { idx, val }
// or { idx: -1 } when there is no significant secondary peak.
function findSecondaryPeak (freqs, psd, fMin, fMax, primaryF, primaryVal) {
  let bestIdx = -1
  let bestVal = -1
  for (let k = 1; k < psd.length - 1; k++) {
    const f = freqs[k]
    if (f < fMin || f > fMax) { continue }
    if (psd[k] < psd[k - 1] || psd[k] < psd[k + 1]) { continue } // local max only
    if (Math.abs(f - primaryF) < SEC_MIN_SEP * primaryF) { continue } // too near primary
    if (psd[k] < SEC_MIN_RATIO * primaryVal) { continue } // not significant
    if (psd[k] > bestVal) { bestVal = psd[k]; bestIdx = k }
  }
  return { idx: bestIdx, val: bestVal }
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
  const half = pp.psd.length
  const pStat = bandStats(pp.freqs, pp.psd, pp.df, fMin, fMax)
  const rStat = bandStats(rp.freqs, rp.psd, rp.df, fMin, fMax)
  if (pStat.peakIdx < 0 || pStat.m0 <= 0) { return null }

  // Dominant ENCOUNTER frequency from the pitch spectrum.
  const fE = refinePeak(pp.freqs, pp.psd, pStat.peakIdx, pp.df)
  const Te = 1 / fE

  // Encounter angle magnitude off the bow axis from the slope-energy ratio:
  //   along-track slope (pitch) ~ cos(alpha),  athwart slope (roll) ~ sin(alpha)
  // so tan(alpha) = sqrt(m0_roll / m0_pitch). alpha in [0, pi/2]; sign/quadrant
  // is unknown from amplitude alone (needs roll/pitch phase — see README).
  const alpha = Math.atan2(Math.sqrt(Math.max(0, rStat.m0)), Math.sqrt(pStat.m0))

  // Port vs starboard from the pitch/roll cross-spectrum at the dominant bin.
  // For a single wave both pitch and roll oscillate co-linearly in time, so the
  // cross-spectrum is real at the peak; its SIGN flips with the side the wave
  // comes from (Re(S_pr) ~ sin(2*beta)). Which sign means starboard depends on the
  // IMU mounting and the attitude sign convention, so it is calibrated by the
  // `flipSide` config after a sea trial — default: positive Re -> starboard.
  const crossRe = crossReAt(pSpec, rSpec, pStat.peakIdx, half)

  // Head vs following: pick the sign of cos(mu). Wind direction is the fallback
  // discriminator; otherwise use the configured default regime.
  let following = ctx.defaultRegime === 'following'
  if (ctx.heading != null && ctx.windDir != null) {
    // Angle between where the wind/waves come from and the bow.
    const rel = Math.abs(((ctx.windDir - ctx.heading + Math.PI) % (2 * Math.PI)) - Math.PI)
    following = rel < Math.PI / 2 ? false : true // wind from ahead -> head seas
  }

  const U = (ctx.stw != null && ctx.stw >= 0) ? ctx.stw : 0
  const prim = wavesFromPeak(fE, alpha, crossRe, following, U, ctx)

  // Height by slope inversion. The surface-slope vector variance equals
  // k^2 * elevation variance, so sigma_eta = sqrt(m0_pitch + m0_roll) / k and
  // the significant-height analogue is Hs = 4 * sigma_eta. Narrowband: a single
  // k (the dominant peak) is applied to the whole band — a proxy, not a measure.
  const slopeVar = pStat.m0 + Math.max(0, rStat.m0)
  const significantHeight = prim.k > 0 ? 4 * Math.sqrt(slopeVar) / prim.k : null

  // ---- Confidence -----------------------------------------------------------
  // Spectral sharpness from the narrowness nu (principled, bin-width independent)
  // rather than a raw peak-bin energy fraction.
  const nu = narrowness(pStat.m0, pStat.m1, pStat.m2)
  const cNarrow = Math.exp(-nu / NARROW_DECAY)

  // Encounter-solve validity.
  const cReg = prim.ok ? 1 : 0.3

  // Roll-resonance discriminator. Genuine beam seas have roll AND pitch peaking at
  // the SAME (wave) frequency, so high roll energy there is real signal. Roll
  // energy peaking at a DIFFERENT frequency is hull roll resonance contaminating
  // the slope ratio — distrust it. Penalty scales with how much roll dominates and
  // how far its peak sits from the pitch peak; in-band beam seas are not penalised.
  let cRoll = 1
  if (rStat.peakIdx >= 0 && rStat.m0 > 0) {
    const fRoll = refinePeak(rp.freqs, rp.psd, rStat.peakIdx, rp.df)
    const rollFrac = rStat.m0 / (pStat.m0 + rStat.m0)
    const dfRel = Math.abs(fRoll - fE) / fE
    const freqMatch = Math.exp(-(dfRel / ROLL_FREQ_TOL) * (dfRel / ROLL_FREQ_TOL))
    cRoll = clamp(1 - rollFrac * (1 - freqMatch), 0, 1)
  }

  // Temporal stability across recent cycles.
  const cStable = stability(ctx.history, prim.period, prim.directionRelative)

  // General confidence (period / direction): well-posed even for short waves, so
  // it deliberately omits the wavelength-vs-hull gate.
  const confidence = clamp(cNarrow * cReg * cRoll * cStable, 0, 1)
  // Height is the weakest output: it additionally needs the hull to contour the
  // wave (lambda >> waterline), so its confidence is always <= the general one.
  const cLambda = clamp((prim.length / ctx.boatLength - 1) / 2, 0, 1)
  const heightConfidence = clamp(confidence * cLambda, 0, 1)

  // ---- Secondary (bimodal) wave system --------------------------------------
  // A second pitch-PSD peak (e.g. swell under a wind sea) the single-component
  // model would otherwise smear into the primary. Its slope ratio is measured in a
  // narrow band around its own peak so the primary system does not bias the angle.
  // It inherits the primary's head/following regime and the flipSide/heading
  // calibration, so its direction shares any primary side mis-resolution.
  let secondary = null
  const sec = findSecondaryPeak(pp.freqs, pp.psd, fMin, fMax, fE, pStat.peakVal)
  if (sec.idx >= 0) {
    const fSec = refinePeak(pp.freqs, pp.psd, sec.idx, pp.df)
    const lo = fSec * (1 - SEC_BAND_HALF)
    const hi = fSec * (1 + SEC_BAND_HALF)
    const psSec = bandStats(pp.freqs, pp.psd, pp.df, lo, hi)
    const rsSec = bandStats(rp.freqs, rp.psd, rp.df, lo, hi)
    const alphaSec = Math.atan2(Math.sqrt(Math.max(0, rsSec.m0)), Math.sqrt(Math.max(psSec.m0, 1e-12)))
    const crossSec = crossReAt(pSpec, rSpec, sec.idx, half)
    const w = wavesFromPeak(fSec, alphaSec, crossSec, following, U, ctx)
    if (w.ok) {
      secondary = {
        period: w.period,
        length: w.length,
        directionRelative: w.directionRelative,
        directionTrue: w.directionTrue,
        confidence: clamp(sec.val / pStat.peakVal, 0, 1)
      }
    }
  }

  return {
    encounterPeriod: Te,
    period: prim.period,
    length: prim.length,
    celerity: prim.celerity,
    groupSpeed: prim.groupSpeed,
    significantHeight,
    rmsSlope: Math.sqrt(slopeVar), // rad; raw motion energy, k-independent gate
    directionRelative: prim.directionRelative,
    directionTrue: prim.directionTrue,
    confidence,
    heightConfidence,
    secondary,
    // diagnostics (not published as SK paths, handy for the status line/log)
    _fs: rs.fs,
    _samples: samples.length,
    _following: following,
    _stw: U,
    _solveOk: prim.ok,
    _narrowness: nu,
    _cRoll: cRoll,
    _cStable: cStable
  }
}

module.exports = { analyze, solveTrueOmega, resample, narrowness, G }
