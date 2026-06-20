'use strict'

// Minimal dependency-free radix-2 FFT plus the spectral helpers the estimator
// needs. Kept self-contained so the plugin pulls in nothing at runtime (the
// server often runs offline on board).

// In-place iterative radix-2 Cooley-Tukey. re/im are Float64Array of length N,
// N must be a power of two. Transforms in place.
function fft (re, im) {
  const n = re.length
  if (n <= 1) { return }
  if ((n & (n - 1)) !== 0) { throw new Error('fft length must be a power of two') }

  // Bit-reversal permutation.
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1
    for (; j & bit; bit >>= 1) { j ^= bit }
    j ^= bit
    if (i < j) {
      const tr = re[i]; re[i] = re[j]; re[j] = tr
      const ti = im[i]; im[i] = im[j]; im[j] = ti
    }
  }

  for (let len = 2; len <= n; len <<= 1) {
    const ang = -2 * Math.PI / len
    const wpr = Math.cos(ang)
    const wpi = Math.sin(ang)
    for (let i = 0; i < n; i += len) {
      let wr = 1
      let wi = 0
      for (let k = 0; k < len / 2; k++) {
        const a = i + k
        const b = i + k + len / 2
        const tr = wr * re[b] - wi * im[b]
        const ti = wr * im[b] + wi * re[b]
        re[b] = re[a] - tr
        im[b] = im[a] - ti
        re[a] += tr
        im[a] += ti
        const nwr = wr * wpr - wi * wpi
        wi = wr * wpi + wi * wpr
        wr = nwr
      }
    }
  }
}

// Largest power of two <= n.
function prevPow2 (n) {
  let p = 1
  while (p * 2 <= n) { p *= 2 }
  return p
}

// Mean-removed, Hann-windowed complex spectrum of a real signal. Returns the
// full-length { re, im, winPow } so callers can form a power spectrum (|X|^2) or
// a cross-spectrum between two signals that share the same window. winPow is the
// sum of w^2 used for power (variance) normalisation.
function windowedSpectrum (signal) {
  const n = signal.length
  const re = new Float64Array(n)
  const im = new Float64Array(n)

  let mean = 0
  for (let i = 0; i < n; i++) { mean += signal[i] }
  mean /= n

  let winPow = 0
  for (let i = 0; i < n; i++) {
    const w = 0.5 * (1 - Math.cos(2 * Math.PI * i / (n - 1)))
    re[i] = (signal[i] - mean) * w
    winPow += w * w
  }

  fft(re, im)
  return { re, im, winPow }
}

// One-sided power spectral density from a windowed spectrum, sampled at fs,
// normalised so that summing psd*df over the band recovers the signal variance.
// Returns { freqs, psd, df } with length N/2.
function psdFromSpectrum (spec, fs) {
  const n = spec.re.length
  const half = n / 2
  const df = fs / n
  const freqs = new Float64Array(half)
  const psd = new Float64Array(half)
  // Normalisation: PSD = |X|^2 / (fs * sum(w^2)). One-sided: double every non-DC
  // bin. The Nyquist bin (k = N/2) is dropped entirely (loop runs k < half) — it
  // sits far above the wave band, so its negligible energy does not affect the
  // in-band moments.
  const norm = 1 / (fs * spec.winPow)
  for (let k = 0; k < half; k++) {
    freqs[k] = k * df
    const mag2 = spec.re[k] * spec.re[k] + spec.im[k] * spec.im[k]
    const oneSided = (k === 0) ? 1 : 2
    psd[k] = mag2 * norm * oneSided
  }
  return { freqs, psd, df }
}

// One-sided PSD of a real signal sampled at fs (convenience wrapper).
function welchPSD (signal, fs) {
  return psdFromSpectrum(windowedSpectrum(signal), fs)
}

module.exports = { fft, prevPow2, welchPSD, windowedSpectrum, psdFromSpectrum }
