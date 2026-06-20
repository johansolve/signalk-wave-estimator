# IMU-based sea-state estimation — knowledge bank

Research and design notes for the future development of `signalk-wave-estimator`.
Source material: a deep read of
[bareboat-necessities/ocean-imu](https://github.com/bareboat-necessities/ocean-imu)
(2026-06-20), compared against the plugin's current pitch/roll-based algorithm.

The purpose is twofold: (1) document how a "real" heave-based system works, so we
know what the path beyond the slope proxy costs and buys, and (2) distil what can
be improved in the current plugin **without** a heave sensor.

---

## 1. The problem: no heave on the bus

The EV-1 only puts attitude on N2K (PGN 127257, pitch/roll/yaw), never vertical
acceleration or heave (127252). Nothing on board provides heave (a RuuviTag won't
do: the LIS2DH12 has no gyro → it cannot separate vertical acceleration from the
gravity projection under tilt, and ~1 Hz BLE is too slow for the wave band). The
plugin therefore lives with a **slope-inversion proxy**: wave height is derived
from slope energy plus the wavelength, not from measured vertical motion. It is a
proxy, not a measurement — see `lib/wave-estimator.js` (the significantHeight
block).

ocean-imu attacks exactly this gap from the other direction: it *reconstructs*
heave from accelerometer data. It is complementary to our plugin, not a
competitor — see §5.

---

## 2. ocean-imu in brief

A mature C++ project (BBN/OpenPlotter ecosystem) that takes IMU acceleration and
reconstructs heave, attitude and wave parameters, on a microcontroller.

### Hardware
- **A single board: M5Stack AtomS3R** (ESP32-S3). All sketches (`sensors/`) run on it.
- IMU chip (via `M5Unified`, not visible in the code): **Bosch BMI270** (6-axis
  accel+gyro) + **Bosch BMM150** (magnetometer, hung off the BMI270 sensor hub →
  "9-axis").
- Consumer/wearable-grade MEMS, **not** navigation grade. The point is that the
  *filter*, not the sensor grade, does the work.
- OU-II sketch: main loop **200 Hz**, magnetometer ~25 Hz, **NMEA-0183** out, 115200 baud.
- No SignalK path in the project — output is NMEA-0183.

### Two architectures to keep apart
- **On-node (ocean-imu):** raw IMU → marine Kalman on the ESP32 → heave/waves out
  as NMEA. You just consume the result; you own no math.
- **In-plugin (ours):** SignalK deltas (attitude) → FFT in JS on the Pi →
  `environment.wave.*`. You own all the math but lack the vertical source.

---

## 3. Drift correction (`doc/kalman_ou_iii`) — getting heave from acceleration

The paper: *OU-Driven Quaternion MEKF for Marine INS and Wave-State Estimation*
(draft). A quaternion **MEKF** (left-multiplicative) with 21 states.

### State vector (21)
```
x = [ δθ  b_g  v  p  S  a_w  b_a ]ᵀ      (seven 3-vectors)
```
| | dim | meaning |
|---|---|---|
| δθ | 3 | attitude error (MEKF correction) |
| b_g | 3 | gyro bias |
| v | 3 | velocity (NED) |
| p | 3 | **heave** (down axis) |
| S | 3 | **integral of displacement**, S = ∫∫∫a |
| a_w | 3 | latent world acceleration, OU process |
| b_a | 3 | accelerometer bias |

The kinematics are a **triple integrator**: `v̇ = a_w`, `ṗ = v`, `Ṡ = p`.
`S` exists *only* to anchor drift.

### The OU process (a convenience prior, NOT a wave model)
World acceleration is modelled as an Ornstein-Uhlenbeck process:
```
ȧ_w = -(1/τ)·a_w + √(2/τ)·L_a·w(t)       (per axis: da = -(1/τ)a dt + √q_c dW, q_c = 2σ²/τ)
```
Frequency response: `S_a(ω) = 2σ²τ / (1 + ω²τ²)` — a **first-order low-pass** on
white acceleration, corner ~1/τ. The authors are unusually honest: this is *not* a
physical wave model (real wave spectra are band-pass/JONSWAP, not low-pass), just a
"low-order stationary Gauss-Markov prior" chosen because it is Markov,
mean-reverting and has a closed-form discretisation. Quality hinges entirely on
tuning τ to the dominant period.

### The drift killer: a fictitious pseudo-measurement (this is the crux)
OU mean-reversion bounds the *acceleration* variance but does **not** solve drift —
integrating zero-mean acceleration twice still yields an unbounded random walk in `p`.

The actual drift correction is a **pseudo-measurement `z_S = 0`** on the third integral:
```
z_S = 0 = S + n_S,    R_S = diag(σ²...)
```
No sensor behind it — a soft constraint "the long-term integral of displacement
should be zero." It turns the pure triple integrator into a **leaky integrator** =
an effective **high-pass** on heave. So:
- OU **low-passes** the acceleration,
- the pseudo-measurement **high-passes** the displacement,
- together they **band-limit** heave.

`R_S` is the one important drift knob (small → strong leak, kills drift but
over-damps long swell; large → weak leak, drift less suppressed). The clever bit:
they scale `R_S ∝ σ_a·τ³` (τ³ is the triple-integral's unit) so the high-pass
corner tracks the sea state → constant *relative* drift suppression regardless of
wave height.

### The honest limitation
The authors admit it: `S` is **not observable** from real sensors; the system is
only "effectively detectable", and `R_S` is "a tuning parameter rather than a
physical noise level". **It all rests on the zero-mean, stationary-sea
assumption.** Any genuine sustained vertical motion (a wave you ride for a long
time, an actual level change) is partly eaten by the `z_S = 0` constraint.
Initialisation avoids "wave accel as gravity" with a tilt-first startup + a **slow
acceleration gate** (waits for a calm window before locking down) and staged
enabling (the heave block is off during warm-up).

> Caveat: the OU-II (18 states) vs OU-III (21) distinction is **not** in the
> source — it comes from the README and was inferred. Don't trust it without
> reading the OU-II paper.

---

## 4. Hs and sea metrics (`doc/spectrum`) — the surprise

**There is no FFT, no Welch PSD, no `S(f)` at all.** The spectrum paper is a
literature survey of *definitions*; the charts are synthetic PM/JONSWAP. The actual
computation is fundamentally different from our plugin:

- An **instantaneous-frequency tracker** (zero-crossing / Aranovskiy / PLL /
  notch-Kalman, interchangeable) gives `ω(t)`, plus an envelope power `P_η(t)`.
- Moments as **running expectations**: `Mₙ = ⟨P_η(t)·ω(t)ⁿ⟩` — no spectrum, no binning.
- `Hs = 4√m0`, but `m0 = ⟨P_η⟩ ≈ heave variance`, so it **collapses in practice to
  `Hs ≈ 4σ`** of the heave time series.
- Periods from moments: `Tz = 2π√(m0/m2)`, `T1 = 2π m0/m1`, `Te = 2π m_{-1}/m0`.
- Spectral width / narrowness from moments: `ν = √(m0·m2/m1² − 1)`,
  `ε = √(1 − m1²/(m0·m2))`, Ochi peakedness `Qp = m0·m4/m2²`.
- A Jensen bias correction on the noisy `ω` (relevant precisely because moments are
  built from a jittery instantaneous frequency, not an FFT).

### What ocean-imu does NOT do
- **No true Tp** (no spectral peak to locate) — only the tracker's dominant frequency.
- **Narrowband, single component** — a mixed sea (swell + wind sea) cannot be
  represented; you get one smeared frequency and one smeared height.
- **No wave direction at all** — the paper integrates direction away (`S(ω)=∫S(ω,θ)dθ`).

---

## 5. ocean-imu vs signalk-wave-estimator

They overlap less than you'd think — they are **complementary**:

| | **Our plugin** | **ocean-imu** |
|---|---|---|
| Input | attitude (pitch/roll) from EV-1 | raw accel/gyro/mag from BMI270 |
| Method | frequency domain (FFT/Welch PSD) | time domain (Kalman + instantaneous frequency) |
| Height | slope-inversion **proxy** (weak) | true `4σ` from reconstructed heave (**stronger**) |
| Direction | pitch/roll cross-spectrum (**we have it**) | **absent** |
| Mixed sea | multiple bins possible (FFT) | no, one component |
| Tp | parabolic peak interpolation | no |
| Where it runs | Pi (Node.js) | ESP32 (on node) |

**Conclusion:** ocean-imu solves what we're weakest at (height) and entirely lacks
what we just got good at (direction). The height strength has a circularity, though:
`Hs = 4σ` of a heave reconstructed under the zero-mean stationary-sea assumption
(`z_S = 0`) — drift control and height measurement share the same assumption.
Validate absolute numbers against an independent source (a wave buoy, weather data);
relative trends are probably fine.

---

## 6. Hardware choice if we want true heave: AtomS3R vs BNO085

| | **AtomS3R** (BMI270+BMM150) | **BNO085** |
|---|---|---|
| Type | Complete board (ESP32-S3 + IMU + screen) | Bare breakout + own MCU |
| Fusion | None — raw data, fusion on host | On chip (CEVA SH-2 MotionEngine) |
| Gives directly | raw values | quaternion + **linear accel** (gravity removed) |
| ocean-imu support | **Yes, out of the box** | no, needs porting |
| Price | ~$15–20 complete | ~$20 breakout + ESP32 |

Key insight: the BNO085's selling point (on-chip fusion delivers ready linear
acceleration) is **exactly** what ocean-imu warns against — generic drone/AR fusion
filters assume the long-term mean of acceleration equals gravity, which breaks in
sustained wave motion ("marine AHRS cannot reuse typical drone filters unchanged").
So the convenience can become a liability in a seaway. The BNO085 does also expose
*raw* outputs, so it can be used as a plain 9-DoF sensor feeding ocean-imu's filter
(but then you lose the convenience).

**Recommendation if/when true heave is wanted:** AtomS3R + ocean-imu (path 1) —
cheap, complete, already supported, proven marine drift handling. The BNO085 route
(an old memory suggestion) was right *when we assumed we'd build the fusion
ourselves*; ocean-imu has already solved the marine fusion on cheap hardware, so the
convenience-fusion chip buys less.

---

## 7. Adjusting the current algorithm (without a heave sensor)

Most of ocean-imu assumes acceleration integration and is **not** applicable to our
attitude-based plugin. But the sea-metrics part has portable ideas.

> **Status 2026-06-20: A–E implemented.** Confidence is now split (`confidence` for
> period/direction, `heightConfidence` for height), narrowness `ν` replaces the old
> `cPeak`, roll resonance is resolved via the roll-vs-pitch peak frequency, temporal
> stability exists (history buffer in `index.js`), and a secondary peak is published
> under `environment.wave.secondary.*`. Verified by `npm test` (30 checks). NOT
> sea-trialled. The details below describe the design.

### 7.1 Worth borrowing (ranked)

**A. Principled spectral width as the confidence metric (HIGH priority).**
The old `cPeak` was just peak-±1-bin energy / m0 — window- and bin-width dependent,
ad hoc. ocean-imu's narrowness `ν = √(m0·m2/m1² − 1)` is dimensionless and standard.
`bandStats` now also accumulates m1, m2 (cheap) and derives ν. A broadband sea
(anchor, broadband roll) → high ν → low confidence. This addresses the anchor case
we saw (roll-dominated yawing → should be low confidence) more robustly than the old
peak fraction.

**B. Better roll-resonance discriminator (HIGH/MEDIUM).**
The old `cRoll` only penalised roll energy >> pitch energy. But in a genuine beam sea
(α near 90°) high roll energy is *legitimate signal*, not resonance — so the old
`cRoll` over-penalised real beam seas. Better discriminator: **compare the roll
dominant frequency to the pitch one.** Coincide → real wave, trust roll. Roll peaks
at a *different* (hull-resonant) frequency → resonance, distrust roll. This separates
resonance from genuine beam seas, which the energy ratio alone cannot.

**C. Temporal stability as confidence (MEDIUM).**
ocean-imu Section 14: "temporal stability index (running variance of Hs)." Each
`analyze()` was independent. Track the variance of the dominant period/direction over
the last N windows; unstable → drop confidence. Catches transient false locks a
single window can't see.

**D. Per-output confidence (MEDIUM/LOW).**
There used to be one confidence for everything. Height is fundamentally the weakest
(a proxy). Now `significantHeight` carries a separate, lower confidence
(`heightConfidence = confidence × cLambda`) than period/direction, so consumers can
trust period/direction even when the height is shaky.

### 7.2 Our latent strength that ocean-imu lacks

**E. Multi-component / secondary-peak detection (MEDIUM, real feature).**
We have the whole spectrum; ocean-imu has a single component and *cannot* separate
swell from wind sea. Detect a second peak in the pitch PSD and report it separately.
This is genuine value *beyond* what a heave node gives.

### 7.3 What is NOT applicable (so nobody chases it)

- **Instantaneous-frequency tracker** (Aranovskiy/PLL/notch): FFT on the Pi is cheap
  and — crucially — we *need* the cross-spectrum for direction, which a scalar
  frequency tracker can't give. Keep the FFT.
- **Jensen bias correction on ω:** N/A. The FFT gives clean spectral moments; the
  problem only exists in their jittery instantaneous frequency.
- **OU prior + `z_S = 0` leaky integral:** relevant only with acceleration
  integration. We don't integrate acceleration, so there's no drift to tame on that
  path. (This also refutes the old memory idea of a frequency-domain `(2πf)⁴`
  integration — that path becomes relevant only *with* a heave/accel source, not in
  the current attitude plugin.)
- **Wave-aware init / accel gate:** the EV-1 already does the attitude fusion; we
  consume finished attitude. Init isn't our problem.

---

## 8. Upgrade paths (with a heave sensor)

If an AtomS3R node is fitted on board, two architectures exist:

1. **Pure on-node:** the AtomS3R runs ocean-imu → `environment.heave` + wave height
   over NMEA → SignalK. Our plugin keeps the **direction** from attitude (which the
   node lacks). Least own math. Height and direction from two sources.
2. **Hybrid in plugin (more in our spirit):** the plugin *consumes*
   `environment.heave` if present and runs our own frequency-domain math on **real**
   heave (`Hs = 4√m0_heave` from the heave PSD) instead of the slope proxy — while
   keeping direction and multi-component spectra. That gives true height *and*
   direction *and* mixed seas in one, more than either source gives alone.

Design implication now: keep the height computation isolated (`wave-estimator.js`,
the significantHeight block) so the slope proxy can be swapped for a heave-PSD path
without touching the rest. The SignalK path for heave is `environment.heave` (m),
under `environment`.

### One IMU for pitch/roll/heave → a true directional spectrum

If a heave IMU is fitted: **take pitch/roll *and* heave from the same sensor**, not
heave from the IMU + attitude from the EV-1. The reason is **not** "same centre of
motion" as one first assumes:

- **Rotations (roll/pitch/yaw) are the same everywhere on a rigid hull** —
  independent of measurement point. So for attitude, co-location is irrelevant.
- **Heave depends on the point** via the lever arm (`heave_point = heave_CoM +
  rotation × leverarm`), but a 9-DoF IMU corrects this with its *own* gyro — it
  doesn't need the EV-1's attitude. So "same centre of motion" really means mounting
  *the IMU itself* near the vessel's centre of motion (low, central) for clean heave.

The decisive factor is instead **phase and time coherence.** Anything you
cross-correlate must share a clock and axes; two sensors = two timebases / latencies
/ calibrations → phase errors between heave and slope, and phase is exactly what
direction is built on. One sensor unlocks the real prize: with phase-coherent
**heave + pitch + roll** you can run the classic **pitch-roll-heave buoy method**
(Datawell-style, Fourier coefficients `a1/b1/a2/b2`) → a **true directional
spectrum**, not just a side. That beats both our current cross-spectrum trick and
ocean-imu (which throws direction away).

**Hybrid for heading:** a cheap IMU's yaw/magnetometer on board may be worse than the
boat's main compass (hard/soft iron). So take the *slow/absolute* part (heading →
`directionTrue`) from the main compass and the *fast/oscillatory* part (wave-band
pitch/roll/heave) from the IMU. The phase coherence where it matters (heave↔slope) is
then still the same sensor. In practice the EV-1 isn't ripped out system-wide — it's
only about what the *wave plugin* subscribes to.

### Known simplification in secondary detection

The secondary system (E) inherits the primary's head/following regime and the
`flipSide`/heading calibration. Reasonable for a single-IMU split, but it means the
secondary direction shares any error in the primary's side resolution. If a sea trial
shows the secondary direction is unreliable, this is the cause.

---

## 9. Open questions / to validate

- **The side sign (flipSide):** could not be verified at anchor (roll dominance →
  `lowConfidence`, conf ~0.09). Calibrate under way — reference observation: waves
  ~210° true under way.
- **ocean-imu's absolute height:** circular assumption (§5) — validate against an
  independent source before trusting absolute figures.
- **AtomS3R drift correction on board:** how well the `z_S = 0` constraint holds on
  Libelle is unproven — needs an on-board test before trusting the heave node.

---

## Sources

- ocean-imu: <https://github.com/bareboat-necessities/ocean-imu> (`doc/kalman_ou_iii`,
  `doc/spectrum`, `doc/freq`, `sensors/`)
- BNO08x datasheet (CEVA): <https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf>
- AtomS3R (M5Stack): <https://docs.m5stack.com/en/core/AtomS3R>
- Related: the plugin's own `README.md`.
