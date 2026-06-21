# signalk-wave-estimator

Derive sea state from boat motion. The plugin watches `navigation.attitude`
(pitch/roll) plus `navigation.speedThroughWater`, estimates the dominant wave
**encounter period** by FFT of the pitch oscillation, corrects it for boat speed
using the deep-water encounter relation, and publishes true wave period,
wavelength, celerity and a **height proxy** under `environment.wave.*`.

![Wave estimator webapp](docs/webapp.png)

> **Alpha — first sea trial 2026-06-21 (Libelle, Elan 333).** Direction is
> validated; period/wavelength/height needed the fix described in *Sea trial*
> below and have **not** yet been re-validated on the water. Wave **height is a
> proxy**, not a measurement — there is no heave/vertical-acceleration sensor on
> the bus, so height is inferred from wave *slope* and carries a confidence value.
> For short coastal wind sea (wavelength barely above the hull) the height proxy
> is only indicative; the webapp shows it as `~x.x`. See *Sea trial* and *Height*
> below.

## Sea trial — 2026-06-21 (Libelle)

First on-water run, ~2.5 h, recorded at the native 10 Hz attitude rate and
replayed offline against the live code. Sea was a coastal wind sea, waves from
**~270° true**, ≤ ~0.7 m, wind ~300°.

- **Direction works.** With waves from a known 270°, the published
  `directionTrue` clustered at a circular mean of **282°** (62 % within ±45°);
  flipping the port/starboard side destroyed that clustering (33 %, R 0.63→0.13).
  Side agreement on beam-ish legs was **77 %**, head/following regime **79 %**.
  `flipSide = false` is therefore **confirmed correct** — no change.
- **Height (and wavelength) were grossly over-read** — median ~1.6 m, peaks > 10 m
  against a real ≤ 0.7 m sea. Root cause: not the height formula (measured RMS
  slope ~2.2° is consistent with ≤ 0.7 m) but an **over-estimated period**.
  Since `Hs ∝ T²` and `λ ∝ T²`, a ~2× period error inflates both ~4×.
- **Why the period was too long:** a sailboat under way has broadband
  *low-frequency* roll/heel motion (slow rolling, heeling, course changes) that
  is **not** wave slope. With the old `periodMax = 20 s` the spectral peak landed
  on it (often 6–18 s, even 30 s when nearly stationary). This is **not** a sharp
  notchable roll resonance — the contaminating roll energy is broadband from
  ~5 s out to the band edge — and it cannot be separated from a genuine beam sea
  in a single window (an amplitude, phase or frequency mask also kills real beam
  seas). The robust fix is to **keep the band tight**.
- **Fix applied:** `periodMax` default **20 → 6 s** (captures coastal wind sea,
  excludes the low-frequency contamination). On the trial recording this pulls
  published height to median 0.3 m / p90 ~1 m / max ~1.7 m, and wavelength from a
  52 m median to ~22 m. The webapp now renders height as an explicit `~x.x`
  proxy, muted when its confidence is low. Re-validation on the water is pending.

## Why height is only a proxy

Pitch and roll follow the wave **slope**, not the surface elevation. For a wave
`η = a·cos(kx − ωt)` the slope amplitude is `a·k`, so

```
height H = 2a ≈ slope · λ / π          (single dominant wave)
```

i.e. recovering height from slope needs the **wavelength**, which the plugin
computes from the period. Spectrally, the slope-variance spectrum equals
`k²` × the elevation-variance spectrum, so the significant-height analogue is

```
Hs = 4·σ_elev = 4·√(m0_pitch + m0_roll) / k
```

with `m0` the in-band slope variance and `k` the dominant wavenumber. This is a
**narrowband** inversion (one `k` applied across the band) and assumes the hull
*contours* the wave — valid only when the wavelength clearly exceeds the
waterline length. A real, calibration-free height would need double-integrated
heave acceleration from an MRU/IMU. (Full background and the algorithm survey are
in the project HANDOFF notes.)

### When to believe the height

| Condition | Height quality |
|---|---|
| Wavelength ≫ hull (λ ≳ 3·LWL), head/following seas, sharp spectral peak | usable proxy |
| Wavelength ≈ hull, or confused/short chop | unreliable — height confidence drops |
| Beam seas (roll-dominated) | genuine beam seas now pass (roll & pitch peak at the same frequency); only roll energy at a *different* (hull-resonant) frequency drops confidence |

Confidence is split per output. `environment.wave.confidence` (0–1) is for
**period and direction** and folds in spectral **narrowness**
(`ν = √(m0·m2/m1² − 1)`), encounter-solve validity, a **roll-resonance**
discriminator (roll energy at a frequency away from the pitch peak is treated as
hull resonance, not beam-sea signal) and **temporal stability** across recent
cycles. `environment.wave.heightConfidence` is for the **height** specifically:
it is `confidence ×` the wavelength-vs-hull contouring gate, so it is always ≤ the
general confidence (height is the weakest output). Estimates whose general
confidence is below **Minimum confidence** are suppressed rather than published as
noise; height is still published with its own (lower) confidence so consumers can
trust period/direction even when the height proxy is shaky.

When a second, well-separated spectral peak is present (e.g. swell under a wind
sea) it is reported under `environment.wave.secondary.*` — the single-component
fallback would otherwise smear the two systems together.

## Path to a measured height (future: heave sensor)

The proxy exists only because nothing on the bus reports vertical motion: the
Raymarine EV-1 puts attitude (PGN 127257) on N2K but **not** heave (PGN 127252),
and there is no MRU aboard. A measured significant height needs a source of
**vertical acceleration in the earth frame** — published as `environment.heave`
(m, an `environment` path, not `navigation.attitude`), or as raw vertical
acceleration the plugin integrates itself.

The practical candidate is a low-cost 9-DoF IMU with on-chip fusion, e.g. a
**BNO085** (the BNO055 is legacy). Notes for an on-board fit:

- **Interface — UART, not I²C.** Both Bosch BNO0xx parts use I²C clock
  stretching, which the Raspberry Pi's hardware I²C cannot handle reliably (you
  get dropped/corrupt reads). Use a Pi UART, or a small ESP32 + SensESP node that
  publishes deltas over Wi-Fi/N2K.
- **Mounting — placement matters, orientation does not.** Mount it near the
  vessel's centre of motion (low, central) so rotation-induced acceleration
  (`r × ω̇` + centripetal) does not leak into heave. Orientation is *free* for
  heave alone: the fusion tracks "down" itself, so the vertical component is
  recovered however the chip is turned — it only has to be rigidly fixed. No
  magnetometer calibration is needed (gravity + gyro suffice; the magnetometer
  only fixes heading), which sidesteps the usual hard/soft-iron grief on board.
- **Plugin side — integrate in the frequency domain.** Rather than time-domain
  double integration (which drifts), take the acceleration PSD and **divide by
  (2πf)⁴** to get the elevation spectrum. The existing band limit
  (`fMin = 1/periodMax`) discards the low-frequency bins where integration would
  otherwise blow up. Height then becomes a true `Hs = 4·√(m0_heave)` — no `k`
  inversion, no narrowband assumption, no λ-vs-hull validity gate.

Until that hardware exists, the slope inversion below is the best available.

## Physics

Deep-water encounter relation, solved for the true angular frequency `ω`:

```
ω_e = ω − (ω²/g)·U·cos(μ)          measured ω_e, speed U, encounter angle μ
a·ω² − ω + ω_e = 0,  a = U·cos(μ)/g
ω = (1 − √(1 − 4·a·ω_e)) / (2a)     physical root; ω → ω_e as a → 0
T = 2π/ω,  λ = g·T²/2π ≈ 1.56·T²,  c = g·T/2π,  c_g = c/2
```

Head seas (`cos μ < 0`) raise the encounter frequency (waves met more often);
following seas lower it and can be multi-valued. As the encounter frequency
approaches its maximum the two roots converge and the solve becomes
ill-conditioned (and beyond the maximum there is no real root at all). In both
cases the corrected period would be systematically wrong rather than merely
uncertain, so the plugin **withholds** the wave parameters for that cycle
(publishing only the heartbeat with `state: lowConfidence`) instead of emitting a
confident-looking but arbitrary value.

The encounter angle magnitude off the bow is estimated from the slope-energy
ratio `tan α = √(m0_roll / m0_pitch)`. Amplitude alone cannot resolve
head-vs-following or port-vs-starboard; wind direction (if available) picks the
head/following sign, otherwise the configured default regime is used.

The **port/starboard side** comes from the **pitch/roll cross-spectrum** at the
dominant bin. For a single wave pitch and roll oscillate co-linearly in time, so
the cross-spectrum is real at the peak and its sign tracks the side the wave
comes from (`Re(S_pr) ∝ sin 2β`). Which sign means starboard depends on the IMU
mounting and the attitude sign convention, so it is calibrated once at sea trial
via the **Flip port/starboard side** setting. `directionRelative` is then a
signed angle in `[−π, π]` (+ve starboard, −ve port), as for other Signal K
relative angles.

## Published paths (`environment.wave.*`, SI units)

| Path | Unit | Meaning |
|---|---|---|
| `period` | s | true wave period (speed-corrected) |
| `encounterPeriod` | s | measured encounter period (uncorrected) |
| `length` | m | wavelength |
| `celerity` | m/s | phase speed |
| `groupSpeed` | m/s | group speed (c/2) |
| `significantHeight` | m | **proxy** Hs from slope inversion |
| `heightConfidence` | ratio | 0–1 confidence in the height proxy (≤ `confidence`) |
| `directionTrue` | rad | direction waves come from (heading + relative) |
| `directionRelative` | rad | signed off-bow angle waves come from (+ve starboard, −ve port) |
| `confidence` | ratio | 0–1 confidence in period/direction |
| `secondary.present` | bool | a distinct second wave system was detected |
| `secondary.period` | s | secondary wave period |
| `secondary.length` | m | secondary wavelength |
| `secondary.directionTrue` | rad | secondary direction (from) |
| `secondary.directionRelative` | rad | secondary signed off-bow angle (from) |
| `secondary.confidence` | ratio | secondary peak prominence vs the primary |

`environment.wave.*` is not part of the formal Signal K spec but is the
conventional namespace (`significantHeight`, `period`, `direction`).

## Configuration

| Setting | Default | Notes |
|---|---|---|
| Analysis window (s) | 90 | FFT buffer length; longer = finer low-frequency resolution |
| Update interval (s) | 5 | how often an estimate is computed |
| Resample rate (Hz) | 4 | attitude is resampled to a uniform grid before the FFT |
| Shortest / longest period (s) | 2 / 6 | analysis band. The 6 s upper bound keeps the peak off broadband low-frequency roll/heel motion (see *Sea trial*); raise it for genuine ocean swell |
| Waterline length (m) | 8.4 | Elan 333 LWL; drives the confidence flag |
| Default sea regime | head | head/following sign when wind is unknown |
| Minimum confidence | 0.1 | suppress estimates below this |
| Minimum motion (° RMS slope) | 0.5 | amplitude gate; suppress when the boat barely moves (dock ≈ 0.04° RMS). 0 disables |
| Flip port/starboard side | off | invert the cross-spectrum side mapping if the reported side is mirrored at sea trial |

## Install / deploy on board

```bash
cd signalk-wave-estimator && npm install && npm link
cd ~/.signalk && npm link signalk-wave-estimator
# then restart signalk-server and enable the plugin
```

## Test

`npm test` runs a synthetic sanity check (no hardware): it builds pitch/roll
from a known wave and verifies the estimator recovers the period, wavelength and
the encounter-speed correction.

## Known limitations / TODO

- **One sea trial so far (2026-06-21); height not yet re-validated.** Direction
  is confirmed; the `periodMax` fix that brings height/wavelength into range is
  validated only on that recording, not yet re-run on the water. The height proxy
  remains uncalibrated against a measured reference.
- **Height is unreliable for short coastal wind sea** (wavelength barely above
  the hull) — the regime of the trial. It is a slope-inversion proxy with no
  heave sensor; see *Path to a measured height* for what a real one would take.
- **Broadband low-frequency roll contamination** (a sailboat's slow rolling/
  heeling under way) is rejected only by the tight `periodMax`, not separated
  signal-wise — it cannot be distinguished from a genuine beam sea in a single
  window. A cross-window persistence tracker for the hull's own motion is the
  next refinement (noted in the research doc).
- Following-seas root ambiguity / ill-conditioning near the encounter-frequency
  maximum is handled by withholding the estimate, not by full resolution.
- `directionRelative` port/starboard side is resolved from the pitch/roll
  cross-spectrum; the trial confirmed the default mapping (`flipSide = false`).
- Narrowband height inversion still biases broadband/confused seas; a distinct
  **second** wave system is now detected and reported under
  `environment.wave.secondary.*`, but three-plus systems are not.
- Optional shallow-water dispersion correction (`environment.depth`) not yet
  implemented; deep-water approximation only.
