# signalk-wave-estimator

Derive sea state from boat motion. The plugin watches `navigation.attitude`
(pitch/roll) plus `navigation.speedThroughWater`, estimates the dominant wave
**encounter period** by FFT of the pitch oscillation, corrects it for boat speed
using the deep-water encounter relation, and publishes true wave period,
wavelength, celerity and a **height proxy** under `environment.wave.*`.

> **Alpha.** Period / wavelength / celerity are well-posed. Wave **height is a
> proxy**, not a measurement — there is no heave/vertical-acceleration sensor on
> the bus, so height is inferred from wave *slope* and carries a confidence
> value. Trust it only when confidence is high. See *Height* below.

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
| Wavelength ≈ hull, or confused/short chop | unreliable — confidence drops |
| Beam seas (roll-dominated) | poor — roll resonance inflates the slope; confidence drops |

The published `environment.wave.confidence` (0–1) folds in wavelength-vs-hull,
spectral sharpness, encounter-solve validity and roll dominance. Estimates below
**Minimum confidence** are suppressed rather than published as noise.

## Physics

Deep-water encounter relation, solved for the true angular frequency `ω`:

```
ω_e = ω − (ω²/g)·U·cos(μ)          measured ω_e, speed U, encounter angle μ
a·ω² − ω + ω_e = 0,  a = U·cos(μ)/g
ω = (1 − √(1 − 4·a·ω_e)) / (2a)     physical root; ω → ω_e as a → 0
T = 2π/ω,  λ = g·T²/2π ≈ 1.56·T²,  c = g·T/2π,  c_g = c/2
```

Head seas (`cos μ < 0`) raise the encounter frequency (waves met more often);
following seas lower it and can be multi-valued — when the boat outruns the wave
energy there is no real root and the plugin falls back to the uncorrected period
with reduced confidence.

The encounter angle magnitude off the bow is estimated from the slope-energy
ratio `tan α = √(m0_roll / m0_pitch)`. Amplitude alone cannot resolve
head-vs-following or port-vs-starboard; wind direction (if available) picks the
head/following sign, otherwise the configured default regime is used. The
port/starboard side of `directionRelative` is left unresolved (would need the
roll/pitch phase relationship).

## Published paths (`environment.wave.*`, SI units)

| Path | Unit | Meaning |
|---|---|---|
| `period` | s | true wave period (speed-corrected) |
| `encounterPeriod` | s | measured encounter period (uncorrected) |
| `length` | m | wavelength |
| `celerity` | m/s | phase speed |
| `groupSpeed` | m/s | group speed (c/2) |
| `significantHeight` | m | **proxy** Hs from slope inversion |
| `directionTrue` | rad | direction waves come from (heading + relative) |
| `directionRelative` | rad | off-bow magnitude waves come from (side unresolved) |
| `confidence` | ratio | 0–1 estimate confidence |

`environment.wave.*` is not part of the formal Signal K spec but is the
conventional namespace (`significantHeight`, `period`, `direction`).

## Configuration

| Setting | Default | Notes |
|---|---|---|
| Analysis window (s) | 90 | FFT buffer length; longer = finer low-frequency resolution |
| Update interval (s) | 5 | how often an estimate is computed |
| Resample rate (Hz) | 4 | attitude is resampled to a uniform grid before the FFT |
| Shortest / longest period (s) | 2 / 20 | analysis band |
| Waterline length (m) | 8.4 | Elan 333 LWL; drives the confidence flag |
| Default sea regime | head | head/following sign when wind is unknown |
| Minimum confidence | 0.1 | suppress estimates below this |

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

- Height is a slope-inversion proxy (no heave sensor); calibrate against a
  sea-trial visual reference.
- Following-seas root ambiguity handled only by fallback, not full resolution.
- `directionRelative` port/starboard side unresolved (needs roll/pitch phase).
- Narrowband height inversion biases broadband/confused seas.
- Optional shallow-water dispersion correction (`environment.depth`) not yet
  implemented; deep-water approximation only.
