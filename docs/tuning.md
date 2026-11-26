# Milestone 6: autotuner and benchmark (pre-registered protocol)

This document is the contract for M6. It was committed **before any tuning run**.
A fresh session can continue from the repository alone. Nothing here may be
changed after seeing held-out results.

## Scope and fixed controller structure

Not in scope: new physics, new disturbance models, controller structure changes.
The following are verified against `src/core/control/controller.ts`
(`defaultControllerParams`) and are **fixed, not tuned**:

| Field | Value |
|---|---|
| `position.integralDwell` | 5 s |
| `position.integralTrimGain` | 3 |
| `position.integralReleaseGuard` | 2 s |
| `rate.derivativeCutoff` | 50 rad/s |
| `position.maxAccelXY / maxAccelZ / maxTilt / minThrust / maxThrust` | defaults |
| `attitude.maxTiltRate / maxYawRate / maxBodyRate` | defaults |
| `rate.maxAngularAcceleration` | defaults |

`position.integralSeparation` is the only gate constant in the search vector.
`assertFixedDefaults` enforces the four scalar defaults in code and tests.

## Parameterization (9 dimensions)

Order and log-space bounds (`src/core/tuning/params.ts`):

| # | gain | default | bounds |
|---:|---|---:|---|
| 1 | position kp | 0.25 | [0.05, 1] |
| 2 | position ki | 0.01 | [0.001, 0.2] |
| 3 | position kd | 1.0 | [0.1, 4] |
| 4 | attitude tilt gain | 1.2 | [0.2, 3] |
| 5 | attitude yaw gain | 0.8 | [0.1, 3] |
| 6 | rate kp | 3.0 | [0.5, 10] |
| 7 | rate ki | 0.5 | [0.01, 3] |
| 8 | rate kd | 0.02 | [0.002, 0.3] |
| 9 | integralSeparation | 0.1 | [0.02, 1] |

All coordinates are `ln(gain)`, clipped to `[ln(lower), ln(upper)]` on decode;
`encode`/`decode` are pure with a round-trip test. `withGains` copies only these
nine fields, so no fixed field can drift.

## Evaluation conditions (no shortcuts)

Every cost and benchmark evaluation uses **full 120 s** runs (120,000 steps at
dt = 1 ms). There is no proxy horizon and no proxy-validity study. Sensor noise
is **on** at `DEFAULT_SENSOR_PARAMS` sigmas and the realistic 6S pack sag is
**on** (`REALISTIC_PACK`, SoC 1, stress pack never used). All six baselines are
scored under these identical conditions:
(1) M3 gate (`integralDwell = 0`, default gains), (2) M5 default (release on),
(3) gate off (`integralSeparation = 1e9`, reference only), (4) ZN ultimate-gain,
(5) ZN reaction-curve, (6) autotuned. Every baseline number that differs from
the README noise-off table is reported; that section is updated later.

## Cost function (`src/core/tuning/cost.ts`)

Eleven full-length runs per evaluation; constant 5 m/s wind is **explicitly
included**:

| term | weight |
|---|---:|
| 5 m step | 3/16 |
| square tracking | 3/16 |
| light, moderate, gusty, strong wind | 1/16 each |
| constant 5 m/s wind | 1/16 |
| +20% mass | 2/16 |
| noise-on hover jitter | 1/16 |
| 5 m step, kT −10% + tau +30% | 1/16 |
| 5 m step, mass +10% | 1/16 |

Weights are binary fractions summing to exactly 1. The step term is
`0.5·overshoot + 0.3·settling + 0.2·ISE`; the mass term is
`0.75·RMS + 0.25·contact_seconds`. Other terms use mean true-position RMS over
[20,120] s; jitter is the standard deviation of the per-step motor-speed
derivative.

Normalization: each subterm is `1 + (value - default_value) / max(default_value,
floor)`, so a floor-dominated default still scores exactly **1.000**. Floors:
overshoot 2%, settling 0.5 s, ISE 1e-6, RMS 1e-4, contact 1 s, jitter 1e-6.
Floor-limited deviations are scaled by the floor; this is documented, not a
tolerance change. The measured default cost is **1.000000000000** on dev seeds
1–5, and the dominant weighted contribution is the step term.

Hard failures return the finite penalty **1e6** with a logged reason: non-finite
or divergent state (|x| or |v| > 100), ground contact in flight, mixer
saturation above 20% of steps. A single rollout may stop early on failure.
**Exception:** ground contact in the +20% mass scenario is a continuous cost
term (the default touches down around 4 s), never a failure. The default gains
pass every scenario.

Model mismatch is applied to the plant config only (`PlantOverride` in the
shared loop); the controller and mixer keep nominal constants.

## Seeds and selection

Tuning/selection uses dev seeds **1–5 only**. Within a generation every
candidate uses the **same** seed subset (common random numbers); the subset
rotates `[(1,2),(2,3),(3,4),(4,5),(5,1)]` by generation so the tuner cannot
memorize one wind realization. The final **top-5 re-score** uses all dev seeds
1–5 and is the only basis for winner selection — never the in-run rotating-seed
cost. Held-out seeds **1001–1020** are used only by `scripts/benchmark.ts`, once
per final candidate; no held-out seed appears in any test or tuning code path.
No re-tuning after held-out results; a second tuning attempt must be reported.

## CMA-ES and budget

Hand-written CMA-ES (`src/core/tuning/cmaes.ts`): λ = 4 + ⌊3 ln 9⌋ = **10**,
μ = 5, σ₀ = 0.25, own xoshiro128** stream, documented box clipping, Jacobi
eigen-decomposition, `ask`/`tell`, full JSON checkpoint after every generation.
Seed 20261126. The full state (including RNG) is serialized; resume is
bit-identical (tested).

Budget: **750 evaluations** (~75 generations), **8 workers** (measured best on
this 6-core/12-thread laptop: 3.66× speedup; 12 workers degrade to 3.41×).
Pre-registered extension rule (dev-seed cost only, held-out never consulted):
extend to **1000** evaluations only if the best dev-seed cost improved by more
than **2% over the last 15 generations**; otherwise stop at 750. Single-rollout
early stop on hard failure is allowed. Final evaluation count and learning curve
are in `out/autotune_history.csv`.

Measured full-length noisy cost: **45.37 s/evaluation** single-threaded after
baseline caching (93.59 s including baseline computation). At 8 workers:
≈ **12.4 s/evaluation**, so 750 evals ≈ **2 h 35 m** plus ≈ 5–6 min baseline
precompute; the 1000-eval extension ≈ **3 h 27 m**. Session plan: **three
sessions of about one hour**, resuming from the checkpoint between them.

## Ziegler-Nichols baselines (`scripts/zn.ts`)

dt = 1 ms, identification under no disturbances, scoring under the noisy
protocol. Oscillation rule: at least 3 complete cycles, half-period drift < 5%,
amplitude > 1% of the step; ultimate gain is where the half-cycle peak trend
crosses 1 (decaying below, growing above). Safety caps on every sweep.

- **ZN ultimate-gain**, loop by loop: rate loop P-only (others default), attitude
  P-only with the ZN rate loop closed, position P-only with both inner loops
  closed. Classic rules: PID `Kp=0.6 Ku, Ti=0.5 Tu, Td=0.125 Tu` for rate; PI
  `Kp=0.45 Ku, Ti=0.83 Tu` for attitude and position (position kd unchanged).
- **ZN reaction-curve** for the rate loop: the continuous-time rate plant
  `τ s² + s + kp` is stable for all kp > 0 (integrator plus motor lag), so an
  ultimate-gain sweep can only find discrete-delay/saturation artifacts. Measure
  the open-loop step response slope `R` and apparent dead time `L` (tangent at
  steepest slope), then `Kp=1.2/(R L), Ti=2L, Td=0.5L`. Attitude/position use
  ultimate gain where finite, reaction curve otherwise; the choice is logged.

Neither variant is hand-tuned; a poor or unstable result is reported as the
baseline. Full Ku/Tu/R/L and gains: `out/zn_log.md`, `data/znGains.json`.

## Reporting protocol

Primary metric: per scenario (light, moderate, gusty, strong, constant 5 m/s,
+20% mass), mean true-position RMS over [20,120] s on held-out seeds. Headline:
geometric mean over the **five stochastic scenarios** of tuned/baseline RMS,
with a bootstrap 95% CI (**10,000 resamples**, seed indices resampled with
replacement, paired by seed across variants). A per-scenario win is claimed only
if its paired CI excludes zero. Paired per-seed differences against each
baseline: mean, SE, bootstrap CI and fraction of seeds improved.

Secondary: windows [20,60] and [90,120]; 5 m step overshoot/settling/ISE; max
tilt; saturation steps; motor-command jitter with noise on; ground-contact
count. Robustness (reported, not tuned on): mass ±10%, kT ±10%, tau ±30% with
the controller keeping nominal values; report degradation, not only wins.

## Limitations (state these with every claim)

- The model constants are **placeholders, not measured hardware data**. Claims
  apply only "on this simulated hexacopter under these disturbances".
- 750 evaluations is **partial convergence** on a noisy problem. Calibration at
  λ = 10 (seed 1): 10-D sphere reached f < 1e-10 in 1,730 evals; κ = 1e3
  quadratic improved 100× in 400 evals. Expect a local improvement, not an
  optimum.
- Held-out seeds use the **same wind model** as dev seeds; the benchmark tests
  seed variation, not model-class generalization.

## Commands (run these yourself in PowerShell, D:\oc2)

```
npm run zn                 # regenerate ZN baselines and log
npm run tune:probe         # one full default evaluation, breakdown + timing
npm run tune               # start: 750 evaluations
npm run tune:resume        # continue from out/autotune_checkpoint.json
npm run tune:extend        # resume and apply the 2%/15-gen extension rule
Get-Content out/autotune_history.csv -Tail 5    # check progress
npm run benchmark          # only after data/tunedGains.json exists
```

Outputs: `out/autotune_history.csv` (learning curve), `out/autotune_checkpoint.json`
(resumable full state), `data/tunedGains.json` (final gains, seed, version,
weights, evaluation count), `out/benchmark.csv`, printed markdown tables.

To stop safely press **Ctrl+C**: the checkpoint written after the last completed
generation is valid; a partially completed generation is simply replayed on
resume. Laptop advice: plug in power, set **High performance**, disable sleep,
close other heavy apps. The tuner takes ~2.5 h across three ~1 h sessions; the
benchmark takes ~20 min.
