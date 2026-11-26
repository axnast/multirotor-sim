# Milestone 6: autotuner and benchmark (pre-registered protocol)

This document is the contract for M6. It was committed **before any tuning run**.
A fresh session can continue from the repository alone. Nothing here may be
changed after seeing held-out results.

## Pre-run amendments (applied before any tuning)

1. The tuning objective is now a weighted **geometric** mean
   `exp(Σ w_i ln(term_i/default_i))` with floors applied before the log, so the
   default is still exactly 1.000 and the objective matches the held-out
   headline. Weights (sum 1.00): light .08, moderate .10, gusty .12, strong .10,
   constant 5 m/s .08, mass20 .14, step .12, square .08, hoverJitter .06,
   mismatchKTau .06, mismatchMass .06. Hard failures still return finite 1e6.
2. The ZN position loop is **not applicable**: a P-only loop on a double
   integrator has no finite ultimate gain (the plant contributes −180° only at
   zero frequency), so no Ku/Tu exists and the default position gains are kept.
   This is a property of the plant, not a hand-tuned choice.
3. The ZN attitude loop is **P-only** (tilt/yaw gains are proportional attitude
   errors; derivative action lives in the fixed rate loop). Therefore the PI
   rule `Kp = 0.45 Ku, Ti = 0.83 Tu` is used, not the PID rule
   `Kp = 0.6 Ku, Ti = 0.5 Tu, Td = 0.125 Tu`: there is no attitude derivative
   term to set, and adding one would duplicate the rate loop's fixed D action.
4. A `--max-evals N` smoke mode was added to the tune script (and a `--steps N`
   smoke-only horizon override). It is used solely to verify CLI/history/
   checkpoint/resume mechanics; it is not a tuning run and is never used for
   reported costs.
5. A third ZN baseline, **ZN reaction-curve PI**, uses the classic PI rule for
   the rate loop `Kp = 0.9/(R L), Ti = 3.33 L, Kd = 0`; attitude and position
   are unchanged from the PID variant. Benchmark baselines are now: M3 gate,
   M5 default, gate off, ZN ultimate-gain, ZN reaction-curve PID,
   ZN reaction-curve PI, autotuned. A bug in the running saturation-fraction
   check (a single saturated step at step 0 gives 1/1 > 20%) was fixed with a
   1000-step warm-up; the whole-run check still applies. This is a root-cause
   bug fix, not a tolerance change.
6. The extension rule now uses **fixed dev-seed 1–5 re-scores of the incumbent
   best candidate at generations 60 and 75**, because rotating-subset in-run
   costs are not comparable across generations. Extend to 1000 only if
   `(cost60 - cost75)/cost60 > 0.02`; held-out seeds are never consulted. The
   history CSV logs both re-scores.

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
(5) ZN reaction-curve PID, (6) ZN reaction-curve PI, (7) autotuned. Every
baseline number that differs from the README noise-off table is reported; that
section is updated later.

## Cost function (`src/core/tuning/cost.ts`)

Eleven full-length runs per evaluation; constant 5 m/s wind is **explicitly
included**:

| term | weight |
|---|---:|
| light / moderate / gusty / strong wind | 0.08 / 0.10 / 0.12 / 0.10 |
| constant 5 m/s wind | 0.08 |
| +20% mass | 0.14 |
| 5 m step | 0.12 |
| square tracking | 0.08 |
| noise-on hover jitter | 0.06 |
| 5 m step, kT −10% + tau +30% | 0.06 |
| 5 m step, mass +10% | 0.06 |

These are the **pre-run amendment** weights (sum 1.00), re-weighted toward the
headline. The step term is
`0.5·overshoot + 0.3·settling + 0.2·ISE`; the mass term is
`0.75·RMS + 0.25·contact_seconds`. Other terms use mean true-position RMS over
[20,120] s; jitter is the standard deviation of the per-step motor-speed
derivative.

Normalization: each subterm is `1 + (value - default_value) / max(default_value,
floor)`, so a floor-dominated default still scores exactly **1.000**. Floors:
overshoot 2%, settling 0.5 s, ISE 1e-6, RMS 1e-4, contact 1 s, jitter 1e-6,
applied **before** the log. The objective is the weighted **geometric** mean
`exp(Σ w_i ln(term_i / default_i))`, matching the pre-registered held-out
headline; any hard failure still makes the total the finite **1e6**. The
measured default cost is **1.000000000000** on dev seeds 1–5.

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
Pre-registered extension rule (pre-run amendment): re-score the incumbent best
candidate on the **fixed full dev seeds 1–5** at generations **60 and 75**;
extend to **1000** only if the 75-vs-60 cost improved by **more than 2%**.
Rotating-subset in-run costs are not comparable across generations; held-out
seeds are never consulted. Single-rollout early stop on hard failure is allowed.
The history CSV logs `rescore_generation,rescore_dev_cost` at those two points.

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
- **ZN reaction-curve PID** for the rate loop: the continuous-time rate plant
  `τ s² + s + kp` is stable for all kp > 0 (integrator plus motor lag), so an
  ultimate-gain sweep can only find discrete-delay/saturation artifacts. Measure
  the open-loop step response slope `R` and apparent dead time `L` (tangent at
  steepest slope), then `Kp=1.2/(R L), Ti=2L, Td=0.5L`. Attitude/position use
  ultimate gain where finite, reaction curve otherwise; the choice is logged.
- **ZN reaction-curve PI**: same R/L and attitude/position, rate loop
  `Kp=0.9/(R L), Ti=3.33 L, Kd=0` (classic PI rule, no derivative). This is the
  third labelled baseline; neither variant is hand-tuned.

Diagnosis of the reaction-curve PID failure: the saturation comes from the
**pitch and yaw axes** — the ZN attitude P gain (tilt=yaw ≈ 44.95) demands rate
commands far above the fixed yaw-rate limit (1.2 rad/s) and yaw acceleration
limit (1.88 rad/s²). With the default attitude gains the rate loop does not
saturate. It is **not derivative-noise amplification**: removing kd (PI variant)
does not help, and the PID variant still saturates with sensor noise off.

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
npm run tune:baselines     # score default + both ZN variants on dev seeds 1-5
npm run tune -- --max-evals 20 --steps 2000   # smoke test only
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
