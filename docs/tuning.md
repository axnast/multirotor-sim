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
   mismatchKTau .06, mismatchMass .06. Failures use the graded finite penalty
   (amendment 8), never 1e6.
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
7. A fourth ZN baseline, **ZN reaction-curve PI, rate loop only** (rate PI with
   attitude and position at M5 defaults — ZN applied only where it is well
   defined), was added. Benchmark baselines are now: M3 gate, M5 default, gate
   off, ZN ultimate-gain, ZN reaction-curve PID, ZN reaction-curve PI,
   ZN reaction-curve PI (rate only), autotuned. A baseline that hard-fails in
   any scenario is printed as **"fails (no valid RMS)"** and excluded from all
   ratios and the geometric-mean headline.
8. The hard-failure rule now measures **sustained** mixer saturation: the
   fraction of steps saturated over `t >= 5 s`, with the startup window
   excluded from both numerator and denominator, failing above 20%. Ground
   contact (except +20% mass), NaN/Inf and divergence remain failures. The
   constant 1e6 term penalty is replaced by a graded finite
   `100 * (1 + excess)`, where `excess` is the fraction by which sustained
   saturation exceeds 20% or the fraction of the run completed before
   contact/divergence; non-finite states use a fixed finite 1000. This gives
   CMA-ES a monotone slope out of failure regions. Sustained saturation is
   measured over the whole `t >= 5 s` window (no early stop on that check:
   stopping at the first crossing would report ~1.0 from a handful of steps and
   destroy the gradient); NaN/Inf/divergence/contact still stop early.

**Startup-burst diagnosis (PI variant, dev seed 1):** the ZN attitude P gain
(44.95) is closed through the aggressive ZN rate loop, and the desired rate
grows to ~2 rad/s over the first second, after which the rate PID demands
~0.94 N·m and one motor group is driven to 0 rad/s while the other reaches
780–844 rad/s. The rate integrators stay near 0.001–0.02, so the large rate
integral gain (Ti = 0.163 s) is not the driver; sensor noise alone
(0.2° × 45 ≈ 0.16 rad/s) is also far too small. The single root cause was
**not isolated**: the growth is consistent with an unstable attitude-loop
oscillation from the very high ZN P gain, but we do not claim to have proven
that mechanism. After the first second the mixer stops saturating (whole-run
fraction only ~0.005), which is why the sustained rule, not the old
cumulative-from-t=0 rule, is the right criterion.

**Yaw-authority reconciliation:** the earlier 0.90 N·m figure summed
`kQ·ω_max²` over all six motors without the thrust constraint and is wrong.
Computed from `mixDetailed`: at hover thrust 14.715 N the maximum feasible
`τ_z` is **0.2207 N·m** (2.354 rad/s² on Izz = 0.09375); with thrust free the
maximum is **0.4500 N·m at 30 N** (4.800 rad/s²). The hand calculation
`0.015 m × 6 × 2.453 N ≈ 0.221 N·m` matches. The controller's own yaw
acceleration limit is 1.8835 rad/s² ≈ 0.1766 N·m, so the ZN torque demand is
far beyond the available yaw authority.

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
(5) ZN reaction-curve PID, (6) ZN reaction-curve PI, (7) ZN reaction-curve PI
(rate only), (8) autotuned. Every baseline number that differs from the README
noise-off table is reported; that section is updated later.

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
headline; any hard failure uses the graded finite penalty of amendment 8. The
measured default cost is **1.000000000000** on dev seeds 1–5.

Hard failures use the graded finite penalty `100*(1+excess)` (non-finite states
1000) with a logged reason: non-finite or divergent state, ground contact in
flight, or **sustained** mixer saturation above 20% of steps over `t >= 5 s`.

Divergence criterion: at every core step the **true** position and velocity are
checked; divergence is `|position| > 100 m` or `|velocity| > 100 m/s` (or a
non-finite value), with `excess` equal to the fraction of the run completed when
it occurs. Over dev seeds 1–5 and all 11 scenarios the maxima are: M5 default
`max |position| = 14.637 m` (strong/seed 5) and `max |velocity| = 2.710 m/s`;
ZN ultimate-gain `13.460 m` and `2.601 m/s`. The margin to the 100 threshold is
therefore at least **6.8×**, so viable candidates are not mislabeled divergent.
A single rollout may stop early on NaN/Inf, divergence or ground contact, but
not on the sustained-saturation check (see amendment 8).
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
The history CSV logs `rescore_generation,rescore_dev_cost` at those two points,
plus `failed_candidates` (count of hard-failed candidates in the generation),
`median_cost` and the generation-0 default reference `default_reference = 1.000`
on every row. Older rows without these columns are accepted on resume with
neutral defaults.

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

Diagnosis of the reaction-curve PID/PI failures: the quantity that saturates is
**torque demand against yaw authority**, not a rate command "beyond the limits".
`attitudeRateCommand` clamps its own output to the fixed tilt/yaw rate limits;
the ZN attitude P gain (tilt=yaw ≈ 44.95, from Ku ≈ 99.89) then makes the rate
PID demand ≈0.94 N·m of torque while the whole hexacopter's yaw authority is
≈0.90 N·m (yaw acceleration limit 1.884 rad/s², Izz ≈ 0.0938 kg·m²). Pitch and
yaw rate axes saturate repeatedly; with default attitude gains the mixer never
saturates. It is **not derivative-noise amplification**: removing kd (PI) does
not help, and the PID variant still saturates with sensor noise off.

Reconciliation of the 20% hard-failure criterion: the failing variants saturate
in a startup burst — at the first checked step (index 999) the **running**
fraction is 0.39–0.65, which fires the check; their whole-run fraction is only
0.003–0.005 because saturation stops after the burst. The check is therefore
not misfiring: the 1000-step warm-up removes the earlier step-0 artifact, and
the M5 default and ZN ultimate-gain variants have a maximum running saturation
fraction of **0.0000** in all 11 scenarios after the warm-up.

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
