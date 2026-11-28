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

## Deviations from the pre-registered protocol

This section records what the completed 750-evaluation run actually did. Where
it conflicts with the pre-registered text above, this section is authoritative.

**(a) One seed per generation, not a two-seed subset.** The evaluator reduced
every seed list to its first element (`seeds[0]`). Generation *g* therefore used
**one** dev seed, rotating `1, 2, 3, 4, 5` by generation (the in-run history
shows rotations such as `5|1`), instead of the planned two-seed subsets
`[(1,2),(2,3),(3,4),(4,5),(5,1)]`. Common random numbers within a generation
still held: every candidate in generation *g* used the same single seed.
Consequently no in-run cost is comparable across generations by itself, which
is why the extension rule used re-scores.

**(b) Winner = in-run incumbent; the top-5 re-score did not happen.** Candidate
vectors beyond the incumbent were not persisted in the checkpoint or the JSON,
so the pre-registered "top-5 full-length dev-seed 1–5 re-score" could not be
performed after the run. The reported winner is the final in-run incumbent (the
best candidate seen in the run), not the winner of a top-5 re-score. Only the
winner vector is stored.

**(c) The extension rule was applied to single-seed values.** The recorded
generation-60 and generation-75 re-scores were computed with the same
`seeds[0]` bug, so they re-evaluated **seed 1**, not dev seeds 1–5. Both
incumbent vectors came from seed-1 generations. The observed values were
`0.117415 -> 0.116469`, an improvement of **0.81 %**, below the 2 % threshold.
The generation-60 incumbent vector was not saved, so a corrected five-seed
re-score of that exact candidate cannot be reconstructed; the decision is
therefore **do not extend**, recorded as made on the single-seed values.

**(d) Corrected five-seed baselines.** Every earlier "dev seeds 1–5" cost in
this document and in commit messages was actually **seed 1 only** for the same
reason. Corrected values on dev seeds 1–5 (full 120 s runs, current cost code):

| variant | earlier value (seed 1) | corrected (seeds 1–5) |
|---|---:|---:|
| M5 default | 1.000000 | 1.000000000 |
| ZN ultimate-gain | 1.453672 | 1.529218277 |
| ZN reaction-curve PID | 1e6 -> 140.147186 | 138.980368633 |
| ZN reaction-curve PI | 1e6 -> 133.732190 | 134.236504063 |
| ZN reaction-curve PI (rate only) | 1.070655 | 1.068490376 |
| autotuned | 0.116469 | 0.122455498 |

The ZN reaction-curve full history is: **1e6** under the old constant penalty
(a startup-saturation artifact, not a real cost) -> **140.147186 / 133.732190**
after the graded penalty was introduced, still on seed 1 -> **138.980369 /
134.236504** on five seeds. Both variants remain hard failures; the numbers are
penalties, not valid tracking costs.

**(e) The result is bound-limited.** Position `kp` is exactly at its upper
bound (**1.0**, 4× the default) and position `ki` is within 2 % of its upper
bound (**0.1982** vs **0.2**). Wider bounds could do better; this run cannot
say how much. A second tuning attempt with wider bounds would have to be
reported as such, and the held-out benchmark would have to be repeated for any
such new candidate.

**`tiltOf` bug (fixed in `7a9b105`).** `cost.ts` decoded the quaternion as
`[x, y, z, w]` while `Quat` is `[w, x, y, z]`, so `RolloutMetrics.maxTilt` was
always `pi`. Max tilt is **not** one of the eleven cost terms (those use
overshoot/settling/ISE, RMS, contact seconds and jitter), so the bug did not
touch any cost, candidate selection, the objective, the gains digest, or any
earlier reported tuning/benchmark number. The `max_tilt_rad` column in
`scripts/benchmark.ts` would have been wrong; the benchmark has **not** been
run. Earlier max-tilt figures in `docs/control.md` (5.53°, 31.4°) come from
`tests/controlCascade.test.ts` via `quatToEulerZYX`, not from `tiltOf`, and are
unaffected by this bug.

## Exploratory robustness (post-hoc, pre-specified rule)

The following is **exploratory, post-hoc, not pre-registered, and not used for
any decision about the candidate**. It does not change the candidate, the
protocol, or the benchmark. It is run on dev seeds 1–5 only; no held-out seed is
consulted.

**Flag rule, fixed before the table was computed.** A condition is flagged if
the tuned controller (i) hard-fails, (ii) diverges, or (iii) its RMS ratio to
the default exceeds **2× its nominal-condition ratio**. The last test asks
whether tuned gains degrade *relative to default* much more than they already
do in the nominal condition, not whether either controller is good in absolute
terms.

**Conditions.** Nominal (reference) plus: measurement/control latency of
**5 ms**, **10 ms**, **50 ms** and **100 ms** (the controller sees the
measurement delayed by that many 1 ms steps; **this is sensor-to-controller
latency only, actuator latency remains zero**); sensor-noise sigmas **×2**;
mean wind from **+y** (90°) and from **45°** instead of +x; and the existing
**kT −10 % / tau +30 %** and **mass +10 %** plant mismatch. For each condition
and controller: light, gusty and strong RMS (mean over seeds), 5 m step
overshoot and settling (mean over seeds), max tilt (max over seeds, fixed
quaternion order), max motor jitter and peak motor rate (maxima over the four
rollout types and seeds), sustained saturation fraction (max over seeds), and
failures.

### Results (dev seeds 1–5, full 120 s)

Run on the committed tuned vector (`data/tunedGains.json`), 600 rollouts, 8
workers. `M3 gate` is `integralDwell = 0` with default gains. Max tilt, jitter
and motor rate are maxima over the step/light/gusty/strong rollouts and seeds
(jitter rad/s, motor rate rad/s²).

| condition | controller | light RMS | gusty RMS | strong RMS | step overshoot | step settling | max tilt | max jitter | max motor rate | sustained sat | failures |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| nominal | M3 gate | 0.694 | 3.579 | 4.747 | 0.042 | 14.00 | 30.11° | 0.052 | 266 | 0.0000 | none |
| nominal | M5 default | 0.665 | 3.579 | 4.747 | 0.042 | 14.00 | 30.11° | 0.052 | 266 | 0.0000 | none |
| nominal | autotuned | 0.112 | 0.921 | 0.958 | 0.064 | 5.56 | 30.01° | 0.174 | 923 | 0.0000 | none |
| latency 5 ms | M3 gate | 0.693 | 3.579 | 4.747 | 0.042 | 13.99 | 30.11° | 0.052 | 266 | 0.0000 | none |
| latency 5 ms | M5 default | 0.666 | 3.579 | 4.747 | 0.042 | 13.99 | 30.11° | 0.052 | 266 | 0.0000 | none |
| latency 5 ms | autotuned | 0.112 | 0.921 | 0.958 | 0.065 | 5.55 | 30.00° | 0.174 | 923 | 0.0000 | none |
| latency 10 ms | M3 gate | 0.694 | 3.579 | 4.748 | 0.042 | 13.98 | 30.10° | 0.052 | 266 | 0.0000 | none |
| latency 10 ms | M5 default | 0.666 | 3.579 | 4.748 | 0.042 | 13.98 | 30.10° | 0.052 | 266 | 0.0000 | none |
| latency 10 ms | autotuned | 0.112 | 0.921 | 0.958 | 0.066 | 5.54 | 30.00° | 0.174 | 922 | 0.0000 | none |
| latency 50 ms | M3 gate | 0.694 | 3.581 | 4.753 | 0.043 | 13.94 | 30.07° | 0.052 | 266 | 0.0000 | none |
| latency 50 ms | M5 default | 0.666 | 3.582 | 4.753 | 0.043 | 13.94 | 30.07° | 0.052 | 266 | 0.0000 | none |
| latency 50 ms | autotuned | 0.113 | 0.921 | 0.958 | 0.071 | 6.47 | 30.01° | 0.174 | 923 | 0.0000 | none |
| latency 100 ms | M3 gate | 0.694 | 3.586 | 4.761 | 0.044 | 13.88 | 30.00° | 0.052 | 265 | 0.0000 | none |
| latency 100 ms | M5 default | 0.640 | 3.586 | 4.761 | 0.044 | 13.88 | 30.00° | 0.052 | 265 | 0.0000 | none |
| latency 100 ms | autotuned | 0.113 | 0.922 | 0.958 | 0.078 | 7.16 | 32.81° | 0.175 | 911 | 0.0000 | none |
| sensor sigma ×2 | M3 gate | 0.684 | 3.578 | 4.743 | 0.058 | 13.68 | 30.10° | 0.104 | 539 | 0.0000 | none |
| sensor sigma ×2 | M5 default | 0.657 | 3.578 | 4.743 | 0.058 | 13.68 | 30.10° | 0.104 | 539 | 0.0000 | none |
| sensor sigma ×2 | autotuned | 0.113 | 0.923 | 0.959 | 0.079 | 5.44 | 30.00° | 0.347 | 1951 | 0.0000 | none |
| wind from +y (90°) | M3 gate | 0.722 | 3.743 | 4.921 | 0.042 | 14.00 | 30.14° | 0.052 | 268 | 0.0000 | none |
| wind from +y (90°) | M5 default | 0.693 | 3.743 | 4.921 | 0.042 | 14.00 | 30.14° | 0.052 | 268 | 0.0000 | none |
| wind from +y (90°) | autotuned | 0.112 | 0.935 | 0.945 | 0.064 | 5.56 | 30.03° | 0.172 | 923 | 0.0000 | none |
| wind from 45° | M3 gate | 0.707 | 3.748 | 4.954 | 0.042 | 14.00 | 30.18° | 0.052 | 266 | 0.0000 | none |
| wind from 45° | M5 default | 0.668 | 3.748 | 4.954 | 0.042 | 14.00 | 30.18° | 0.052 | 266 | 0.0000 | none |
| wind from 45° | autotuned | 0.113 | 0.962 | 1.012 | 0.064 | 5.56 | 30.02° | 0.172 | 923 | 0.0000 | none |
| kT −10 %, tau +30 % | M3 gate | 2.130 | 3.520 | 4.581 | 0.000 | 120.00 | 26.24° | 0.038 | 195 | 0.0000 | 1: ground contact in flight |
| kT −10 %, tau +30 % | M5 default | 1.596 | 3.520 | 4.581 | 0.008 | 81.28 | 26.24° | 0.038 | 195 | 0.0000 | 1: ground contact in flight |
| kT −10 %, tau +30 % | autotuned | 0.128 | 1.029 | 1.088 | 0.006 | 19.78 | 30.05° | 0.126 | 666 | 0.0000 | none |
| mass +10 % | M3 gate | 1.893 | 3.962 | 4.294 | 0.000 | 120.00 | 26.93° | 0.050 | 253 | 0.0000 | 1: ground contact in flight |
| mass +10 % | M5 default | 1.358 | 3.962 | 4.294 | 0.010 | 79.06 | 26.93° | 0.050 | 253 | 0.0000 | 1: ground contact in flight |
| mass +10 % | autotuned | 0.115 | 0.919 | 0.945 | 0.006 | 19.35 | 30.00° | 0.164 | 871 | 0.0000 | none |

Flag-rule check (tuned/default RMS ratio versus its nominal value): the largest
**ratio-of-ratios** change is **1.18× nominal** (kT/tau, strong). This is not the
absolute tuned RMS change: that row is
`(1.088 / 4.581) / (0.958 / 4.747) = 1.176`, because the default strong RMS also
moves from 4.747 to 4.581. In absolute tuned terms the same row is
`1.088 / 0.958 = 1.136×`, and the largest absolute tuned change is light under
kT/tau, `0.128 / 0.112 = 1.143×`. No condition is flagged: the tuned controller
does not hard-fail or diverge anywhere, and no ratio-of-ratios exceeds 2×.

**Findings.** The tuned gains do **not** degrade more than the default in any
tested condition and never go unstable or hard-fail. Latency 5/10 ms and ×2
sensor noise leave tuned RMS essentially unchanged (≤ 2 %). The 50/100 ms
positive control is still stable: at 100 ms the tuned step settles in 7.16 s
versus 5.56 s nominal, overshoot rises from 0.064 m to 0.078 m, and max tilt
reaches 32.81° (the command clamp is 30°, so the lag overshoots it) with no
failure. Sustained saturation stays exactly 0 everywhere, so the high-gain loop
does not exhibit chatter that the noise-free parts of the model merely tolerate;
motor activity is nevertheless high: tuned max jitter is 0.174 vs 0.052 rad/s
for default and max motor rate 923 vs 266 rad/s², roughly 3.3–3.5×, doubling
again under ×2 sensor noise. +y and 45° mean wind change tuned RMS by at most
~5.6 % (strong, 45°), the same order as the default.

Under kT −10 %/tau +30 % and mass +10 %, the **default hard-fails once**
(ground contact in flight in light wind) while the tuned controller does not;
its ratio to default improves. The mismatch rows are still a real absolute
limitation for the tuned controller: step settling rises from 5.56 s nominal to
19.35–19.78 s (**~3.5× worse**), although that is still ~4× faster than the
default's 79–81 s in the same runs. Max tilt reaches the ~30° clamp in wind for
both controllers; it is a command limit, not a simulator artifact.

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

> **Actual run (see "Deviations from the pre-registered protocol"):** the
> evaluator used one seed per generation, the top-5 re-score was not performed,
> and the winner is the in-run incumbent.

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
npm run tune:robustness    # exploratory latency/noise/wind/mismatch grid (dev seeds only)
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
