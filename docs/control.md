# Controller (Milestone 3)

The controller is a pure cascade: **position → attitude → rate → wrench**. Its
output `[T, τx, τy, τz]` is exactly the `Wrench` consumed by `mix()`. All memory
(integrators, derivative filters) lives in the explicitly returned
`ControllerState`; there are no hidden fields and no DOM/Node imports in
`src/core/control`.

Source of truth:

| Topic | Module |
|---|---|
| Shared types | `src/core/control/types.ts` |
| PID with filtered derivative and anti-windup | `src/core/control/pid.ts` |
| Quaternion error and desired attitude | `src/core/control/attitude.ts` |
| Outer position loop | `src/core/control/position.ts` |
| Defaults, cascade, inner rate loop | `src/core/control/controller.ts` |
| Flight trace | `scripts/flightDemo.ts` |
| Viewer runner and diagnostics | `src/viewer/sim/`, `src/viewer/coords.ts` |

`controllerStepDetailed` is an additive diagnostics wrapper around the same
cascade: it returns the identical state and wrench as `controllerStep` plus the
desired orientation, body rate, acceleration and thrust for viewer telemetry.
`controllerStep` delegates to it and its output is unchanged.

See [multirotor.md](multirotor.md) for the plant and [conventions.md](conventions.md)
for frames and signs.

## Loop structure

```
position error (world)  --PI-D-->  a_des (world)  --+ gravity--> f_des = m(a_des - g)
        |                                                       |
        |                                     T = |f_des|, body +z along f_des
        |                                     q_des from thrust direction + yaw setpoint
        v                                                       v
attitude error q* ⊗ q_des --P--> ω_des (body) --PID--> α_des (body)
                                                       |
                                      τ = I·α_des + ω × (Iω)
```

Because the rigid-body equation is `v_dot = F/m + g` with `g = [0, 0, -9.81]`,
the thrust needed for `a_des` is **`m(a_des - g)`**, not `m(a_des + g)`. A level
hover needs `F = +mg` along world z. This sign is easy to get wrong and was
caught by the hover-hold test.

The controller is called every simulator step (1 kHz) with a zero-order hold.
`controllerStep` is rate-agnostic — it is pure and takes `dt` — so rate-decimated
cascades would need an external scheduler and are out of scope for M3.

## Rate loop

For each body axis, the PID output is a desired angular **acceleration**:

```
α_des = kp·(ω_sp - ω) + ki·∫(ω_sp - ω) + kd·dω_filtered/dt
τ     = I_diag·α_des + ω × (I_diag·ω)
```

The derivative is on the **measurement** (angular rate), filtered with
`α_f = exp(-cutoff·dt)`. Taking the derivative on measurement avoids a setpoint
kick. Expressing the output in acceleration units makes gains independent of
airframe size; the inertia and gyroscopic feedforward are applied afterwards.
The presets have diagonal inertia, so `I_diag` is exact for them.

Per-axis acceleration limits are `[20, 20, 1.8835] rad/s²` for the default hex.
The yaw limit is **computed**, not fixed: at hover, thrust is greedily allocated
to the motors with the largest `-spin·kQ/kT`, giving `max τz = 0.015 × 14.715 =
0.220725 N·m`; with `Izz = 0.09375`, 80 % of that is `1.8835 rad/s²`. Keeping
yaw demand below the real authority prevents the mixer from shedding yaw and
winding up the yaw integrator.

## Attitude loop

```
q_err = q* ⊗ q_des                 (body-frame error)
if q_err.w < 0: q_err = -q_err     (shortest path)
ω_des.x/y = 2·tiltGain·q_err.x/y   (clamped to tiltRate)
ω_des.z   = 2·yawGain·q_err.z      (clamped to yawRate)
```

Tilt has priority: if the combined body-rate magnitude exceeds `maxBodyRate`,
yaw is reduced first and only then tilt. No Euler angles enter the control
path; they are logged for display only.

## Outer loop and anti-windup

The position loop uses `kp` and `ki` on position error and `kd` on the **true**
velocity error, `kd·(v_sp - v)`. No derivative filter is used here: velocity is
already measured, so filtering a differentiated position would only add phase
lag. Optional `v_sp` is a feedforward; the integral is separated:

```
integrate only if |error| < integralSeparation OR error·v <= 0
```

`integralSeparation` is a **tunable parameter** of `PositionParams` (default
`0.1 m`, validated as positive); M6 can tune it alongside the gains. This is an
extension of conditional integration. Without it, the integral winds up during
the slow approach (the attitude loop adds about 0.8 s of lag), giving 12.7 %
overshoot and more than 40 s settling even with `ki = 0.01`. With it, the same
gains give 0.06 % overshoot and 15.2 s settling while retaining zero steady-state
error. Output saturation uses the usual rule: do not integrate further in the
direction that increases saturation.

### Persistent-error release

Measured in M5: the separation gate has an absorbing equilibrium. With a
constant wind or a mass deficit, the residual velocity is small but points
toward the setpoint, so `error·v <= 0` stays false and the integral is held
forever (3.67 m wind offset; 4.99 m residual for +20 % mass). Widening
`integralSeparation` to 1e9 fixes that but reintroduces the step overshoot.

The gate is therefore augmented with a **release** for persistent errors:

```
vf        = vf + dt/(τ+dt) · (v - vf)      τ = 0.3 s, signed (noise-immune)
stallTime = (|vf| < 0.06 m/s && |e| >= sep) ? stallTime + dt : 0
released  = stallTime >= integralDwell, latched until |e| < sep or saturation
integrate iff !pushFurther && (|e| < sep || e·v <= 0 || released)
integral contribution is multiplied by integralTrimGain while released
```

A release may only arm after the setpoint has been unchanged for
`integralReleaseGuard` seconds; a setpoint change clears the latch. Defaults:
`integralDwell = 5 s`, `integralTrimGain = 3`, `integralReleaseGuard = 2 s`.
`integralDwell = 0` disables the release entirely and is **bit-identical to the
M3 gate** — the frozen flight digest `8ed20e2e…` is unchanged. The three
settings are plain data and have viewer sliders; the 0.06 m/s rest threshold and
0.3 s filter are documented constants.

Measured effect: the 5 m step, square, takeoff, 500 m recovery and 45 s flight
digest are unchanged; a constant 5 m/s wind trims from 3.67 m to <0.05 m by
~90 s (with sensor noise); centred +20 % mass recovers from 4.99 m to <0.1 m;
the centred 10 g payload recovers from 0.202 m. The release does **not** improve
gusty scenarios — their residual error is driven by the wind process, not by the
frozen integrator (see [disturbances.md](disturbances.md)).

**M6 caution:** the release makes the tuning cost surface **discontinuous** —
crossing `integralDwell` switches a different integration law on. Treat
`integralDwell` and `integralTrimGain` as fixed for comparisons, or tune them
deliberately with a fine sweep and report both sides of the threshold; do not
feed them to an optimiser that assumes a smooth response.

Thrust is clamped to `[0.1·mg, 0.8·ΣkTω_max²]` — `[1.4715, 48] N` for the hex —
leaving margin for torque authority. The desired tilt is clamped to 30°, which
caps horizontal acceleration at `g·tan(30°) = 5.66 m/s²`.

## Gains and bandwidth

| Loop | Gains | Limits |
|---|---|---|
| Position | `kp = 0.25 s⁻²`, `ki = 0.01 s⁻³`, `kd = 1.0 s⁻¹`, `integralSeparation = 0.1 m` | `a_xy ≤ 6`, `a_z ≤ 3 m/s²`; tilt 30°; `T ∈ [1.4715, 48] N` |
| Attitude | tilt `1.2 s⁻¹`, yaw `0.8 s⁻¹` | tilt 2.5, yaw 1.2, total 3.0 rad/s |
| Rate | `kp = 3.0 s⁻¹`, `ki = 0.5 s⁻²`, `kd = 0.02` | `[20, 20, 1.8835] rad/s²`; derivative cutoff 50 rad/s |

The motor lag is `τ = 0.05 s`, so `1/(3τ) = 6.67 rad/s` is the conservative
bandwidth ceiling. The rate loop crosses over near `3 rad/s` (phase margin
≈80°, closed-loop poles near −3.45 and −16.37 rad/s); `ki = 0.5` places the PI
zero far enough from the slow pole to avoid the ~4.6 s tail that `ki = 1.5`
produced. The attitude loop is roughly 3× slower (1.2 rad/s), and the position
loop roughly 2.4× slower again (`ω_n = 0.5 rad/s`, critically damped PD part).
The position loop is overdamped because linear drag adds damping: with
`c1 = 0.25 N/(m/s)` and `m = 1.5 kg`, the characteristic polynomial is
`s² + 1.167 s + 0.25` with roots `−0.283` and `−0.884 rad/s`, which explains the
~15 s 2 % settling time; this is the headroom M6 autotuning will use.
All gains are plain parameters; M6 will tune them.

## Measured step responses

- Rate step (1 rad/s): rise (10–90 %) **0.5590 s**, overshoot **4.61 %**,
  5 % settling **0.7450 s**.
- Attitude steps: 10° roll and 10° pitch settle to 0.5° in **1.591 s** with
  **0.2346°** overshoot; 20° yaw in **3.209 s** with no overshoot.
- Large-angle: 200° yaw (forcing the `q_err.w < 0` flip) ends within **0.3264°**
  after 18 s with integrated path **2.7868 rad** vs **2.7925 rad** shortest.
- Hover hold: max error **1.117e-12 m** over 10 s.
- 5 m position step: overshoot **0.06 %**, 2 % settling **15.235 s**,
  steady-state error **0.00296 m**, max tilt **5.53°**.
- Ground takeoff: final `z = 5.0197 m`, last contact at **0.142 s**.
- 500 m command then 5 m: `|∫| = 0.276 m·s`, max tilt **31.4°**, final
  `[-0.006, 0, 4.986] m`; no NaN and no mixer saturation.
- Quad/hex/octo with arms 0.20/0.25/0.30 m and the same gains: all settle in
  **15.052 s** to `x = 2.0080 m`.

## Demo

`npm run demo:flight` runs six 1 kHz phases (ground takeoff, four 5 m square
legs, hold) and writes `out/flight.csv` (gitignored). The 35 columns contain
time, phase, setpoint position/yaw, actual position, velocity, quaternion,
display Euler angles, angular rates, commanded wrench, saturation/contact
flags, and all six motor speeds. A printed SHA-256 makes two runs comparable;
as in [conventions.md](conventions.md), byte-identical output is guaranteed only
within one JavaScript engine. Reference run: **105 000 rows × 35 columns**,
**43.74 MB**; overshoot 0 m.

The printed "final-3s max error" is the **maximum error over the final 3 s of each
20 s leg** (`t = 17–20 s`, i.e. 1.8–4.8 s after the 2 % settling time). It is a
tail maximum, not a steady-state error: the reference run gives **0.064–0.081 m**
in that window, while the error at `t = 20 s` is only **0.023–0.032 m**. A single
5 m step measured at `t = 25 s` is **0.0035 m**, matching the steady-state error
quoted above; the square pattern simply uses a shorter observation window.

## Limitations

- No sensor model: `Measurement` is the true simulator state; M5 will supply
  noisy estimates through the same interface.
- No rate decimation, trajectory generator, or setpoint interpolation; a large
  setpoint step is handled by integral separation rather than a velocity profile.
- Yaw authority is computed at hover thrust and is conservative at the high end
  of the thrust range.
- Diagonal-inertia feedforward; off-diagonal presets would need the full tensor.
- No autotuning, wind, noise, battery sag, or viewer yet (M4–M6).
