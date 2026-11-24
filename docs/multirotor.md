# Multirotor model (Milestone 2)

This document specifies the multirotor layer built on top of the Milestone 1
rigid body. The rigid-body equations, integrators and energy diagnostics are
**unchanged**; motors, allocation, drag and ground contact are added through the
same force/torque inputs and an extended state. There is no controller yet.

Source of truth for every claim below:

| Topic | Module |
|---|---|
| Defaults, validation, layouts | `src/core/multirotor/config.ts` |
| Thrust, reaction torque, lag, clamping | `src/core/multirotor/motors.ts` |
| Allocation matrix and mixer | `src/core/multirotor/mixer.ts` |
| Assembly, RK4 step, drag, environment | `src/core/multirotor/dynamics.ts` |
| Ground contact | `src/core/multirotor/ground.ts` |
| Three-phase trace | `scripts/hoverDemo.ts` |

See [conventions.md](conventions.md) for frames, quaternion and Euler rules,
and [physics.md](physics.md) for the underlying rigid-body equations.

## Frames, units and signs

- **World:** right-handed, Z-up; default gravity `[0, 0, -9.81]` m/s².
- **Body:** right-handed **FLU** — `+x` forward, `+y` left, `+z` up. The origin
  is the centre of mass. `q` rotates body vectors into world vectors.
- Position and velocity are world-frame; angular velocity `ω` and torque `τ`
  are body-frame; applied forces are world-frame. SI units throughout.
- Rotor speed `ω ≥ 0` is a **magnitude** in rad/s. `spin = +1` means
  counter-clockwise seen from above (about body `+z`); the reaction torque on
  the airframe is clockwise: `Q = -spin · kQ · ω · |ω|` about body `z`, which
  equals `-spin · kQ · ω²` on the physical domain `ω ≥ 0`.
- Thrust acts along body `+z`. Every preset hub lies at body `z = 0`.
- A body wrench is `[T, τx, τy, τz]`: total thrust (N) and body torque (N·m)
  in x/y/z order.

## Motor model

```
f_i   = kT_i · ω_i²                          (N)
Q_i   = -spin_i · kQ_i · ω_i · |ω_i|          (N·m, about body z)
dω_i/dt = (ω_cmd,i - ω_i) / τ_i               (first-order lag)
```

Commands are zero-order held over each integration step and clamped to
`[minOmega, maxOmega]` before use. The motor speeds are part of the extended
state; RK4 integrates them together with `[p, v, q, ω]`. Final speeds are
projected back onto their limits after the step.

Default hex constants (placeholder values, chosen to be plausible, not
measured):

| Constant | Value |
|---|---|
| mass `m` | 1.5 kg |
| arm radius `R` | 0.25 m |
| `kT` | 1e-5 N/(rad/s)² |
| `kQ` | 1.5e-7 N·m/(rad/s)² |
| `ω` limits | `[0, 1000]` rad/s |
| lag `τ` | 0.05 s |
| linear drag `c1` | `[0.25, 0.25, 0.4]` N/(m/s) |
| quadratic drag `c2` | `[0.02, 0.02, 0.03]` N/(m/s)² |

Default inertia is a thin ring, `Ixx = Iyy = ½mR² = 0.046875` and
`Izz = mR² = 0.09375` kg·m². Weight is `mg = 14.715 N`, maximum total thrust
is `6·kT·1000² = 60 N` (`T/W = 4.077`), and an even hover split needs
`ω = 495.227 rad/s` per rotor.

## Layouts

| Factory | Rotors | First hub angle | Angles | Spins |
|---|---|---|---|---|
| `makeQuad()` | 4 | 45° (`π/4`) | 45°, 135°, 225°, 315° | `+ − + −` |
| `makeHex()` | 6 | 0° | 0°, 60°, …, 300° | `+ − + − + −` |
| `makeOcto()` | 8 | 0° | 0°, 45°, …, 315° | `+ − + − + − + −` |

Hub `i` sits at `[R cos θ_i, R sin θ_i, 0]`; angle is measured
counter-clockwise from body `+x`, and spins alternate by index starting with
`+1`. Factories accept optional overrides (`mass`, `armLength`, `inertia`,
`gravity`, `kT`, `kQ`, `maxOmega`, `minOmega`, `motorTimeConstant`,
`linearDrag`, `quadraticDrag`, `groundPlane`), resolve a complete config and
validate it. Motor order is fixed and must be preserved by state and commands.

## Allocation and mixer

The allocation matrix `A` is `4 × N` and maps rotor **thrusts** to body wrench,
`A · f = [T, τx, τy, τz]`, with column `i`:

```
row 0:  1                       T   = Σ f_i
row 1:  y_i                     τx  = Σ y_i f_i
row 2: -x_i                     τy  = Σ -x_i f_i
row 3: -spin_i · kQ_i/kT_i      τz  = Σ -spin_i kQ_i ω_i|ω_i|
```

`kQ/kT` has units of length (default `0.015 m`), so row 3 converts newtons to
newton-metres. Row 3 is the reaction-torque model above rewritten with
`f_i = kT_i ω_i²`.

For `N > 4` the system is underdetermined. The mixer uses the minimum-norm
pseudo-inverse `M = A⁺ = Aᵀ(AAᵀ)⁻¹` (`N × 4`); the `4 × 4` Gram matrix is
inverted by pure Gauss–Jordan elimination with scaled partial pivoting, and a
singular geometry throws `RangeError`. The identity is

```
A · M = I₄          (and A · M · A = A)
```

not `M · A`. `A·M = I₄` is pinned for quad, hex and octo; `A·M·A = A` for hex
and octo. `mix()` returns
speed commands; `mixDetailed()` additionally reports achieved rotor thrusts,
the achieved wrench `A · actualThrusts`, and the `saturated` flag.

## Saturation policy

Applied in this exact order:

1. Clamp the requested total thrust into `[Σ kT minΩ², Σ kT maxΩ²]`
   (`[0, 60] N` for the default hex) **before** solving torque.
2. At the requested torque, intersect all per-motor feasible intervals for
   total thrust. If a feasible `T` exists, use the value **nearest** the
   clamped request (`mixer.ts:395–397`); `T` can therefore move slightly to
   preserve the requested torque.
3. If no `T` works, hold the clamped thrust and scale **yaw** by the largest
   feasible `α_z ∈ [0,1]`; roll and pitch stay intact.
4. If roll/pitch are still infeasible, scale them together by `α_xy`.
5. If no solution exists even at zero torque (pathological heterogeneous
   limits), fall back to a clamped uniform split.
6. Clamp each thrust to its motor limits, convert with `ω = sqrt(f/kT)`, clamp
   speeds, and recompute the achieved wrench.

`saturated` is true when thrust was clamped or moved, either torque scale is
below 1, or the fallback ran. The minimum-norm allocator is **conservative**
with unequal motor limits: a hex with one rotor capped at 800 rad/s cannot use
all of its remaining thrust authority. A capability-weighted box-QP is
deliberately out of scope for M2.

## Drag and environment

```
v_air_world = v_world - wind_world
v_air_body  = R(q)ᵀ v_air_world
F_drag_body = -(c1 ⊙ v_air_body + c2 ⊙ |v_air_body| · v_air_body)
```

The quadratic term uses the **isotropic** airspeed magnitude `|v_air_body|`,
not a per-axis square. Drag acts at the centre of mass and therefore adds **no
torque**. Wind is a world-frame velocity, held constant across a step; the
`Environment` argument (default `STILL_AIR`) is already plumbed through the
derivative and step functions. Gusts, sensor noise and battery sag are M5.

## Integration and ground contact

State is `[p, v, q, ω, ω_0 … ω_{N-1}]`. One `stepMultirotor` call:

1. clamps and zero-order holds the commands across all four RK4 stages;
2. evaluates forces with a **unit** quaternion, leaving stage quaternions
   unnormalized;
3. normalizes `q` **once** after the full step and projects motor speeds onto
   their limits;
4. resolves ground contact, if enabled, on the fully resolved body.

`dt` is caller-chosen; there is no substepping. The code documents
`dt ≤ τ/10 = 5 ms` for sub-0.01 % lag error and an RK4 linear stability limit
of `dt < 2.79 τ ≈ 139.5 ms`. The demo uses **`dt = 1 ms`**.

Ground contact is opt-in (`config.groundPlane`, default `false`). The plane is
infinite at world `z = 0` with normal `+z`. Per-step `GroundParams` default to
`restitution e = 0` (fully inelastic, range `[0,1]`) and isotropic Coulomb-style
friction `μ = 0.6` (non-negative); parameters are validated even when the plane
is disabled. For an inward velocity the normal impulse is
`J_n = (1+e) m |v_z|`; the tangential impulse is
`min(μ J_n, m |v_t|)` and preserves direction. Tiny components are zeroed to
avoid `-0` and resting jitter.

Contact changes **position and linear velocity only**. It applies no torque:
a resting vehicle rotates freely under an applied torque, and a tilted body is
lifted so that only its centre of mass lies on the plane — its geometry may
still intersect it. There is no tipping, support polygon, landing gear, rolling
resistance, rotational ground damping, ground effect, downwash or rotor strike.
A ground phase must therefore command equal motor speeds (zero net torque).
`multirotorDerivatives` remains **free-flight**; contact is a non-smooth
post-step event, not part of the ODE. `stepMultirotorDetailed` exposes
`ground: GroundContactResult | null`; `stepMultirotor` is the thin wrapper.

## Known simplifications

- No rotor gyroscopic torque and no spin-up reaction torque from rotor
  inertia; only steady `kQ ω|ω|` is modelled.
- No blade flapping, inflow, ground effect, air-density variation or drag
  torque.
- No motor-failure model: `MotorConfig` has no fault flag, the dynamics has no
  per-motor override, and the mixer assumes every rotor is available. Matrix
  test **11** (hex motor-out authority) is deliberately skipped for this reason.
- Placeholder motor/drag coefficients; no identified airframe data.
- No controller, sensors, battery model or wind disturbances yet (M3–M5).

## Demo and CSV trace

`npm run demo:hover` runs three independent 10 s phases at 1 kHz, each from a
fresh state, and writes `out/hover.csv` (gitignored):

| Phase | Config | Initial | Command | Steps |
|---|---|---|---|---|
| `hover` | `makeHex()` | `z = 5 m`, hover speeds | `[mg, 0, 0, 0]` | 10 000 |
| `takeoff` | `makeHex({ groundPlane: true })` | `z = 0`, stopped rotors | `[1.1 mg, 0, 0, 0]` | 10 000 |
| `roll-pulse` | `makeHex()` | `z = 5 m`, hover speeds | `[mg, 0.01, 0, 0]` for steps `1000 ≤ i < 1100`, else hover | 10 000 |

The takeoff command is collective, so the symmetric hex gets equal speeds and
zero net torque; this matters because contact applies no torque. Lift-off is
predicted by `τ·ln(1/(1 − 1/√1.1)) = 0.153 s`.

The single CSV has one header and **41 columns**: `phase`, global/phase time;
position, velocity, quaternion, display-only roll/pitch/yaw, angular velocity;
the four commanded wrench components; the four **achieved** wrench components;
`saturated`, `contact`, penetration, normal and friction impulses; kinetic,
potential and total energy; and all six rotor speeds. A row holds the state at
`t` plus the command over `[t, t+dt)`; ground columns describe the step that
**ended** at that row. Numbers use `toFixed(9)`, booleans are `0`/`1`.

Determinism: fixed phase order and step counts, no clock, randomness or locale
formatting. The script prints a SHA-256 so two runs can be compared
byte-for-byte. Output is bit-identical on the same JavaScript engine;
cross-engine last-bit differences are possible (`Math.sin`/`cos`), as noted in
[conventions.md](conventions.md). The hash is printed at runtime and is not a
fixed specification.

**Reference run** (default hex, 1 kHz — reproducible with `npm run demo:hover`):

| Phase | Measured |
|---|---|
| `hover` | max drift `2.152e-11 m`, final `z = 5.000 m` |
| `takeoff` | `154` contact steps, max penetration `4.904e-6 m`, max `J_n = 1.471e-2 N·s`; lift-off `0.155 s` (analytic `0.153 s`); final `z = 21.185 m`, `v_z = 2.922 m/s` |
| `roll-pulse` | peak `|ω_x| = 0.0213 rad/s`, peak roll `0.1899 rad`, final `y = -16.627 m`, final `z = 3.941 m` (`3.940709533 m` in the CSV) |

After step 2000 (`t ≥ 2 s`), with zero body torque and no angular drag, `ω_x`
stays constant: max `|Δω_x| = 1.424e-10 rad/s` and
`max(|ω_y|, |ω_z|) = 8.787e-16 rad/s`, both below `1e-9`. Phase C is
**open-loop**: the residual roll tilts body thrust and integrates into lateral
drift until a controller closes the loop in M3. The trace is `30 003` data rows
plus a header, `41` columns, `14.72 MB`.

## Validation

- `tests/multirotorIntegration.test.ts`: matrix 1–17 (hover, drag, lag,
  saturation, frame covariance, determinism, and ground in 14–17), with test 11
  skipped as explained above.
- `tests/multirotorMixer.test.ts`: the four allocation rows, `A·M = I₄`,
  `A·M·A = A`, saturation order and unequal-limit conservatism.
- `tests/multirotorGround.test.ts` (`G1`–`G11`): impulse and friction levers,
  sliding stop and parameter validation. For `μ = 0.6` the continuous sliding
  stop is `1/(2μg) = 0.08494733 m`; the discrete stepper stops at
  `0.08544761 m` after `170` steps at 1 kHz.
- `npm run typecheck` and `npm test` are required before every commit.
