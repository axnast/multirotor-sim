# Physics notes

## State and parameters

The full state is `x = [p, v, q, ω]` with 13 scalar components (3 + 3 + 4 + 3):

- `p` — centre-of-mass position, world frame (m)
- `v` — centre-of-mass velocity, world frame (m/s)
- `q` — unit orientation quaternion, body → world
- `ω` — angular velocity, body frame (rad/s)

Parameters are mass `m`, body-frame inertia tensor `I`, and world-frame gravity
`g` (default `[0, 0, -9.81]`). The origin of the body frame is the centre of
mass, so there are no parallel-axis terms.

## Equations of motion

`derivatives(state, params, inputs)` evaluates:

```
p_dot = v
v_dot = F_world / m + g
q_dot = 0.5 * q ⊗ (0, ω_body)
ω_dot = I⁻¹ (τ_body - ω_body × (I ω_body))
```

The last term is the **gyroscopic term**. It comes from Euler's rigid-body
equation `I ω_dot + ω × (I ω) = τ`. It is essential: without it, a torque-free
body's `ω` would stay constant. Note that energy and angular-momentum
*magnitude* alone do **not** detect a missing gyroscopic term, because a body
with constant `ω` still conserves both. The symmetric-top analytic precession
test and the intermediate-axis tumble test are what actually validate it.

`F_world` and `τ_body` are inputs to the derivative function and default to
zero. Milestone 2 supplies motor forces and moments through the same inputs.

## Integration

### Why RK4

Classical fourth-order Runge–Kutta is used because it is accurate at moderate
fixed steps, deterministic, easy to reason about, and straightforward to port
to Rust/WASM. It is well matched to this smooth ODE. Explicit Euler is kept as
a baseline for comparison tests only.

For a step `h`:

```
k1 = f(x)
k2 = f(x + h/2 * k1)
k3 = f(x + h/2 * k2)
k4 = f(x + h * k3)
x_next = x + h/6 * (k1 + 2*k2 + 2*k3 + k4)
```

Halving `h` should reduce the global error by about 16× for RK4 and about 2×
for Euler. The convergence test uses an asymmetric tumbling body at
`h = 1/50, 1/100, 1/200 s` against a `1/12800 s` reference solution, keeping
errors well above machine precision.

### Why renormalize the quaternion

The quaternion norm is conserved analytically by `q_dot = 0.5 q ⊗ (0, ω)`, but
the linear combinations inside RK4 do not preserve it exactly. Without
renormalization the norm drifts, the rotation matrix stops being orthonormal,
and energy/momentum diagnostics degrade. The integrator therefore projects the
quaternion back to unit norm **once after each full step**, never inside the
sub-stages, so RK4's order is preserved. The Euler baseline does the same for a
fair comparison.

## Diagnostics

For a state `x`:

```
KE_trans = 0.5 * m * |v|²
KE_rot   = 0.5 * ω · (I ω)
PE       = -m * (g · p)              (zero at the world origin)
L_world  = R(q) * (I ω)              (angular momentum, world frame)
```

With no external force/torque and constant gravity, total energy `KE + PE` is
conserved. In torque-free rotational motion, `KE_rot` and `|L_world|` are
conserved. These are checked over 10–20 s in `tests/integrator.test.ts`.

## Model limits

Milestone 1 assumes a single rigid body with constant mass and inertia. There is
no ground contact; the demo box falls through `z = 0`. There is no aerodynamic
drag, motor lag, actuator saturation, sensor noise, or wind.

## Further reading

- Schaub & Junkins, *Analytical Mechanics of Space Systems* — rigid-body
  kinematics, Euler equations, and attitude representations.
- Stevens & Lewis, *Aircraft Control and Simulation* — Newton–Euler dynamics
  and rigid-body integration.
