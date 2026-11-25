# Conventions

This document fixes the conventions used by `src/core`. The core is pure,
dependency-free TypeScript with no DOM, Node, or three.js imports so that it
can be ported to Rust/WASM later.

## Coordinate frames

- **World frame:** right-handed, **Z-up**. `x` is forward/east, `y` is
  left/north, `z` is up. Gravity is `[0, 0, -9.81]` by default.
- **Body frame:** right-handed **FLU** (forward, left, up), attached to the
  rigid body at its centre of mass. `x` is forward, `y` is left, `z` is up.
- Position `p` and velocity `v` are expressed in the **world** frame.
- Angular velocity `ω` and applied torque `τ` are expressed in the **body**
  frame, about the centre of mass. Applied force `F` is expressed in the
  **world** frame.
- The inertia tensor `I` is constant, symmetric, positive definite, and
  expressed in the **body** frame about the centre of mass.

## Rotors and layouts (Milestone 2)

- Rotor speed `ω ≥ 0` is a magnitude in rad/s. `spin = +1` means
  counter-clockwise seen from above, i.e. about body `+z`; the reaction torque
  on the airframe is clockwise about `-z`:
  `Q = -spin · kQ · ω · |ω|`, which equals `-spin · kQ · ω²` for `ω ≥ 0`.
- Rotor thrust acts along body `+z` and preset hubs lie at body `z = 0`.
- Hub angles are measured counter-clockwise from body `+x`: quad starts at
  45° (`π/4`); hex and octo start at 0°. Spins alternate by index, starting
  with `+1`. See [multirotor.md](multirotor.md).

## Quaternions

- A quaternion is the readonly tuple `[w, x, y, z]`: scalar part first, using
  the **Hamilton** convention (`i*j = k`).
- `q` is a **unit** quaternion that rotates body vectors into the world frame:
  `v_world = q ⊗ (0, v_body) ⊗ q*`.
- `q` and `-q` represent the same orientation.
- Composition: `R(a ⊗ b) = R(a) R(b)`, so `a ⊗ b` applies `b` first, then `a`.
- Kinematics with body-frame angular velocity:
  `q_dot = 0.5 * q ⊗ (0, ω_body)`.
- Rotation matrices are row-major and act on column vectors:
  `v_world = toRotationMatrix(q) * v_body`.

## Euler angles

- `quatToEulerZYX` returns `[roll, pitch, yaw]` in **radians**, using the
  intrinsic Z-Y-X sequence (yaw about body/world `z`, then pitch about `y`, then
  roll about `x`).
- Euler angles are for display/logging only. The quaternion is the source of
  truth. At pitch = ±90° the representation is singular (gimbal lock) and roll
  is reported as zero.

## Units

All quantities use SI units unless stated otherwise:

| Quantity | Unit |
|---|---|
| position, length | m |
| velocity | m/s |
| mass | kg |
| force | N |
| torque | N·m |
| inertia | kg·m² |
| time | s |
| angle | rad |
| angular velocity | rad/s |
| energy | J |

## Purity and determinism

- Core functions are data-in/data-out: readonly tuples and plain objects, no
  classes with hidden state, no mutation of inputs.
- Integrators use a **fixed step**. There is no adaptive stepping, randomness,
  or wall-clock time in the core.
- Outputs are bit-identical for identical inputs on the same JavaScript engine.
  Cross-engine results may differ in the last bits because `Math.sin`/`cos`
  implementations are not required to be bit-identical.
- The demo's printed SHA-256 (`npm run demo:hover`) is a checkable determinism
  artifact for one engine, not a cross-engine guarantee.

## Scope of Milestone 1

A single rigid body with constant mass and inertia, gravity, and externally
supplied force/torque inputs. No motors, mixer, aerodynamics, drag, sensors,
controllers, or ground contact yet. Those inputs are included now so Milestone 2
can plug in a multirotor model without refactoring the core.

## Scope of Milestone 2

Motors (`T = kT·ω²`, first-order lag and clamping), quad/hex/octo layouts, the
minimum-norm allocation mixer, body-frame air-relative drag with an `Environment`
wind argument, and opt-in single-point ground contact. The Milestone 1 rigid-body
core is unchanged; details and limitations are in [multirotor.md](multirotor.md).

## Viewer (Three.js) mapping

The browser viewer maps core Z-up coordinates to Three.js Y-up in exactly one
helper, `src/viewer/coords.ts`:

```
p_three = (x, z, -y)_core
q_three = [x, z, -y, w]_core   (Three stores x,y,z,w)
```

This is `R_x(-90°)` with determinant +1, so handedness is preserved: body `+x`
is Three `+x`, body `+y` is Three `-z`, body `+z` is Three `+y`. The conversion
is a view detail only; the physics frame conventions above are unchanged. See
[viewer.md](viewer.md) for the tests and rationale.
