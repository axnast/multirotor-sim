# Conventions

This document fixes the conventions used by `src/core`. The core is pure,
dependency-free TypeScript with no DOM, Node, or three.js imports so that it
can be ported to Rust/WASM later.

## Coordinate frames

- **World frame:** right-handed, **Z-up**. `x` is forward/east, `y` is
  left/north, `z` is up. Gravity is `[0, 0, -9.81]` by default.
- **Body frame:** right-handed, attached to the rigid body at its centre of
  mass. `x` is forward, `y` is left, `z` is up.
- Position `p` and velocity `v` are expressed in the **world** frame.
- Angular velocity `ω` and applied torque `τ` are expressed in the **body**
  frame, about the centre of mass. Applied force `F` is expressed in the
  **world** frame.
- The inertia tensor `I` is constant, symmetric, positive definite, and
  expressed in the **body** frame about the centre of mass.

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

## Scope of Milestone 1

A single rigid body with constant mass and inertia, gravity, and externally
supplied force/torque inputs. No motors, mixer, aerodynamics, drag, sensors,
controllers, or ground contact yet. Those inputs are included now so Milestone 2
can plug in a multirotor model without refactoring the core.
