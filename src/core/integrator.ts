/**
 * Fixed-step integrators for the rigid-body state.
 *
 * Both integrators are pure: they return a new state and never mutate their
 * inputs. The quaternion is treated as a plain 4-vector during the stages of
 * RK4 and is renormalized once after the full step (never inside sub-stages),
 * which preserves RK4's order while preventing drift away from unit norm.
 *
 * Determinism: no adaptive stepping, no randomness, no wall-clock time.
 */

import { quatNormalize, type Quat } from "./math/quat";
import type { Vec3 } from "./math/vec3";
import {
  derivatives,
  type RigidBodyInputs,
  type RigidBodyParams,
  type RigidBodyState,
  type StateDerivative,
} from "./rigidBody";

function assertValidDt(dt: number): void {
  if (!Number.isFinite(dt) || dt <= 0) {
    throw new RangeError("dt must be a finite positive number");
  }
}

/** state + h * derivative, component-wise. The quaternion is not normalized. */
function addScaled(
  state: RigidBodyState,
  derivative: StateDerivative,
  h: number,
): RigidBodyState {
  const q = state.orientation;
  const qd = derivative.orientation;
  return {
    position: [
      state.position[0] + h * derivative.position[0],
      state.position[1] + h * derivative.position[1],
      state.position[2] + h * derivative.position[2],
    ],
    velocity: [
      state.velocity[0] + h * derivative.velocity[0],
      state.velocity[1] + h * derivative.velocity[1],
      state.velocity[2] + h * derivative.velocity[2],
    ],
    orientation: [
      q[0] + h * qd[0],
      q[1] + h * qd[1],
      q[2] + h * qd[2],
      q[3] + h * qd[3],
    ] as Quat,
    angularVelocity: [
      state.angularVelocity[0] + h * derivative.angularVelocity[0],
      state.angularVelocity[1] + h * derivative.angularVelocity[1],
      state.angularVelocity[2] + h * derivative.angularVelocity[2],
    ],
  };
}

/** (k1 + 2*k2 + 2*k3 + k4) / 6 for a Vec3. */
function weightedVec3(k1: Vec3, k2: Vec3, k3: Vec3, k4: Vec3): Vec3 {
  return [
    (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6,
    (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6,
    (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6,
  ];
}

/** (k1 + 2*k2 + 2*k3 + k4) / 6 for a quaternion 4-vector. */
function weightedQuat(k1: Quat, k2: Quat, k3: Quat, k4: Quat): Quat {
  return [
    (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6,
    (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6,
    (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6,
    (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]) / 6,
  ];
}

/** Classical RK4 weighted average of the four stage derivatives. */
function weightedDerivative(
  k1: StateDerivative,
  k2: StateDerivative,
  k3: StateDerivative,
  k4: StateDerivative,
): StateDerivative {
  return {
    position: weightedVec3(k1.position, k2.position, k3.position, k4.position),
    velocity: weightedVec3(k1.velocity, k2.velocity, k3.velocity, k4.velocity),
    orientation: weightedQuat(
      k1.orientation,
      k2.orientation,
      k3.orientation,
      k4.orientation,
    ),
    angularVelocity: weightedVec3(
      k1.angularVelocity,
      k2.angularVelocity,
      k3.angularVelocity,
      k4.angularVelocity,
    ),
  };
}

/**
 * One classical RK4 step over the full state:
 *   k1 = f(x), k2 = f(x + dt/2 * k1), k3 = f(x + dt/2 * k2),
 *   k4 = f(x + dt * k3), x_next = x + dt/6 * (k1 + 2k2 + 2k3 + k4).
 * The orientation quaternion is renormalized after the full step.
 */
export function step(
  state: RigidBodyState,
  params: RigidBodyParams,
  inputs: RigidBodyInputs,
  dt: number,
): RigidBodyState {
  assertValidDt(dt);

  const k1 = derivatives(state, params, inputs);
  const k2 = derivatives(addScaled(state, k1, dt / 2), params, inputs);
  const k3 = derivatives(addScaled(state, k2, dt / 2), params, inputs);
  const k4 = derivatives(addScaled(state, k3, dt), params, inputs);

  const next = addScaled(state, weightedDerivative(k1, k2, k3, k4), dt);
  return { ...next, orientation: quatNormalize(next.orientation) };
}

/**
 * Explicit Euler baseline: x_next = x + dt * f(x).
 * Renormalizes the quaternion after the full step for a fair comparison.
 */
export function eulerStep(
  state: RigidBodyState,
  params: RigidBodyParams,
  inputs: RigidBodyInputs,
  dt: number,
): RigidBodyState {
  assertValidDt(dt);

  const k1 = derivatives(state, params, inputs);
  const next = addScaled(state, k1, dt);
  return { ...next, orientation: quatNormalize(next.orientation) };
}
