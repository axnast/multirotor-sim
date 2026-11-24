/**
 * Rigid-body dynamics for a single constant-mass body.
 *
 * Conventions (see docs/conventions.md):
 * - Position and velocity are expressed in the world frame; the orientation
 *   quaternion q rotates body vectors into the world frame.
 * - Angular velocity ω and applied torque τ are expressed in the body frame,
 *   about the centre of mass. Applied force F is expressed in the world frame.
 * - The inertia tensor is constant and expressed in the body frame.
 * - No aerodynamics, motors, ground contact, or other effects are modelled.
 *
 * Equations integrated by the integrator module:
 *   p_dot = v
 *   v_dot = F_world / m + g
 *   q_dot = 0.5 * q ⊗ (0, ω_body)
 *   ω_dot = I⁻¹ (τ_body - ω_body × (I ω_body))
 */

import { mat3Inverse, mat3MulVec, type Mat3 } from "./math/mat3";
import { quatDerivative, type Quat } from "./math/quat";
import {
  vec3Add,
  vec3Cross,
  vec3Scale,
  vec3Sub,
  type Vec3,
} from "./math/vec3";

/** Standard gravity in the Z-up world frame (m/s²). */
export const DEFAULT_GRAVITY: Vec3 = [0, 0, -9.81];

export interface RigidBodyState {
  /** Centre-of-mass position in the world frame (m). */
  readonly position: Vec3;
  /** Centre-of-mass velocity in the world frame (m/s). */
  readonly velocity: Vec3;
  /** Unit quaternion rotating body vectors into the world frame (w, x, y, z). */
  readonly orientation: Quat;
  /** Angular velocity in the body frame (rad/s). */
  readonly angularVelocity: Vec3;
}

export interface RigidBodyParams {
  /** Mass (kg), strictly positive. */
  readonly mass: number;
  /** Inertia tensor about the centre of mass, in the body frame (kg·m²). */
  readonly inertia: Mat3;
  /** Gravity acceleration in the world frame (m/s²). Defaults to DEFAULT_GRAVITY. */
  readonly gravity?: Vec3;
}

/** External inputs. Both default to zero, so Milestone 2 can plug motors in. */
export interface RigidBodyInputs {
  readonly forceWorld: Vec3;
  readonly torqueBody: Vec3;
}

/** Time derivative of RigidBodyState. `orientation` is q_dot, not a unit quaternion. */
export interface StateDerivative {
  readonly position: Vec3;
  readonly velocity: Vec3;
  readonly orientation: Quat;
  readonly angularVelocity: Vec3;
}

export const ZERO_INPUTS: RigidBodyInputs = {
  forceWorld: [0, 0, 0],
  torqueBody: [0, 0, 0],
};

function assertFiniteVec3(name: string, v: Vec3): void {
  if (!Number.isFinite(v[0]) || !Number.isFinite(v[1]) || !Number.isFinite(v[2])) {
    throw new RangeError(`${name} must contain only finite numbers`);
  }
}

function assertFiniteQuat(name: string, q: Quat): void {
  if (
    !Number.isFinite(q[0]) ||
    !Number.isFinite(q[1]) ||
    !Number.isFinite(q[2]) ||
    !Number.isFinite(q[3])
  ) {
    throw new RangeError(`${name} must contain only finite numbers`);
  }
}

function assertValidParams(params: RigidBodyParams): void {
  if (!Number.isFinite(params.mass) || params.mass <= 0) {
    throw new RangeError("mass must be a finite positive number");
  }
  for (const value of params.inertia) {
    if (!Number.isFinite(value)) {
      throw new RangeError("inertia must contain only finite numbers");
    }
  }
  const gravity = params.gravity ?? DEFAULT_GRAVITY;
  assertFiniteVec3("gravity", gravity);
  mat3Inverse(params.inertia);
}

function assertValidState(state: RigidBodyState): void {
  assertFiniteVec3("position", state.position);
  assertFiniteVec3("velocity", state.velocity);
  assertFiniteQuat("orientation", state.orientation);
  assertFiniteVec3("angularVelocity", state.angularVelocity);
  if (
    state.orientation[0] === 0 &&
    state.orientation[1] === 0 &&
    state.orientation[2] === 0 &&
    state.orientation[3] === 0
  ) {
    throw new RangeError("orientation must not be the zero quaternion");
  }
}

/**
 * Evaluate the rigid-body equations of motion at a single state.
 *
 * p_dot = v
 * v_dot = F_world / m + g
 * q_dot = 0.5 * q ⊗ (0, ω_body)
 * ω_dot = I⁻¹ (τ_body - ω_body × (I ω_body))
 */
export function derivatives(
  state: RigidBodyState,
  params: RigidBodyParams,
  inputs: RigidBodyInputs = ZERO_INPUTS,
): StateDerivative {
  assertValidState(state);
  assertValidParams(params);
  assertFiniteVec3("forceWorld", inputs.forceWorld);
  assertFiniteVec3("torqueBody", inputs.torqueBody);

  const gravity = params.gravity ?? DEFAULT_GRAVITY;
  const inertiaInverse = mat3Inverse(params.inertia);
  const iOmega = mat3MulVec(params.inertia, state.angularVelocity);

  return {
    position: state.velocity,
    velocity: vec3Add(vec3Scale(inputs.forceWorld, 1 / params.mass), gravity),
    orientation: quatDerivative(state.orientation, state.angularVelocity),
    angularVelocity: mat3MulVec(
      inertiaInverse,
      vec3Sub(inputs.torqueBody, vec3Cross(state.angularVelocity, iOmega)),
    ),
  };
}
