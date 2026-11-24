/**
 * Energy and momentum diagnostics for conservation tests and the demo.
 * Pure functions; no hidden state and no external dependencies.
 */

import { mat3MulVec } from "./math/mat3";
import { quatRotateVector } from "./math/quat";
import { vec3Dot, vec3LengthSquared, type Vec3 } from "./math/vec3";
import { DEFAULT_GRAVITY, type RigidBodyParams, type RigidBodyState } from "./rigidBody";

function gravityOf(params: RigidBodyParams): Vec3 {
  return params.gravity ?? DEFAULT_GRAVITY;
}

/** Translational kinetic energy: ½ m |v|². */
export function translationalKineticEnergy(
  state: RigidBodyState,
  params: RigidBodyParams,
): number {
  return 0.5 * params.mass * vec3LengthSquared(state.velocity);
}

/** Rotational kinetic energy: ½ ω · (I ω). */
export function rotationalKineticEnergy(
  state: RigidBodyState,
  params: RigidBodyParams,
): number {
  return 0.5 * vec3Dot(state.angularVelocity, mat3MulVec(params.inertia, state.angularVelocity));
}

/** Total kinetic energy: translational + rotational. */
export function kineticEnergy(
  state: RigidBodyState,
  params: RigidBodyParams,
): number {
  return translationalKineticEnergy(state, params) + rotationalKineticEnergy(state, params);
}

/** Gravitational potential energy: -m (g · p), zero at the world origin. */
export function potentialEnergy(
  state: RigidBodyState,
  params: RigidBodyParams,
): number {
  return -params.mass * vec3Dot(gravityOf(params), state.position);
}

/** Total mechanical energy: KE + PE. */
export function totalEnergy(
  state: RigidBodyState,
  params: RigidBodyParams,
): number {
  return kineticEnergy(state, params) + potentialEnergy(state, params);
}

/** Angular momentum expressed in the world frame: L_world = R(q) (I ω_body). */
export function angularMomentumWorld(
  state: RigidBodyState,
  params: RigidBodyParams,
): Vec3 {
  return quatRotateVector(
    state.orientation,
    mat3MulVec(params.inertia, state.angularVelocity),
  );
}
