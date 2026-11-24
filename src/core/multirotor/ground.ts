/**
 * Crude ground plane contact for multirotors.
 *
 * The ground is an infinite, flat, fixed plane at world z = 0 with normal +z.
 * Contact is a single-point impulse at the centre of mass, resolved once
 * *after* each integration step (see dynamics.ts). It is not part of the
 * derivative vector field: the ODE stays smooth and free-flight, while the
 * non-smooth impact is handled by the stepper. This avoids penalty stiffness
 * and keeps the model stable at 1 kHz.
 *
 * Contact changes **position and linear velocity only**. Orientation and
 * angular velocity are returned unchanged: there is no contact torque, no
 * tipping, no support polygon and no rotational ground damping. A vehicle
 * resting on the plane therefore rotates freely under any applied torque, and
 * a tilted body is lifted so that only its centre of mass sits on the plane;
 * its geometry may still intersect it. A ground phase must command equal
 * motor speeds (zero net torque), otherwise the vehicle spins in place.
 *
 * No ground effect, downwash, rotor strike, rolling resistance, surface
 * roughness, landing gear or multi-point contact is modelled.
 *
 * Everything here is pure: inputs are never mutated and no state is hidden.
 */

import type { Quat } from "../math/quat";
import type { Vec3 } from "../math/vec3";
import type { RigidBodyState } from "../rigidBody";

/** Tunable contact parameters. Ground contact itself is opt-in via the config. */
export interface GroundParams {
  /** Coefficient of restitution in [0, 1]. Default 0 (fully inelastic). */
  readonly restitution: number;
  /** Isotropic Coulomb friction coefficient μ >= 0. Default 0.6. */
  readonly friction: number;
}

/** Inelastic landing with a plausible friction placeholder. */
export const DEFAULT_GROUND_PARAMS: GroundParams = {
  restitution: 0,
  friction: 0.6,
};

/** The resolved body and diagnostics from one contact check. */
export interface GroundContactResult {
  /** A new body when contact occurred, otherwise the input body unchanged. */
  readonly body: RigidBodyState;
  /** True when the plane was touched (penetration or closing at z = 0). */
  readonly contacted: boolean;
  /** Penetration depth before correction (m, >= 0). */
  readonly penetration: number;
  /** Normal impulse magnitude (N·s, >= 0). */
  readonly normalImpulse: number;
  /** Tangential friction impulse magnitude (N·s, >= 0). */
  readonly frictionImpulse: number;
}

function assertFiniteNumber(name: string, value: number): void {
  if (!Number.isFinite(value)) {
    throw new RangeError(`${name} must be a finite number`);
  }
}

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

/** Throws a RangeError if the contact parameters are invalid. */
export function validateGroundParams(params: GroundParams): void {
  assertFiniteNumber("ground.restitution", params.restitution);
  if (params.restitution < 0 || params.restitution > 1) {
    throw new RangeError("ground.restitution must be in [0, 1]");
  }
  assertFiniteNumber("ground.friction", params.friction);
  if (params.friction < 0) {
    throw new RangeError("ground.friction must be non-negative");
  }
}

/**
 * Resolve contact with the plane at world z = 0.
 *
 * The normal impulse is `(1 + e) m |v_z|` for an inward velocity, and the
 * friction impulse is capped at `μ J_n` (and at the impulse needed to stop
 * the tangential motion). Tangential direction is preserved; tiny velocity
 * components are zeroed to avoid -0 and resting jitter.
 */
export function resolveGroundContact(
  body: RigidBodyState,
  mass: number,
  params: GroundParams = DEFAULT_GROUND_PARAMS,
): GroundContactResult {
  assertFiniteNumber("mass", mass);
  if (mass <= 0) {
    throw new RangeError("mass must be a finite positive number");
  }
  assertFiniteVec3("body.position", body.position);
  assertFiniteVec3("body.velocity", body.velocity);
  assertFiniteVec3("body.angularVelocity", body.angularVelocity);
  assertFiniteQuat("body.orientation", body.orientation);
  if (
    body.orientation[0] === 0 &&
    body.orientation[1] === 0 &&
    body.orientation[2] === 0 &&
    body.orientation[3] === 0
  ) {
    throw new RangeError("body.orientation must not be the zero quaternion");
  }
  validateGroundParams(params);

  const z = body.position[2];
  const vz = body.velocity[2];
  const penetrating = z < 0;
  const closingAtSurface = z === 0 && vz < 0;
  if (!penetrating && !closingAtSurface) {
    return {
      body,
      contacted: false,
      penetration: 0,
      normalImpulse: 0,
      frictionImpulse: 0,
    };
  }

  const penetration = Math.max(0, -z);

  let vx = body.velocity[0];
  let vy = body.velocity[1];
  let nextVz = vz;

  let normalImpulse = 0;
  if (vz < 0) {
    normalImpulse = -(1 + params.restitution) * mass * vz;
    nextVz = -params.restitution * vz;
  }

  let frictionImpulse = 0;
  const tangentialSpeed = Math.hypot(vx, vy);
  if (normalImpulse > 0 && tangentialSpeed > 0) {
    const desiredImpulse = mass * tangentialSpeed;
    const applied = Math.min(params.friction * normalImpulse, desiredImpulse);
    const scale = applied / desiredImpulse;
    vx -= vx * scale;
    vy -= vy * scale;
    frictionImpulse = applied;
  }

  if (Math.abs(vx) < 1e-12) vx = 0;
  if (Math.abs(vy) < 1e-12) vy = 0;
  if (Math.abs(nextVz) < 1e-12) nextVz = 0;

  const position: Vec3 = [body.position[0], body.position[1], 0];
  const velocity: Vec3 = [vx, vy, nextVz];
  return {
    body: { ...body, position, velocity },
    contacted: true,
    penetration,
    normalImpulse,
    frictionImpulse,
  };
}
