/**
 * Quaternion primitives.
 *
 * Conventions (see docs/conventions.md):
 * - `Quat` is the readonly tuple [w, x, y, z] (scalar part first), using the
 *   Hamilton convention with i*j = k.
 * - A unit quaternion q rotates body-frame vectors into the world frame:
 *   v_world = q ⊗ (0, v_body) ⊗ q*.
 * - Body-frame angular velocity ω is used in the kinematic equation
 *   q_dot = 0.5 * q ⊗ (0, ω).
 * - Multiplication order is "first b, then a": R(a ⊗ b) = R(a) R(b).
 * - Euler output uses the intrinsic Z-Y-X sequence (yaw, pitch, roll) and is
 *   for display only; the quaternion is the source of truth.
 */

import { mat3, type Mat3 } from "./mat3";
import {
  vec3Add,
  vec3Cross,
  vec3Normalize,
  vec3Scale,
  type Vec3,
} from "./vec3";

export type Quat = readonly [number, number, number, number];

export const QUAT_IDENTITY: Quat = [1, 0, 0, 0];

export function quat(w: number, x: number, y: number, z: number): Quat {
  return [w, x, y, z];
}

/** Hamilton product q ⊗ p. Applies p first, then q. */
export function quatMultiply(a: Quat, b: Quat): Quat {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}

/** Quaternion conjugate q* = (w, -x, -y, -z); equals q⁻¹ for unit q. */
export function quatConjugate(q: Quat): Quat {
  return [q[0], -q[1], -q[2], -q[3]];
}

/** Unit-norm quaternion q / |q|. Throws for a zero-length input. */
export function quatNormalize(q: Quat): Quat {
  const norm = Math.sqrt(
    q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3],
  );
  if (norm === 0) {
    throw new RangeError("Cannot normalize a zero-length quaternion");
  }
  return [q[0] / norm, q[1] / norm, q[2] / norm, q[3] / norm];
}

/** Right-hand-rule rotation of `angleRad` about unit `axis`. */
export function quatFromAxisAngle(axis: Vec3, angleRad: number): Quat {
  if (angleRad === 0) {
    return QUAT_IDENTITY;
  }
  const n = vec3Normalize(axis);
  const half = angleRad / 2;
  const s = Math.sin(half);
  return [Math.cos(half), n[0] * s, n[1] * s, n[2] * s];
}

/**
 * Rotate a body-frame vector into the world frame:
 * v_world = v + 2w(qv × v) + 2 qv × (qv × v).
 * Requires a unit quaternion.
 */
export function quatRotateVector(q: Quat, v: Vec3): Vec3 {
  const qv: Vec3 = [q[1], q[2], q[3]];
  const t = vec3Scale(vec3Cross(qv, v), 2);
  return vec3Add(vec3Add(v, vec3Scale(t, q[0])), vec3Cross(qv, t));
}

/**
 * Rotation matrix R(q) such that v_world = R(q) v_body.
 * Requires a unit quaternion.
 */
export function quatToRotationMatrix(q: Quat): Mat3 {
  const [w, x, y, z] = q;
  return mat3(
    1 - 2 * (y * y + z * z),
    2 * (x * y - w * z),
    2 * (x * z + w * y),
    2 * (x * y + w * z),
    1 - 2 * (x * x + z * z),
    2 * (y * z - w * x),
    2 * (x * z - w * y),
    2 * (y * z + w * x),
    1 - 2 * (x * x + y * y),
  );
}

/** Quaternion time derivative: q_dot = 0.5 * q ⊗ (0, ω_body). */
export function quatDerivative(q: Quat, omegaBody: Vec3): Quat {
  const omegaQuat: Quat = [0, omegaBody[0], omegaBody[1], omegaBody[2]];
  const product = quatMultiply(q, omegaQuat);
  return [0.5 * product[0], 0.5 * product[1], 0.5 * product[2], 0.5 * product[3]];
}

/** Euler angles [roll (x), pitch (y), yaw (z)] in radians, Z-Y-X order. */
export function quatToEulerZYX(q: Quat): Vec3 {
  const r = quatToRotationMatrix(q);
  const sinPitch = Math.min(1, Math.max(-1, -r[6]));
  const pitch = Math.asin(sinPitch);
  const cosPitch = Math.cos(pitch);

  if (Math.abs(cosPitch) < 1e-6) {
    // Gimbal lock: only yaw + roll is determined, so roll is set to zero.
    return [0, pitch, Math.atan2(-r[1], r[4])];
  }
  return [
    Math.atan2(r[7], r[8]),
    pitch,
    Math.atan2(r[3], r[0]),
  ];
}
