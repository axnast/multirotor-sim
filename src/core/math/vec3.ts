/**
 * 3D vector primitives.
 *
 * Conventions (see docs/conventions.md):
 * - Right-handed coordinates, Z-up world frame.
 * - `Vec3` is a readonly tuple [x, y, z]; all functions are pure and return
 *   new values. No DOM, Node, or three.js dependencies: this is portable to
 *   Rust/WASM.
 * - SI units are used by callers (m, m/s, kg, N, N·m, rad, rad/s).
 */

export type Vec3 = readonly [number, number, number];

export const VEC3_ZERO: Vec3 = [0, 0, 0];

export function vec3(x: number, y: number, z: number): Vec3 {
  return [x, y, z];
}

/** Component-wise sum: a + b. */
export function vec3Add(a: Vec3, b: Vec3): Vec3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

/** Component-wise difference: a - b. */
export function vec3Sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}

/** Scalar multiplication: s * a. */
export function vec3Scale(a: Vec3, s: number): Vec3 {
  return [a[0] * s, a[1] * s, a[2] * s];
}

/** Negation: -a. */
export function vec3Negate(a: Vec3): Vec3 {
  return [-a[0], -a[1], -a[2]];
}

/** Dot product: a · b = ax*bx + ay*by + az*bz. */
export function vec3Dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/** Right-hand-rule cross product: a × b. */
export function vec3Cross(a: Vec3, b: Vec3): Vec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

/** Squared Euclidean length: |a|² = a · a. */
export function vec3LengthSquared(a: Vec3): number {
  return vec3Dot(a, a);
}

/** Euclidean length: |a| = sqrt(a · a). */
export function vec3Length(a: Vec3): number {
  return Math.sqrt(vec3LengthSquared(a));
}

/** Unit vector along a: a / |a|. Throws for a zero-length input. */
export function vec3Normalize(a: Vec3): Vec3 {
  const length = vec3Length(a);
  if (length === 0) {
    throw new RangeError("Cannot normalize a zero-length vector");
  }
  return [a[0] / length, a[1] / length, a[2] / length];
}
