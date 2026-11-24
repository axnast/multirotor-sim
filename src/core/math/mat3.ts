/**
 * 3x3 matrix primitives, used for the body-frame inertia tensor.
 *
 * Conventions (see docs/conventions.md):
 * - Row-major storage: [m00, m01, m02, m10, m11, m12, m20, m21, m22].
 * - `Mat3` is a readonly tuple; functions are pure and return new values.
 * - `mat3MulVec(m, v)` computes m * v with v as a column vector.
 * - `mat3Inverse` throws on (numerically) singular matrices rather than
 *   returning NaNs.
 */

import type { Vec3 } from "./vec3";

export type Mat3 = readonly [
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
];

export const MAT3_IDENTITY: Mat3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];

export function mat3(
  m00: number,
  m01: number,
  m02: number,
  m10: number,
  m11: number,
  m12: number,
  m20: number,
  m21: number,
  m22: number,
): Mat3 {
  return [m00, m01, m02, m10, m11, m12, m20, m21, m22];
}

/** Diagonal matrix diag(d[0], d[1], d[2]). */
export function mat3FromDiagonal(d: Vec3): Mat3 {
  return [d[0], 0, 0, 0, d[1], 0, 0, 0, d[2]];
}

/** Matrix-vector product: m * v, with v as a column vector. */
export function mat3MulVec(m: Mat3, v: Vec3): Vec3 {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2],
    m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
    m[6] * v[0] + m[7] * v[1] + m[8] * v[2],
  ];
}

/** Determinant via cofactor expansion along the first row. */
export function mat3Determinant(m: Mat3): number {
  return (
    m[0] * (m[4] * m[8] - m[5] * m[7]) -
    m[1] * (m[3] * m[8] - m[5] * m[6]) +
    m[2] * (m[3] * m[7] - m[4] * m[6])
  );
}

/**
 * Inverse via the adjugate divided by the determinant:
 * m⁻¹ = adj(m) / det(m).
 */
export function mat3Inverse(m: Mat3): Mat3 {
  const c00 = m[4] * m[8] - m[5] * m[7];
  const c01 = -(m[3] * m[8] - m[5] * m[6]);
  const c02 = m[3] * m[7] - m[4] * m[6];
  const c10 = -(m[1] * m[8] - m[2] * m[7]);
  const c11 = m[0] * m[8] - m[2] * m[6];
  const c12 = -(m[0] * m[7] - m[1] * m[6]);
  const c20 = m[1] * m[5] - m[2] * m[4];
  const c21 = -(m[0] * m[5] - m[2] * m[3]);
  const c22 = m[0] * m[4] - m[1] * m[3];

  const det = m[0] * c00 + m[1] * c01 + m[2] * c02;
  const scale = Math.max(
    Math.abs(m[0]),
    Math.abs(m[1]),
    Math.abs(m[2]),
    Math.abs(m[3]),
    Math.abs(m[4]),
    Math.abs(m[5]),
    Math.abs(m[6]),
    Math.abs(m[7]),
    Math.abs(m[8]),
  );
  if (Math.abs(det) <= Number.EPSILON * scale * scale * scale) {
    throw new RangeError("Matrix is singular and cannot be inverted");
  }

  // adj(m) is the transpose of the cofactor matrix.
  return [
    c00 / det,
    c10 / det,
    c20 / det,
    c01 / det,
    c11 / det,
    c21 / det,
    c02 / det,
    c12 / det,
    c22 / det,
  ];
}
