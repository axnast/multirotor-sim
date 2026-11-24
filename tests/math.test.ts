import { describe, expect, test } from "vitest";

import {
  mat3,
  mat3Determinant,
  mat3FromDiagonal,
  mat3Inverse,
  mat3MulVec,
  MAT3_IDENTITY,
  type Mat3,
} from "../src/core/math/mat3";
import {
  QUAT_IDENTITY,
  quatConjugate,
  quatDerivative,
  quatFromAxisAngle,
  quatMultiply,
  quatNormalize,
  quatRotateVector,
  quatToEulerZYX,
  quatToRotationMatrix,
  type Quat,
} from "../src/core/math/quat";
import {
  VEC3_ZERO,
  vec3,
  vec3Add,
  vec3Cross,
  vec3Dot,
  vec3Length,
  vec3LengthSquared,
  vec3Negate,
  vec3Normalize,
  vec3Scale,
  vec3Sub,
  type Vec3,
} from "../src/core/math/vec3";

const TOL = 1e-12;

function expectNumberClose(actual: number, expected: number, tol = TOL): void {
  expect(Math.abs(actual - expected)).toBeLessThan(tol);
}

function expectVec3Close(actual: Vec3, expected: Vec3, tol = TOL): void {
  expectNumberClose(actual[0], expected[0], tol);
  expectNumberClose(actual[1], expected[1], tol);
  expectNumberClose(actual[2], expected[2], tol);
}

function expectQuatClose(actual: Quat, expected: Quat, tol = TOL): void {
  expectNumberClose(actual[0], expected[0], tol);
  expectNumberClose(actual[1], expected[1], tol);
  expectNumberClose(actual[2], expected[2], tol);
  expectNumberClose(actual[3], expected[3], tol);
}

function expectMat3Close(actual: Mat3, expected: Mat3, tol = TOL): void {
  for (let i = 0; i < 9; i++) {
    expectNumberClose(actual[i]!, expected[i]!, tol);
  }
}

describe("vec3", () => {
  test("add, subtract, scale, and negate are component-wise", () => {
    const a = vec3(1, 2, 3);
    const b = vec3(4, -5, 6);

    expectVec3Close(vec3Add(a, b), vec3(5, -3, 9));
    expectVec3Close(vec3Sub(a, b), vec3(-3, 7, -3));
    expectVec3Close(vec3Scale(a, 2.5), vec3(2.5, 5, 7.5));
    expectVec3Close(vec3Negate(a), vec3(-1, -2, -3));
  });

  test("dot product", () => {
    expectNumberClose(vec3Dot(vec3(1, 2, 3), vec3(4, -5, 6)), 12);
    expectNumberClose(vec3Dot(VEC3_ZERO, vec3(1, 2, 3)), 0);
  });

  test("cross product follows the right-hand rule", () => {
    expectVec3Close(vec3Cross(vec3(1, 0, 0), vec3(0, 1, 0)), vec3(0, 0, 1));
    expectVec3Close(vec3Cross(vec3(0, 1, 0), vec3(1, 0, 0)), vec3(0, 0, -1));
    expectVec3Close(vec3Cross(vec3(1, 0, 0), vec3(1, 0, 0)), VEC3_ZERO);
  });

  test("length and normalize", () => {
    const a = vec3(3, 4, 0);
    expectNumberClose(vec3LengthSquared(a), 25);
    expectNumberClose(vec3Length(a), 5);
    expectVec3Close(vec3Normalize(a), vec3(0.6, 0.8, 0));
    expectNumberClose(vec3Length(vec3Normalize(a)), 1);
    expect(() => vec3Normalize(VEC3_ZERO)).toThrow(RangeError);
  });
});

describe("mat3", () => {
  test("diagonal matrices multiply vectors component-wise", () => {
    const m = mat3FromDiagonal(vec3(2, 3, 4));
    expectVec3Close(mat3MulVec(m, vec3(1, 1, 1)), vec3(2, 3, 4));
    expectVec3Close(mat3MulVec(MAT3_IDENTITY, vec3(-1, 0.5, 7)), vec3(-1, 0.5, 7));
  });

  test("determinant and inverse", () => {
    expectNumberClose(mat3Determinant(MAT3_IDENTITY), 1);
    expectNumberClose(mat3Determinant(mat3(1, 2, 3, 4, 5, 6, 7, 8, 10)), -3);
    expectMat3Close(
      mat3Inverse(mat3FromDiagonal(vec3(2, 3, 4))),
      mat3FromDiagonal(vec3(0.5, 1 / 3, 0.25)),
    );

    const m = mat3(1, 2, 3, 0, 1, 4, 5, 6, 0);
    const inv = mat3Inverse(m);
    const v = vec3(0.3, -1.2, 2.5);
    expectVec3Close(mat3MulVec(inv, mat3MulVec(m, v)), v);
    expectVec3Close(mat3MulVec(m, mat3MulVec(inv, v)), v);
  });

  test("singular matrices are rejected", () => {
    expect(() => mat3Inverse(mat3(1, 2, 3, 2, 4, 6, 0, 1, 1))).toThrow(
      RangeError,
    );
    expect(() => mat3Inverse(mat3(0, 0, 0, 0, 0, 0, 0, 0, 0))).toThrow(
      RangeError,
    );
  });
});

describe("quat", () => {
  test("identity and rotations about coordinate axes", () => {
    expectVec3Close(quatRotateVector(QUAT_IDENTITY, vec3(1, 2, 3)), vec3(1, 2, 3));

    const qz90 = quatFromAxisAngle(vec3(0, 0, 1), Math.PI / 2);
    expectVec3Close(quatRotateVector(qz90, vec3(1, 0, 0)), vec3(0, 1, 0));
  });

  test("conjugate undoes a rotation", () => {
    const q = quatFromAxisAngle(vec3Normalize(vec3(1, 2, 3)), 0.7);
    const v = vec3(0.4, -0.2, 1.3);
    const rotated = quatRotateVector(q, v);
    expectVec3Close(quatRotateVector(quatConjugate(q), rotated), v);
  });

  test("multiplication composes rotations: first b, then a", () => {
    const qz = quatFromAxisAngle(vec3(0, 0, 1), Math.PI / 2);
    const qx = quatFromAxisAngle(vec3(1, 0, 0), 0.6);
    const v = vec3(0.3, -0.7, 0.2);

    const composed = quatMultiply(qz, qx);
    const sequential = quatRotateVector(qz, quatRotateVector(qx, v));
    expectVec3Close(quatRotateVector(composed, v), sequential);
  });

  test("normalize preserves the rotation", () => {
    const q = quatFromAxisAngle(vec3(1, -2, 0.5), 1.1);
    const scaled: Quat = [3.7 * q[0], 3.7 * q[1], 3.7 * q[2], 3.7 * q[3]];
    expectQuatClose(quatNormalize(scaled), q);
    expect(() => quatNormalize([0, 0, 0, 0])).toThrow(RangeError);
  });

  test("fromAxisAngle rejects a zero axis for a nonzero angle", () => {
    expect(() => quatFromAxisAngle(VEC3_ZERO, 1)).toThrow(RangeError);
    expectQuatClose(quatFromAxisAngle(VEC3_ZERO, 0), QUAT_IDENTITY);
  });

  test("rotation matrix is orthonormal and matches quatRotateVector", () => {
    const q = quatFromAxisAngle(vec3Normalize(vec3(0.2, -0.5, 1)), 1.3);
    const r = quatToRotationMatrix(q);
    const v = vec3(0.7, 0.1, -0.4);

    expectVec3Close(mat3MulVec(r, v), quatRotateVector(q, v));

    const column = (index: number): Vec3 =>
      vec3(r[index]!, r[3 + index]!, r[6 + index]!);
    for (let i = 0; i < 3; i++) {
      expectNumberClose(vec3Length(column(i)), 1);
    }
    expectNumberClose(vec3Dot(column(0), column(1)), 0);
    expectNumberClose(vec3Dot(column(0), column(2)), 0);
    expectNumberClose(vec3Dot(column(1), column(2)), 0);
  });

  test("quatDerivative matches q_dot = 0.5 * q ⊗ (0, ω)", () => {
    const q = quatFromAxisAngle(vec3Normalize(vec3(0.3, -0.1, 1)), 0.4);
    const omega = vec3(1, 2, 3);
    const h = 1e-5;

    // Constant body-rate solution: q(t ± h) = q(t) ⊗ exp(±0.5 ω h).
    // A central difference is O(h²); a forward difference is only O(h) and
    // its truncation error would dominate this check.
    const omegaNorm = vec3Length(omega);
    const qNext = quatMultiply(q, quatFromAxisAngle(omega, omegaNorm * h));
    const qPrev = quatMultiply(q, quatFromAxisAngle(omega, -omegaNorm * h));
    const numeric: Quat = [
      (qNext[0] - qPrev[0]) / (2 * h),
      (qNext[1] - qPrev[1]) / (2 * h),
      (qNext[2] - qPrev[2]) / (2 * h),
      (qNext[3] - qPrev[3]) / (2 * h),
    ];

    expectQuatClose(quatDerivative(q, omega), numeric, 1e-9);
  });

  test("Euler Z-Y-X extraction round-trips", () => {
    const roll = 0.2;
    const pitch = -0.3;
    const yaw = 1.1;
    const q = quatMultiply(
      quatMultiply(
        quatFromAxisAngle(vec3(0, 0, 1), yaw),
        quatFromAxisAngle(vec3(0, 1, 0), pitch),
      ),
      quatFromAxisAngle(vec3(1, 0, 0), roll),
    );

    expectVec3Close(quatToEulerZYX(q), vec3(roll, pitch, yaw));
  });

  test("Euler extraction handles gimbal lock", () => {
    const yaw = 0.4;
    const q = quatMultiply(
      quatFromAxisAngle(vec3(0, 0, 1), yaw),
      quatFromAxisAngle(vec3(0, 1, 0), Math.PI / 2),
    );
    const euler = quatToEulerZYX(q);

    expectNumberClose(euler[0], 0);
    expectNumberClose(euler[1], Math.PI / 2);
    expectNumberClose(euler[2], yaw);
  });
});
