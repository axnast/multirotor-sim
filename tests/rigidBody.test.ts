import { describe, expect, test } from "vitest";

import {
  mat3FromDiagonal,
  mat3Inverse,
  mat3MulVec,
  type Mat3,
} from "../src/core/math/mat3";
import {
  QUAT_IDENTITY,
  quatDerivative,
  type Quat,
} from "../src/core/math/quat";
import {
  VEC3_ZERO,
  vec3Cross,
  vec3Length,
  vec3Scale,
  type Vec3,
} from "../src/core/math/vec3";
import {
  DEFAULT_GRAVITY,
  derivatives,
  ZERO_INPUTS,
  type RigidBodyParams,
  type RigidBodyState,
} from "../src/core/rigidBody";

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

function makeState(partial: Partial<RigidBodyState> = {}): RigidBodyState {
  return {
    position: partial.position ?? [0, 0, 0],
    velocity: partial.velocity ?? [0, 0, 0],
    orientation: partial.orientation ?? QUAT_IDENTITY,
    angularVelocity: partial.angularVelocity ?? [0, 0, 0],
  };
}

function makeParams(mass: number, inertia: Mat3): RigidBodyParams {
  return { mass, inertia };
}

describe("rigid body derivatives", () => {
  test("p_dot = v, v_dot = F/m + g, q_dot = 0.5 q ⊗ (0, ω)", () => {
    const state = makeState({
      position: [1, 2, 3],
      velocity: [4, 5, 6],
      angularVelocity: [0.1, 0.2, 0.3],
    });
    const params = makeParams(2, mat3FromDiagonal([1, 1, 1]));
    const d = derivatives(state, params, ZERO_INPUTS);

    expectVec3Close(d.position, [4, 5, 6]);
    expectVec3Close(d.velocity, DEFAULT_GRAVITY);
    expectQuatClose(
      d.orientation,
      quatDerivative(state.orientation, state.angularVelocity),
    );
    expectVec3Close(d.angularVelocity, VEC3_ZERO);
  });

  test("gyroscopic term: ω_dot = I⁻¹(-ω × (I ω)) for zero torque", () => {
    const inertia = mat3FromDiagonal([0.01, 0.02, 0.03]);
    const omega: Vec3 = [3, 2, 1];
    const d = derivatives(
      makeState({ angularVelocity: omega }),
      makeParams(1, inertia),
    );

    const iOmega = mat3MulVec(inertia, omega);
    const expected = mat3MulVec(
      mat3Inverse(inertia),
      vec3Scale(vec3Cross(omega, iOmega), -1),
    );
    expectVec3Close(d.angularVelocity, expected);
    expect(vec3Length(d.angularVelocity)).toBeGreaterThan(0.1);
  });

  test("constant torque from rest: ω_dot = I⁻¹ τ", () => {
    const inertia = mat3FromDiagonal([0.02, 0.02, 0.04]);
    const d = derivatives(makeState(), makeParams(2, inertia), {
      forceWorld: VEC3_ZERO,
      torqueBody: [0, 0, 0.4],
    });

    expectVec3Close(d.angularVelocity, [0, 0, 10]);
  });

  test("invalid mass and singular inertia are rejected", () => {
    const inertia = mat3FromDiagonal([1, 1, 1]);
    expect(() => derivatives(makeState(), makeParams(0, inertia))).toThrow(
      RangeError,
    );
    expect(() =>
      derivatives(
        makeState(),
        makeParams(1, mat3FromDiagonal([1, 0, 1])),
      ),
    ).toThrow(RangeError);
  });
});
