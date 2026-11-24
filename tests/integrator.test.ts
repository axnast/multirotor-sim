import { describe, expect, test } from "vitest";

import {
  angularMomentumWorld,
  rotationalKineticEnergy,
  totalEnergy,
} from "../src/core/energy";
import { eulerStep, step } from "../src/core/integrator";
import { mat3FromDiagonal } from "../src/core/math/mat3";
import {
  QUAT_IDENTITY,
  quatFromAxisAngle,
  type Quat,
} from "../src/core/math/quat";
import {
  VEC3_ZERO,
  vec3Length,
  vec3Sub,
  type Vec3,
} from "../src/core/math/vec3";
import {
  ZERO_INPUTS,
  type RigidBodyInputs,
  type RigidBodyParams,
  type RigidBodyState,
} from "../src/core/rigidBody";

type Integrator = (
  state: RigidBodyState,
  params: RigidBodyParams,
  inputs: RigidBodyInputs,
  dt: number,
) => RigidBodyState;

function makeState(partial: Partial<RigidBodyState> = {}): RigidBodyState {
  return {
    position: partial.position ?? [0, 0, 0],
    velocity: partial.velocity ?? [0, 0, 0],
    orientation: partial.orientation ?? QUAT_IDENTITY,
    angularVelocity: partial.angularVelocity ?? [0, 0, 0],
  };
}

function makeParams(
  mass: number,
  inertia: readonly [number, number, number],
  gravity: Vec3 = [0, 0, 0],
): RigidBodyParams {
  return { mass, inertia: mat3FromDiagonal(inertia), gravity };
}

function simulate(
  state: RigidBodyState,
  params: RigidBodyParams,
  inputs: RigidBodyInputs,
  dt: number,
  steps: number,
  integrator: Integrator = step,
): RigidBodyState {
  let current = state;
  for (let i = 0; i < steps; i++) {
    current = integrator(current, params, inputs, dt);
  }
  return current;
}

function relativeError(actual: number, expected: number): number {
  return Math.abs(actual - expected) / Math.abs(expected);
}

// Euclidean distance on the unit quaternion double cover. Unlike
// 2*acos(|a·b|), this stays well-conditioned for very small angle errors.
function quatDistance(a: Quat, b: Quat): number {
  let plus = 0;
  let minus = 0;
  for (let i = 0; i < 4; i++) {
    plus += (a[i]! - b[i]!) ** 2;
    minus += (a[i]! + b[i]!) ** 2;
  }
  return Math.sqrt(Math.min(plus, minus));
}

function stateError(a: RigidBodyState, b: RigidBodyState): number {
  const omegaError = vec3Length(vec3Sub(a.angularVelocity, b.angularVelocity));
  const orientationError = quatDistance(a.orientation, b.orientation);
  return Math.sqrt(omegaError * omegaError + orientationError * orientationError);
}

function observedOrder(dts: number[], errors: number[]): number {
  const xs = dts.map((dt) => Math.log2(dt));
  const ys = errors.map((error) => Math.log2(error));
  const meanX = xs.reduce((sum, x) => sum + x, 0) / xs.length;
  const meanY = ys.reduce((sum, y) => sum + y, 0) / ys.length;
  let numerator = 0;
  let denominator = 0;
  for (let i = 0; i < xs.length; i++) {
    numerator += (xs[i]! - meanX) * (ys[i]! - meanY);
    denominator += (xs[i]! - meanX) ** 2;
  }
  return numerator / denominator;
}

function expectBitIdenticalState(a: RigidBodyState, b: RigidBodyState): void {
  for (const key of ["position", "velocity", "angularVelocity"] as const) {
    expect(a[key][0]).toBe(b[key][0]);
    expect(a[key][1]).toBe(b[key][1]);
    expect(a[key][2]).toBe(b[key][2]);
  }
  expect(a.orientation[0]).toBe(b.orientation[0]);
  expect(a.orientation[1]).toBe(b.orientation[1]);
  expect(a.orientation[2]).toBe(b.orientation[2]);
  expect(a.orientation[3]).toBe(b.orientation[3]);
}

describe("integrators", () => {
  test("1. free fall from z=100 matches the analytic solution after 2 s", () => {
    const params: RigidBodyParams = {
      mass: 2,
      inertia: mat3FromDiagonal([1, 1, 1]),
    };
    const initial = makeState({ position: [0, 0, 100] });
    const dt = 1e-3;
    const t = 2;
    const final = simulate(initial, params, ZERO_INPUTS, dt, t / dt);

    expect(Math.abs(final.position[2] - (100 - 0.5 * 9.81 * t * t))).toBeLessThan(1e-9);
    expect(Math.abs(final.velocity[2] + 9.81 * t)).toBeLessThan(1e-9);
    expect(Math.abs(final.position[0])).toBeLessThan(1e-12);
    expect(Math.abs(final.position[1])).toBeLessThan(1e-12);
  });

  test("2. total energy is conserved in free fall over 10 s", () => {
    const params: RigidBodyParams = {
      mass: 2,
      inertia: mat3FromDiagonal([1, 1, 1]),
    };
    const initial = makeState({ position: [0, 0, 100] });
    const dt = 1e-3;
    const final = simulate(initial, params, ZERO_INPUTS, dt, 10 / dt);

    const initialEnergy = totalEnergy(initial, params);
    const finalEnergy = totalEnergy(final, params);
    expect(relativeError(finalEnergy, initialEnergy)).toBeLessThan(1e-10);
  });

  test("3. torque-free symmetric body: ω constant, orientation rotates by ωt", () => {
    const params = makeParams(1, [0.02, 0.02, 0.04]);
    const omega: Vec3 = [0, 0, 5];
    const initial = makeState({ angularVelocity: omega });
    const t = 4;
    const final = simulate(initial, params, ZERO_INPUTS, 1e-3, t / 1e-3);

    expect(vec3Length(vec3Sub(final.angularVelocity, omega))).toBeLessThan(1e-12);
    const expectedOrientation = quatFromAxisAngle([0, 0, 1], 5 * t);
    expect(quatDistance(final.orientation, expectedOrientation)).toBeLessThan(1e-9);
  });

  test("3b. torque-free symmetric top matches analytic precession (validates gyroscopic term)", () => {
    const params = makeParams(1, [0.02, 0.02, 0.04]);
    const omega0: Vec3 = [1, 0, 3];
    const initial = makeState({ angularVelocity: omega0 });
    const t = 4;
    const final = simulate(initial, params, ZERO_INPUTS, 5e-4, t / 5e-4);

    // I1 = I2 = A, I3 = C: Ω = ((C - A)/A) ω3 = 3 rad/s,
    // ω1(t) = cos(Ωt), ω2(t) = sin(Ωt), ω3(t) = 3.
    const analytic: Vec3 = [Math.cos(3 * t), Math.sin(3 * t), 3];
    expect(vec3Length(vec3Sub(final.angularVelocity, analytic))).toBeLessThan(1e-9);
  });

  test("4. asymmetric torque-free rotation conserves KE and |L| over 20 s", () => {
    const params = makeParams(1, [0.01, 0.02, 0.03]);
    const omega0: Vec3 = [3, 2, 1];
    const initial = makeState({ angularVelocity: omega0 });
    const dt = 1e-3;
    const steps = 20 / dt;

    const energy0 = rotationalKineticEnergy(initial, params);
    const momentum0 = vec3Length(angularMomentumWorld(initial, params));

    let current = initial;
    let maxEnergyDrift = 0;
    let maxMomentumDrift = 0;
    let maxOmegaChange = 0;
    for (let i = 0; i < steps; i++) {
      current = step(current, params, ZERO_INPUTS, dt);
      maxEnergyDrift = Math.max(
        maxEnergyDrift,
        relativeError(rotationalKineticEnergy(current, params), energy0),
      );
      maxMomentumDrift = Math.max(
        maxMomentumDrift,
        relativeError(vec3Length(angularMomentumWorld(current, params)), momentum0),
      );
      maxOmegaChange = Math.max(
        maxOmegaChange,
        vec3Length(vec3Sub(current.angularVelocity, omega0)),
      );
    }

    console.log(
      `[asymmetric] max KE drift=${maxEnergyDrift.toExponential(2)}, max |L| drift=${maxMomentumDrift.toExponential(2)}, max Δω=${maxOmegaChange.toFixed(3)} rad/s`,
    );
    expect(maxEnergyDrift).toBeLessThan(1e-8);
    expect(maxMomentumDrift).toBeLessThan(1e-8);
    expect(maxOmegaChange).toBeGreaterThan(0.1);
  });

  test("5. intermediate-axis tumble conserves KE and |L| over 20 s", () => {
    const params = makeParams(1, [1, 2, 3]);
    const omega0: Vec3 = [1e-3, 5, 0];
    const initial = makeState({ angularVelocity: omega0 });
    const dt = 1e-3;
    const steps = 20 / dt;

    const energy0 = rotationalKineticEnergy(initial, params);
    const momentum0 = vec3Length(angularMomentumWorld(initial, params));

    let current = initial;
    let maxEnergyDrift = 0;
    let maxMomentumDrift = 0;
    let maxAbsOmegaX = 0;
    for (let i = 0; i < steps; i++) {
      current = step(current, params, ZERO_INPUTS, dt);
      maxEnergyDrift = Math.max(
        maxEnergyDrift,
        relativeError(rotationalKineticEnergy(current, params), energy0),
      );
      maxMomentumDrift = Math.max(
        maxMomentumDrift,
        relativeError(vec3Length(angularMomentumWorld(current, params)), momentum0),
      );
      maxAbsOmegaX = Math.max(maxAbsOmegaX, Math.abs(current.angularVelocity[0]));
    }

    console.log(
      `[Dzhanibekov] max KE drift=${maxEnergyDrift.toExponential(2)}, max |L| drift=${maxMomentumDrift.toExponential(2)}, max |ω_x|=${maxAbsOmegaX.toFixed(3)} rad/s`,
    );
    expect(maxEnergyDrift).toBeLessThan(1e-8);
    expect(maxMomentumDrift).toBeLessThan(1e-8);
    expect(maxAbsOmegaX).toBeGreaterThan(0.1);
  });

  test("6. quaternion norm stays within 1e-12 after 10,000 steps", () => {
    const params = makeParams(1, [0.01, 0.02, 0.03]);
    const initial = makeState({ angularVelocity: [3, 2, 1] });
    const dt = 1e-3;

    let current = initial;
    let maxNormError = 0;
    for (let i = 0; i < 10_000; i++) {
      current = step(current, params, ZERO_INPUTS, dt);
      const q = current.orientation;
      const norm = Math.sqrt(q[0] ** 2 + q[1] ** 2 + q[2] ** 2 + q[3] ** 2);
      maxNormError = Math.max(maxNormError, Math.abs(norm - 1));
    }

    expect(maxNormError).toBeLessThan(1e-12);
  });

  test("7. convergence: RK4 order ~4, Euler order ~1 vs a fine-dt reference", () => {
    const params = makeParams(1, [0.01, 0.02, 0.03]);
    const initial = makeState({ angularVelocity: [3, 2, 1] });
    const t = 2;
    const referenceDt = 1 / 12_800;
    const reference = simulate(
      initial,
      params,
      ZERO_INPUTS,
      referenceDt,
      Math.round(t / referenceDt),
    );

    const dts = [1 / 50, 1 / 100, 1 / 200];
    const rk4Errors = dts.map((dt) =>
      stateError(simulate(initial, params, ZERO_INPUTS, dt, Math.round(t / dt)), reference),
    );
    const eulerErrors = dts.map((dt) =>
      stateError(
        simulate(initial, params, ZERO_INPUTS, dt, Math.round(t / dt), eulerStep),
        reference,
      ),
    );
    const rk4Order = observedOrder(dts, rk4Errors);
    const eulerOrder = observedOrder(dts, eulerErrors);

    console.log(
      `[convergence] RK4 errors=[${rk4Errors.map((e) => e.toExponential(2)).join(", ")}] order=${rk4Order.toFixed(2)}; Euler errors=[${eulerErrors.map((e) => e.toExponential(2)).join(", ")}] order=${eulerOrder.toFixed(2)}`,
    );
    expect(rk4Errors.every((error) => error > 1e-12)).toBe(true);
    expect(Math.abs(rk4Order - 4)).toBeLessThan(0.3);
    expect(Math.abs(eulerOrder - 1)).toBeLessThan(0.15);
  });

  test("8. constant torque about one axis matches analytic angular acceleration", () => {
    const params = makeParams(1, [0.02, 0.02, 0.04]);
    const initial = makeState();
    const inputs: RigidBodyInputs = {
      forceWorld: VEC3_ZERO,
      torqueBody: [0, 0, 0.4],
    };
    const t = 2;
    const final = simulate(initial, params, inputs, 1e-3, t / 1e-3);

    const alpha = 0.4 / 0.04;
    expect(Math.abs(final.angularVelocity[2] - alpha * t)).toBeLessThan(1e-9);
    expect(Math.abs(final.angularVelocity[0])).toBeLessThan(1e-12);
    expect(Math.abs(final.angularVelocity[1])).toBeLessThan(1e-12);
    const expectedOrientation = quatFromAxisAngle([0, 0, 1], 0.5 * alpha * t * t);
    expect(quatDistance(final.orientation, expectedOrientation)).toBeLessThan(1e-9);
  });

  test("9. identical inputs produce bit-identical outputs", () => {
    const params = makeParams(1, [0.01, 0.02, 0.03]);
    const makeInitial = (): RigidBodyState =>
      makeState({ angularVelocity: [3, 2, 1] });

    const a = simulate(makeInitial(), params, ZERO_INPUTS, 1e-3, 10_000);
    const b = simulate(makeInitial(), params, ZERO_INPUTS, 1e-3, 10_000);

    expectBitIdenticalState(a, b);
  });

  test("invalid dt is rejected", () => {
    const params = makeParams(1, [1, 1, 1]);
    expect(() => step(makeState(), params, ZERO_INPUTS, 0)).toThrow(RangeError);
    expect(() => eulerStep(makeState(), params, ZERO_INPUTS, -1)).toThrow(RangeError);
  });
});
