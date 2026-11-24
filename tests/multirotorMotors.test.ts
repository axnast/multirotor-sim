import { describe, expect, test } from "vitest";

import {
  clampMotorCommand,
  motorReactionTorque,
  motorSpeedDerivative,
  motorThrust,
} from "../src/core/multirotor/motors";
import {
  makeHex,
  type MotorConfig,
} from "../src/core/multirotor/config";

const TOL = 1e-12;

function expectNumberClose(actual: number, expected: number, tol = TOL): void {
  expect(Math.abs(actual - expected)).toBeLessThan(tol);
}

function motorAt(index: number): MotorConfig {
  const motor = makeHex().motors[index];
  if (motor === undefined) {
    throw new Error(`no motor ${index}`);
  }
  return motor;
}

function expectVec3Close(
  actual: readonly [number, number, number],
  expected: readonly [number, number, number],
  tol = TOL,
): void {
  expectNumberClose(actual[0], expected[0], tol);
  expectNumberClose(actual[1], expected[1], tol);
  expectNumberClose(actual[2], expected[2], tol);
}

describe("motor thrust", () => {
  test("follows T = kT ω²", () => {
    const motor = motorAt(0);
    expectNumberClose(motorThrust(0, motor), 0);
    expectNumberClose(motorThrust(100, motor), motor.kT * 10_000);
    expectNumberClose(motorThrust(500, motor), motor.kT * 250_000);
  });

  test("is independent of spin direction", () => {
    const ccw = { ...motorAt(0), spin: 1 as const };
    const cw = { ...motorAt(1), spin: -1 as const };
    expectNumberClose(motorThrust(420, ccw), motorThrust(420, cw));
  });

  test("rejects non-finite speeds", () => {
    const motor = motorAt(0);
    expect(() => motorThrust(Number.NaN, motor)).toThrow(RangeError);
    expect(() => motorThrust(Number.POSITIVE_INFINITY, motor)).toThrow(RangeError);
  });
});

describe("motor reaction torque", () => {
  test("a CCW rotor (+1) pushes the airframe clockwise about -z", () => {
    const motor = { ...motorAt(0), spin: 1 as const, kQ: 1.5e-7 };
    const torque = motorReactionTorque(400, motor);
    expectVec3Close(torque, [0, 0, -1.5e-7 * 160_000]);
    expect(torque[2]).toBeLessThan(0);
  });

  test("a CW rotor (-1) pushes the airframe counter-clockwise about +z", () => {
    const motor = { ...motorAt(0), spin: -1 as const, kQ: 1.5e-7 };
    const torque = motorReactionTorque(400, motor);
    expectVec3Close(torque, [0, 0, 1.5e-7 * 160_000]);
    expect(torque[2]).toBeGreaterThan(0);
  });

  test("magnitude grows with ω² and the torque is odd in ω", () => {
    const motor = { ...motorAt(0), spin: 1 as const, kQ: 2e-7 };
    const quarter = motorReactionTorque(250, motor)[2];
    const half = motorReactionTorque(500, motor)[2];
    expectNumberClose(half, 4 * quarter);
    expectNumberClose(half, -motorReactionTorque(-500, motor)[2]);
    expectNumberClose(motorReactionTorque(0, motor)[2], 0);
  });

  test("is only about body z", () => {
    const torque = motorReactionTorque(600, motorAt(0));
    expect(torque[0]).toBe(0);
    expect(torque[1]).toBe(0);
  });

  test("rejects non-finite speeds", () => {
    const motor = motorAt(0);
    expect(() => motorReactionTorque(Number.NaN, motor)).toThrow(RangeError);
  });
});

describe("first-order motor lag", () => {
  test("dω/dt = (ω_cmd - ω) / τ", () => {
    const motor = { ...motorAt(0), motorTimeConstant: 0.05 };
    expectNumberClose(motorSpeedDerivative(0, 1000, motor), 20_000);
    expectNumberClose(motorSpeedDerivative(1000, 1000, motor), 0);
    expectNumberClose(motorSpeedDerivative(1000, 0, motor), -20_000);
    expectNumberClose(
      motorSpeedDerivative(250, 500, motor),
      (500 - 250) / 0.05,
    );
  });

  test("rate is inversely proportional to the time constant", () => {
    const fast = { ...motorAt(0), motorTimeConstant: 0.025 };
    const slow = { ...motorAt(0), motorTimeConstant: 0.1 };
    expectNumberClose(
      motorSpeedDerivative(0, 500, fast),
      4 * motorSpeedDerivative(0, 500, slow),
    );
  });

  test("rejects non-finite inputs", () => {
    const motor = motorAt(0);
    expect(() => motorSpeedDerivative(Number.NaN, 0, motor)).toThrow(RangeError);
    expect(() => motorSpeedDerivative(0, Number.NaN, motor)).toThrow(RangeError);
  });
});

describe("motor command clamping", () => {
  const motor = { ...motorAt(0), minOmega: 100, maxOmega: 900 };

  test("clamps to [minOmega, maxOmega] and passes through feasible values", () => {
    expectNumberClose(clampMotorCommand(-50, motor), 100);
    expectNumberClose(clampMotorCommand(0, motor), 100);
    expectNumberClose(clampMotorCommand(450, motor), 450);
    expectNumberClose(clampMotorCommand(900, motor), 900);
    expectNumberClose(clampMotorCommand(5000, motor), 900);
  });

  test("rejects non-finite commands", () => {
    expect(() => clampMotorCommand(Number.NaN, motor)).toThrow(RangeError);
    expect(() => clampMotorCommand(Number.POSITIVE_INFINITY, motor)).toThrow(
      RangeError,
    );
  });
});
