import { describe, expect, test } from "vitest";

import { QUAT_IDENTITY } from "../src/core/math/quat";
import {
  hoverMotorSpeeds,
  makeHex,
  makeOcto,
  makeQuad,
  maxThrust,
  type MultirotorConfig,
} from "../src/core/multirotor/config";
import {
  motorWrench,
  type MultirotorState,
} from "../src/core/multirotor/dynamics";
import {
  allocationMatrix,
  mix,
  mixDetailed,
  mixerMatrix,
  type Wrench,
} from "../src/core/multirotor/mixer";

const TOL = 1e-12;

function close(actual: number, expected: number, tol = TOL): void {
  expect(Math.abs(actual - expected)).toBeLessThan(tol);
}

function vecClose(
  actual: readonly number[],
  expected: readonly number[],
  tol = TOL,
): void {
  for (let index = 0; index < expected.length; index++) {
    close(at(actual, index), at(expected, index), tol);
  }
}

function at<T>(values: readonly T[], index: number): T {
  const value = values[index];
  if (value === undefined) {
    throw new Error(`missing value at index ${index}`);
  }
  return value;
}

function cell(matrix: readonly (readonly number[])[], row: number, col: number): number {
  const values = matrix[row];
  if (values === undefined) {
    throw new Error(`missing matrix row ${row}`);
  }
  return at(values, col);
}

function multiply(a: number[][], b: number[][]): number[][] {
  const rows = a.length;
  const cols = b[0]?.length ?? 0;
  const inner = b.length;
  const result: number[][] = [];
  for (let row = 0; row < rows; row++) {
    const values: number[] = [];
    for (let col = 0; col < cols; col++) {
      let sum = 0;
      for (let k = 0; k < inner; k++) {
        sum += cell(a, row, k) * cell(b, k, col);
      }
      values.push(sum);
    }
    result.push(values);
  }
  return result;
}

function hoverWrench(config: MultirotorConfig): Wrench {
  return [config.mass * 9.81, 0, 0, 0];
}

function torqueWrench(config: MultirotorConfig, axis: number, value: number): Wrench {
  const components = [config.mass * 9.81, 0, 0, 0];
  components[axis] = value;
  return [at(components, 0), at(components, 1), at(components, 2), at(components, 3)];
}

function stateWithSpeeds(speeds: readonly number[]): MultirotorState {
  return {
    body: {
      position: [0, 0, 0],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    },
    motorSpeeds: speeds,
  };
}

function meanSpeed(config: MultirotorConfig, speeds: readonly number[], spin: 1 | -1): number {
  const selected = config.motors
    .map((motor, index) => (motor.spin === spin ? at(speeds, index) : null))
    .filter((value): value is number => value !== null);
  return selected.reduce((sum, value) => sum + value, 0) / selected.length;
}

describe("allocation matrix", () => {
  test("has the documented four rows for quad, hex and octo", () => {
    for (const config of [makeQuad(), makeHex(), makeOcto()]) {
      const a = allocationMatrix(config);
      expect(a).toHaveLength(4);
      for (const row of a) {
        expect(row).toHaveLength(config.motors.length);
      }

      for (const [index, motor] of config.motors.entries()) {
        close(cell(a, 0, index), 1);
        close(cell(a, 1, index), motor.position[1]);
        close(cell(a, 2, index), -motor.position[0]);
        close(cell(a, 3, index), -motor.spin * (motor.kQ / motor.kT));
      }
    }
  });

  test("matches the physics module's motor wrench", () => {
    const config = makeHex();
    const speeds = config.motors.map((_, index) => 300 + index * 10);
    const a = allocationMatrix(config);
    const thrusts = config.motors.map((motor, index) => {
      const omega = at(speeds, index);
      return motor.kT * omega * omega;
    });
    const expected = [0, 0, 0, 0].map((_, row) =>
      thrusts.reduce((sum, thrust, index) => sum + cell(a, row, index) * thrust, 0),
    );
    const wrench = motorWrench(stateWithSpeeds(speeds), config);

    close(at(expected, 0), wrench.forceBody[2]);
    close(at(expected, 1), wrench.torqueBody[0]);
    close(at(expected, 2), wrench.torqueBody[1]);
    close(at(expected, 3), wrench.torqueBody[2]);
  });

  test("rejects an invalid config", () => {
    expect(() => allocationMatrix({ ...makeHex(), mass: 0 })).toThrow(RangeError);
  });
});

describe("mixer matrix", () => {
  test("A · M = I₄ for quad, hex and octo", () => {
    for (const config of [makeQuad(), makeHex(), makeOcto()]) {
      const identity = multiply(allocationMatrix(config), mixerMatrix(config));
      expect(identity).toHaveLength(4);
      for (let row = 0; row < 4; row++) {
        for (let col = 0; col < 4; col++) {
          close(cell(identity, row, col), row === col ? 1 : 0, 1e-12);
        }
      }
    }
  });

  test("A · M · A = A (Moore–Penrose condition)", () => {
    for (const config of [makeHex(), makeOcto()]) {
      const a = allocationMatrix(config);
      const product = multiply(multiply(a, mixerMatrix(config)), a);
      expect(product).toHaveLength(4);
      for (let row = 0; row < 4; row++) {
        vecClose(at(product, row), at(a, row), 1e-12);
      }
    }
  });

  test("throws on singular geometry", () => {
    const base = makeHex();
    const collinear: MultirotorConfig = {
      ...base,
      motors: base.motors.map((motor, index) => ({
        ...motor,
        position: [0.25 - index * 0.05, 0, 0],
      })),
    };
    expect(() => mixerMatrix(collinear)).toThrow(RangeError);
    expect(() => mix(collinear, hoverWrench(collinear))).toThrow(RangeError);
  });
});

describe("feasible mixing", () => {
  test("hover maps back to the hover motor speeds", () => {
    const config = makeHex();
    const result = mixDetailed(config, hoverWrench(config));
    const speeds = hoverMotorSpeeds(config);
    vecClose(result.motorSpeeds, speeds, 1e-9);
    vecClose(result.achieved, hoverWrench(config), 1e-9);
    expect(result.saturated).toBe(false);
  });

  test("round trips an off-hover wrench for quad, hex and octo", () => {
    const wrench: Wrench = [14.715, 0.05, -0.03, 0.01];
    for (const config of [makeQuad(), makeHex(), makeOcto()]) {
      const result = mixDetailed(config, [config.mass * 9.81, 0.05, -0.03, 0.01]);
      expect(result.saturated).toBe(false);
      vecClose(result.achieved, wrench, 1e-9);
      for (const speed of result.motorSpeeds) {
        expect(speed).toBeGreaterThanOrEqual(0);
        expect(speed).toBeLessThanOrEqual(1000);
      }
    }
  });

  test("achieves pure +τx, +τy and +τz with the right signs", () => {
    const config = makeHex();
    for (const axis of [1, 2, 3]) {
      const value = axis === 3 ? 0.01 : 0.05;
      const result = mixDetailed(config, torqueWrench(config, axis, value));
      vecClose(result.achieved, torqueWrench(config, axis, value), 1e-9);
      expect(at(result.achieved, axis)).toBeGreaterThan(0);
    }
  });

  test("positive yaw speeds up clockwise rotors", () => {
    const config = makeHex();
    const result = mixDetailed(config, torqueWrench(config, 3, 0.01));
    const clockwise = meanSpeed(config, result.motorSpeeds, -1);
    const counterClockwise = meanSpeed(config, result.motorSpeeds, 1);
    expect(clockwise).toBeGreaterThan(counterClockwise);
    expect(at(result.achieved, 3)).toBeGreaterThan(0);
  });

  test("handles unequal motor constants", () => {
    const base = makeHex();
    const config: MultirotorConfig = {
      ...base,
      motors: base.motors.map((motor, index) =>
        index === 0
          ? { ...motor, kT: 2e-5, kQ: 3e-7, maxOmega: 800 }
          : motor,
      ),
    };
    const wrench = torqueWrench(config, 1, 0.02);
    const result = mixDetailed(config, wrench);
    vecClose(result.achieved, wrench, 1e-9);
    expect(result.saturated).toBe(false);
    for (const speed of result.motorSpeeds) {
      expect(speed).toBeGreaterThanOrEqual(0);
      expect(speed).toBeLessThanOrEqual(1000);
    }
  });

  test("reported thrusts and wrench are consistent with the speeds", () => {
    const config = makeHex();
    const wrench = torqueWrench(config, 1, 0.04);
    const detailed = mixDetailed(config, wrench);
    const recomputed = config.motors.map((motor, index) => {
      const omega = at(detailed.motorSpeeds, index);
      return motor.kT * omega * omega;
    });
    vecClose(detailed.motorThrusts, recomputed, 1e-12);
    vecClose(detailed.achieved, wrench, 1e-9);
    expect(mix(config, wrench)).toEqual(detailed.motorSpeeds);
  });
});

describe("saturation", () => {
  test("clamps an excessive thrust request before solving", () => {
    const config = makeHex();
    const result = mixDetailed(config, [10 * maxThrust(config), 0, 0, 0]);
    close(at(result.achieved, 0), maxThrust(config), 1e-9);
    for (const speed of result.motorSpeeds) {
      close(speed, 1000, 1e-9);
    }
    expect(result.saturated).toBe(true);
  });

  test("clamps a negative thrust request", () => {
    const config = makeHex();
    const result = mixDetailed(config, [-10, 0, 0, 0]);
    close(at(result.achieved, 0), 0, 1e-9);
    for (const speed of result.motorSpeeds) {
      close(speed, 0, 1e-9);
    }
    expect(result.saturated).toBe(true);
  });

  test("respects a positive minimum rotor speed", () => {
    const config = makeHex({ minOmega: 200 });
    const result = mixDetailed(config, [0, 0, 0, 0]);
    const expected = config.motors.reduce(
      (sum, motor) => sum + motor.kT * 200 * 200,
      0,
    );
    close(at(result.achieved, 0), expected, 1e-9);
    for (const speed of result.motorSpeeds) {
      close(speed, 200, 1e-9);
    }
    expect(result.saturated).toBe(true);
  });

  test("retains roll authority when the request is infeasible", () => {
    const config = makeHex();
    const result = mixDetailed(config, torqueWrench(config, 1, 100));
    expect(at(result.achieved, 1)).toBeGreaterThan(0);
    expect(at(result.achieved, 1)).toBeLessThan(100);
    close(at(result.achieved, 0), config.mass * 9.81, 1e-9);
    close(at(result.achieved, 2), 0, 1e-9);
    close(at(result.achieved, 3), 0, 1e-9);
    expect(result.saturated).toBe(true);
    for (const speed of result.motorSpeeds) {
      expect(speed).toBeGreaterThanOrEqual(0);
      expect(speed).toBeLessThanOrEqual(1000);
    }
  });

  test("retains yaw authority when the request is infeasible", () => {
    const config = makeHex();
    const result = mixDetailed(config, torqueWrench(config, 3, 100));
    expect(at(result.achieved, 3)).toBeGreaterThan(0);
    close(at(result.achieved, 0), config.mass * 9.81, 1e-9);
    expect(result.saturated).toBe(true);
  });

  test("sheds yaw before roll and pitch", () => {
    const config = makeHex();
    const wrench: Wrench = [config.mass * 9.81, 100, 0, 100];
    const result = mixDetailed(config, wrench);
    close(at(result.achieved, 3), 0, 1e-9);
    expect(at(result.achieved, 1)).toBeGreaterThan(0);
    close(at(result.achieved, 0), config.mass * 9.81, 1e-9);
    expect(result.saturated).toBe(true);
  });

  test("preserves the roll/pitch direction while scaling both", () => {
    const config = makeHex();
    const result = mixDetailed(config, [config.mass * 9.81, 100, 50, 0]);
    close(at(result.achieved, 1) / at(result.achieved, 2), 2, 1e-9);
    expect(result.saturated).toBe(true);
  });

  test("is conservative with unequal motor limits (documented limitation)", () => {
    const base = makeHex();
    const config: MultirotorConfig = {
      ...base,
      motors: base.motors.map((motor, index) =>
        index === 1 ? { ...motor, maxOmega: 800 } : motor,
      ),
    };
    const result = mixDetailed(config, [50, 0, 0, 0]);
    expect(at(result.achieved, 0)).toBeLessThan(50);
    for (const speed of result.motorSpeeds) {
      expect(speed).toBeLessThanOrEqual(1000);
    }
    expect(at(result.motorSpeeds, 1)).toBeLessThanOrEqual(800);
    expect(result.saturated).toBe(true);
  });

  test("falls back safely for pathological heterogeneous bounds", () => {
    const base = makeHex();
    const config: MultirotorConfig = {
      ...base,
      motors: base.motors.map((motor, index) => {
        if (index === 0) return { ...motor, minOmega: 900, maxOmega: 1000 };
        if (index === 1) return { ...motor, minOmega: 0, maxOmega: 800 };
        return motor;
      }),
    };
    const result = mixDetailed(config, hoverWrench(config));
    for (const [index, motor] of config.motors.entries()) {
      const speed = at(result.motorSpeeds, index);
      expect(Number.isFinite(speed)).toBe(true);
      expect(speed).toBeGreaterThanOrEqual(motor.minOmega);
      expect(speed).toBeLessThanOrEqual(motor.maxOmega);
    }
    const a = allocationMatrix(config);
    const expected = [0, 0, 0, 0].map((_, row) =>
      result.motorThrusts.reduce(
        (sum, thrust, index) => sum + cell(a, row, index) * thrust,
        0,
      ),
    );
    vecClose(result.achieved, expected, 1e-12);
    expect(result.saturated).toBe(true);
  });
});

describe("robustness", () => {
  const config = makeHex();

  test("rejects malformed wrenches", () => {
    expect(() => mix(config, [14.715, 0, 0, Number.NaN])).toThrow(RangeError);
    expect(() => mix(config, [14.715, 0, 0, Number.POSITIVE_INFINITY])).toThrow(
      RangeError,
    );
    expect(() =>
      mix(config, [14.715, 0, 0] as unknown as Wrench),
    ).toThrow(RangeError);
  });

  test("is deterministic and does not mutate its inputs", () => {
    const wrench = torqueWrench(config, 1, 0.04);
    const configBefore = JSON.stringify(config);
    const wrenchBefore = JSON.stringify(wrench);

    expect(mix(config, wrench)).toEqual(mix(config, wrench));
    expect(mixDetailed(config, wrench)).toEqual(mixDetailed(config, wrench));
    expect(JSON.stringify(config)).toBe(configBefore);
    expect(JSON.stringify(wrench)).toBe(wrenchBefore);
  });
});
