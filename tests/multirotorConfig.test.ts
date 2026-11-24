import { describe, expect, test } from "vitest";

import { mat3FromDiagonal } from "../src/core/math/mat3";
import type { Vec3 } from "../src/core/math/vec3";
import {
  DEFAULT_ARM_LENGTH,
  DEFAULT_GROUND_PLANE,
  DEFAULT_KQ,
  DEFAULT_KT,
  DEFAULT_LINEAR_DRAG,
  DEFAULT_MASS,
  DEFAULT_MAX_OMEGA,
  DEFAULT_MIN_OMEGA,
  DEFAULT_MOTOR_TIME_CONSTANT,
  DEFAULT_QUADRATIC_DRAG,
  HEX_START_ANGLE,
  OCTO_START_ANGLE,
  QUAD_START_ANGLE,
  hoverMotorSpeeds,
  makeHex,
  makeOcto,
  makeQuad,
  maxThrust,
  minThrust,
  ringInertia,
  rigidBodyParamsOf,
  thrustToWeight,
  validateConfig,
  type MotorConfig,
  type MultirotorConfig,
  type SpinDirection,
} from "../src/core/multirotor/config";

const TOL = 1e-12;

function expectNumberClose(actual: number, expected: number, tol = TOL): void {
  expect(Math.abs(actual - expected)).toBeLessThan(tol);
}

function expectVec3Close(actual: Vec3, expected: Vec3, tol = TOL): void {
  expectNumberClose(actual[0], expected[0], tol);
  expectNumberClose(actual[1], expected[1], tol);
  expectNumberClose(actual[2], expected[2], tol);
}

function motorAt(config: MultirotorConfig, index: number): MotorConfig {
  const motor = config.motors[index];
  if (motor === undefined) {
    throw new Error(`config has no motor ${index}`);
  }
  return motor;
}

/** Rotor angles measured counter-clockwise from body +x, normalised to [0, 2π). */
function rotorAngles(config: MultirotorConfig): number[] {
  return config.motors
    .map((motor) => {
      const angle = Math.atan2(motor.position[1], motor.position[0]);
      return angle < 0 ? angle + 2 * Math.PI : angle;
    })
    .sort((a, b) => a - b);
}

function withMotor(
  config: MultirotorConfig,
  overrides: Partial<MotorConfig>,
): MultirotorConfig {
  return {
    ...config,
    motors: config.motors.map((motor, index) =>
      index === 0 ? { ...motor, ...overrides } : motor,
    ),
  };
}

function expectEvenLayout(
  config: MultirotorConfig,
  count: number,
  startAngle: number,
): void {
  expect(config.motors).toHaveLength(count);

  for (const [index, motor] of config.motors.entries()) {
    expectNumberClose(Math.hypot(motor.position[0], motor.position[1]), DEFAULT_ARM_LENGTH);
    expectNumberClose(motor.position[2], 0);
    expect(motor.spin).toBe(index % 2 === 0 ? 1 : -1);
  }

  // Centred at the centre of mass.
  const centre = config.motors.reduce<Vec3>(
    (sum, motor) => [
      sum[0] + motor.position[0],
      sum[1] + motor.position[1],
      sum[2] + motor.position[2],
    ],
    [0, 0, 0],
  );
  expectNumberClose(centre[0], 0);
  expectNumberClose(centre[1], 0);
  expectNumberClose(centre[2], 0);

  // Evenly spaced, with motor 0 at the documented start angle.
  const angles = rotorAngles(config);
  expectNumberClose(angles[0] ?? Number.NaN, startAngle);
  for (let index = 1; index < angles.length; index++) {
    expectNumberClose(
      (angles[index] ?? Number.NaN) - (angles[index - 1] ?? Number.NaN),
      (2 * Math.PI) / count,
    );
  }
}

describe("multirotor config presets", () => {
  test("default hex has six evenly spaced alternating rotors", () => {
    const config = makeHex();
    expectEvenLayout(config, 6, HEX_START_ANGLE);

    expectNumberClose(config.mass, DEFAULT_MASS);
    expectVec3Close(config.gravity, [0, 0, -9.81]);
    expectVec3Close(config.drag.linear, DEFAULT_LINEAR_DRAG);
    expectVec3Close(config.drag.quadratic, DEFAULT_QUADRATIC_DRAG);
    expect(config.groundPlane).toBe(DEFAULT_GROUND_PLANE);

    for (const motor of config.motors) {
      expectNumberClose(motor.kT, DEFAULT_KT);
      expectNumberClose(motor.kQ, DEFAULT_KQ);
      expectNumberClose(motor.maxOmega, DEFAULT_MAX_OMEGA);
      expectNumberClose(motor.minOmega, DEFAULT_MIN_OMEGA);
      expectNumberClose(motor.motorTimeConstant, DEFAULT_MOTOR_TIME_CONSTANT);
    }

    // Ring inertia with m = 1.5 kg, R = 0.25 m.
    const inertia = ringInertia(DEFAULT_MASS, DEFAULT_ARM_LENGTH);
    expect(config.inertia).toEqual(inertia);
  });

  test("quad uses an X layout starting at 45°", () => {
    const config = makeQuad();
    expectEvenLayout(config, 4, QUAD_START_ANGLE);

    const first = motorAt(config, 0).position;
    const diagonal = DEFAULT_ARM_LENGTH / Math.SQRT2;
    expectVec3Close(first, [diagonal, diagonal, 0]);

    // Moving counter-clockwise from motor 0 goes front-left → back-left etc.
    expectVec3Close(motorAt(config, 1).position, [-diagonal, diagonal, 0]);
    expectVec3Close(motorAt(config, 2).position, [-diagonal, -diagonal, 0]);
    expectVec3Close(motorAt(config, 3).position, [diagonal, -diagonal, 0]);
  });

  test("octo has eight evenly spaced alternating rotors", () => {
    const config = makeOcto();
    expectEvenLayout(config, 8, OCTO_START_ANGLE);
    expectVec3Close(motorAt(config, 0).position, [DEFAULT_ARM_LENGTH, 0, 0]);
  });

  test("options override every resolved field", () => {
    const inertia = mat3FromDiagonal([0.01, 0.02, 0.03]);
    const config = makeQuad({
      mass: 2,
      armLength: 0.3,
      inertia,
      gravity: [0, 0, -10],
      kT: 2e-5,
      kQ: 3e-7,
      maxOmega: 800,
      minOmega: 50,
      motorTimeConstant: 0.08,
      linearDrag: [0.1, 0.2, 0.3],
      quadraticDrag: [0.01, 0.02, 0.03],
      groundPlane: true,
    });

    expectNumberClose(config.mass, 2);
    expectVec3Close(config.gravity, [0, 0, -10]);
    expect(config.inertia).toEqual(inertia);
    expectVec3Close(config.drag.linear, [0.1, 0.2, 0.3]);
    expectVec3Close(config.drag.quadratic, [0.01, 0.02, 0.03]);
    expect(config.groundPlane).toBe(true);

    for (const motor of config.motors) {
      expectNumberClose(Math.hypot(motor.position[0], motor.position[1]), 0.3);
      expectNumberClose(motor.kT, 2e-5);
      expectNumberClose(motor.kQ, 3e-7);
      expectNumberClose(motor.maxOmega, 800);
      expectNumberClose(motor.minOmega, 50);
      expectNumberClose(motor.motorTimeConstant, 0.08);
    }
  });

  test("ring inertia is ½mR² laterally and mR² about z", () => {
    const inertia = ringInertia(2, 0.5);
    expectVec3Close([inertia[0], inertia[4], inertia[8]], [0.25, 0.25, 0.5]);
    expect(inertia[1]).toBe(0);
    expect(inertia[2]).toBe(0);
  });

  test("hover speeds and thrust-to-weight are physically plausible", () => {
    // Hex: 14.715 N / 6 rotors / 1e-5 ⇒ ω ≈ 495.23 rad/s; T/W ≈ 4.08.
    const hex = makeHex();
    const hexHover = hoverMotorSpeeds(hex);
    expect(hexHover).toHaveLength(hex.motors.length);
    for (const speed of hexHover) {
      expect(speed).toBeGreaterThan(495);
      expect(speed).toBeLessThan(496);
      expectNumberClose(speed, hexHover[0] ?? Number.NaN);
    }
    expectNumberClose(maxThrust(hex), 60);
    expectNumberClose(minThrust(hex), 0);
    expectNumberClose(thrustToWeight(hex), 60 / (DEFAULT_MASS * 9.81));

    for (const config of [makeQuad(), makeHex(), makeOcto()]) {
      const speeds = hoverMotorSpeeds(config);
      expect(speeds.every((speed) => speed > 0)).toBe(true);
      expect(thrustToWeight(config)).toBeGreaterThan(1);
    }

    // Quad has four rotors ⇒ 40 N, octo eight ⇒ 80 N.
    expectNumberClose(maxThrust(makeQuad()), 40);
    expectNumberClose(maxThrust(makeOcto()), 80);
  });

  test("hover speeds account for per-motor thrust coefficients", () => {
    const config = withMotor(makeQuad(), { kT: 4e-5 });
    const speeds = hoverMotorSpeeds(config);
    const expectedModified = Math.sqrt(
      (DEFAULT_MASS * 9.81) / config.motors.length / 4e-5,
    );
    expectNumberClose(speeds[0] ?? Number.NaN, expectedModified);

    const untouched = Math.sqrt((DEFAULT_MASS * 9.81) / config.motors.length / DEFAULT_KT);
    expectNumberClose(speeds[1] ?? Number.NaN, untouched);
  });

  test("rigidBodyParamsOf mirrors the config", () => {
    const config = makeHex();
    const params = rigidBodyParamsOf(config);
    expectNumberClose(params.mass, config.mass);
    expect(params.inertia).toEqual(config.inertia);
    expectVec3Close(params.gravity ?? [0, 0, 0], config.gravity);
  });
});

describe("multirotor config validation", () => {
  const base = makeHex();

  test("rejects an invalid body", () => {
    expect(() => validateConfig({ ...base, mass: 0 })).toThrow(RangeError);
    expect(() => validateConfig({ ...base, mass: Number.NaN })).toThrow(
      RangeError,
    );
    expect(() =>
      validateConfig({ ...base, inertia: mat3FromDiagonal([1, 0, 1]) }),
    ).toThrow(RangeError);
    expect(() =>
      validateConfig({ ...base, inertia: mat3FromDiagonal([1, Number.NaN, 1]) }),
    ).toThrow(RangeError);
    expect(() => validateConfig({ ...base, gravity: [0, 0, Number.NaN] })).toThrow(
      RangeError,
    );
  });

  test("rejects invalid drag", () => {
    expect(() =>
      validateConfig({
        ...base,
        drag: { linear: [-1, 0, 0], quadratic: [0, 0, 0] },
      }),
    ).toThrow(RangeError);
    expect(() =>
      validateConfig({
        ...base,
        drag: { linear: [0, 0, 0], quadratic: [0, Number.NaN, 0] },
      }),
    ).toThrow(RangeError);
  });

  test("rejects too few motors", () => {
    expect(() => validateConfig({ ...base, motors: base.motors.slice(0, 3) })).toThrow(
      RangeError,
    );
  });

  test("rejects invalid motor parameters", () => {
    const invalidMotor: ReadonlyArray<Partial<MotorConfig>> = [
      { position: [0, 0, Number.NaN] },
      { spin: 0 as unknown as SpinDirection },
      { kT: 0 },
      { kT: Number.POSITIVE_INFINITY },
      { kQ: -1e-7 },
      { maxOmega: 0 },
      { minOmega: -1 },
      { minOmega: DEFAULT_MAX_OMEGA },
      { motorTimeConstant: 0 },
      { motorTimeConstant: Number.NaN },
    ];

    for (const overrides of invalidMotor) {
      expect(() => validateConfig(withMotor(base, overrides))).toThrow(
        RangeError,
      );
    }
  });

  test("factories validate their resolved configs", () => {
    expect(() => makeHex({ mass: 0 })).toThrow(RangeError);
    expect(() => makeHex({ mass: Number.NaN })).toThrow(RangeError);
    expect(() => makeQuad({ armLength: 0 })).toThrow(RangeError);
    expect(() => makeQuad({ armLength: -0.2 })).toThrow(RangeError);
    expect(() => makeQuad({ kT: -1 })).toThrow(RangeError);
    expect(() => makeOcto({ minOmega: 1200 })).toThrow(RangeError);
    expect(() => makeOcto({ linearDrag: [-0.1, 0, 0] })).toThrow(RangeError);
    expect(() =>
      makeHex({ inertia: mat3FromDiagonal([1, 0, 1]) }),
    ).toThrow(RangeError);
  });

  test("thrustToWeight rejects zero gravity", () => {
    expect(() => thrustToWeight({ ...base, gravity: [0, 0, 0] })).toThrow(
      RangeError,
    );
  });
});
