import { describe, expect, test } from "vitest";

import { totalEnergy } from "../src/core/energy";
import { step } from "../src/core/integrator";
import {
  QUAT_IDENTITY,
  quatFromAxisAngle,
  type Quat,
} from "../src/core/math/quat";
import {
  vec3Add,
  vec3Cross,
  type Vec3,
} from "../src/core/math/vec3";
import {
  ZERO_INPUTS,
  type RigidBodyState,
} from "../src/core/rigidBody";
import {
  hoverMotorSpeeds,
  makeHex,
  rigidBodyParamsOf,
  type MotorConfig,
  type MultirotorConfig,
} from "../src/core/multirotor/config";
import {
  dragForceBody,
  dragForceWorld,
  motorWrench,
  multirotorDerivatives,
  stepMultirotor,
  STILL_AIR,
  totalThrust,
  type Environment,
  type MultirotorState,
} from "../src/core/multirotor/dynamics";

const TOL = 1e-12;

function close(actual: number, expected: number, tol = TOL): void {
  expect(Math.abs(actual - expected)).toBeLessThan(tol);
}

function vecClose(actual: Vec3, expected: Vec3, tol = TOL): void {
  close(actual[0], expected[0], tol);
  close(actual[1], expected[1], tol);
  close(actual[2], expected[2], tol);
}

function quatClose(actual: Quat, expected: Quat, tol = TOL): void {
  close(actual[0], expected[0], tol);
  close(actual[1], expected[1], tol);
  close(actual[2], expected[2], tol);
  close(actual[3], expected[3], tol);
}

function quatNorm(q: Quat): number {
  return Math.sqrt(
    q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3],
  );
}

function motorAt(config: MultirotorConfig, index: number): MotorConfig {
  const motor = config.motors[index];
  if (motor === undefined) {
    throw new Error(`config has no motor ${index}`);
  }
  return motor;
}

interface StateOverrides {
  readonly body?: RigidBodyState;
  readonly position?: Vec3;
  readonly velocity?: Vec3;
  readonly orientation?: Quat;
  readonly angularVelocity?: Vec3;
  readonly motorSpeeds?: readonly number[];
}

function makeState(
  config: MultirotorConfig,
  overrides: StateOverrides = {},
): MultirotorState {
  const body: RigidBodyState = overrides.body ?? {
    position: overrides.position ?? [0, 0, 0],
    velocity: overrides.velocity ?? [0, 0, 0],
    orientation: overrides.orientation ?? QUAT_IDENTITY,
    angularVelocity: overrides.angularVelocity ?? [0, 0, 0],
  };
  return {
    body,
    motorSpeeds: overrides.motorSpeeds ?? hoverMotorSpeeds(config),
  };
}

function stopped(config: MultirotorConfig): number[] {
  return config.motors.map(() => 0);
}

function stepN(
  state: MultirotorState,
  config: MultirotorConfig,
  commands: readonly number[],
  steps: number,
  dt: number,
  environment: Environment = STILL_AIR,
): MultirotorState {
  let current = state;
  for (let index = 0; index < steps; index++) {
    current = stepMultirotor(current, config, commands, dt, environment);
  }
  return current;
}

describe("motor wrench", () => {
  test("hover motors give thrust equal to weight and cancelling torques", () => {
    const config = makeHex();
    const wrench = motorWrench(makeState(config), config);
    vecClose(wrench.forceBody, [0, 0, config.mass * 9.81]);
    vecClose(wrench.torqueBody, [0, 0, 0]);
  });

  test("totalThrust sums kT ω² over the rotors", () => {
    const config = makeHex();
    const state = makeState(config, {
      motorSpeeds: config.motors.map((_, index) => 100 + index * 10),
    });
    const expected = config.motors.reduce(
      (sum, motor, index) => sum + motor.kT * (100 + index * 10) ** 2,
      0,
    );
    close(totalThrust(state, config), expected);
    close(totalThrust(makeState(config), config), config.mass * 9.81);
  });

  test("a single rotor produces r × F plus its signed reaction torque", () => {
    const config = makeHex();
    const speeds = stopped(config);
    speeds[2] = 400;
    const motor = motorAt(config, 2);
    const thrust = motor.kT * 400 * 400;
    const reaction = -motor.spin * motor.kQ * 400 * 400;
    const expectedTorque = vec3Add(
      vec3Cross(motor.position, [0, 0, thrust]),
      [0, 0, reaction],
    );

    const wrench = motorWrench(makeState(config, { motorSpeeds: speeds }), config);
    vecClose(wrench.forceBody, [0, 0, thrust]);
    vecClose(wrench.torqueBody, expectedTorque);
  });

  test("all CCW rotors yaw negative, all CW rotors yaw positive", () => {
    const base = makeHex();
    const speeds = base.motors.map(() => 300);
    const ccw: MultirotorConfig = {
      ...base,
      motors: base.motors.map((motor) => ({ ...motor, spin: 1 as const })),
    };
    const cw: MultirotorConfig = {
      ...base,
      motors: base.motors.map((motor) => ({ ...motor, spin: -1 as const })),
    };
    const ccwWrench = motorWrench(makeState(ccw, { motorSpeeds: speeds }), ccw);
    const cwWrench = motorWrench(makeState(cw, { motorSpeeds: speeds }), cw);
    expect(ccwWrench.torqueBody[2]).toBeLessThan(0);
    expect(cwWrench.torqueBody[2]).toBeGreaterThan(0);
    close(ccwWrench.torqueBody[2], -cwWrench.torqueBody[2]);
  });
});

describe("body-frame drag", () => {
  const config = makeHex();

  test("combines linear and isotropic quadratic terms per axis", () => {
    const force = dragForceBody({
      ...makeState(config).body,
      velocity: [1, 0, 0],
    }, config);
    vecClose(force, [-(0.25 + 0.02), 0, 0]);

    const diagonal = dragForceBody({
      ...makeState(config).body,
      velocity: [3, 4, 0],
    }, config);
    // |v| = 5, so the quadratic term uses 5·v_i on each axis.
    vecClose(diagonal, [-(0.25 * 3 + 0.02 * 5 * 3), -(0.25 * 4 + 0.02 * 5 * 4), 0]);
  });

  test("uses air-relative velocity, not ground velocity", () => {
    const wind: Environment = { windVelocity: [2, 0, 0] };
    const matching = dragForceBody({
      ...makeState(config).body,
      velocity: [2, 0, 0],
    }, config, wind);
    vecClose(matching, [0, 0, 0]);

    const stationary = dragForceBody(makeState(config).body, config, wind);
    vecClose(stationary, [0.25 * 2 + 0.02 * 2 * 2, 0, 0]);
  });

  test("rotates with the body without changing body-frame force", () => {
    const q = quatFromAxisAngle([0, 0, 1], Math.PI / 2);
    const velocityBody: Vec3 = [1, 2, 0];
    const identityForce = dragForceBody({
      ...makeState(config).body,
      velocity: velocityBody,
    }, config);
    const rotated = dragForceBody({
      ...makeState(config).body,
      orientation: q,
      velocity: quatRotateForTest(q, velocityBody),
    }, config);
    vecClose(rotated, identityForce, 1e-13);
  });

  test("is zero at rest in still air and produces no torque", () => {
    vecClose(dragForceBody(makeState(config).body, config), [0, 0, 0]);
    vecClose(dragForceWorld(makeState(config).body, config), [0, 0, 0]);

    const moving = makeState(config, {
      velocity: [1, 0, 0],
      motorSpeeds: stopped(config),
    });
    vecClose(
      multirotorDerivatives(moving, config, stopped(config)).body.angularVelocity,
      [0, 0, 0],
    );
  });
});

// Same convention as quatRotateVector, kept local to make the frame test explicit.
function quatRotateForTest(q: Quat, v: Vec3): Vec3 {
  const [w, x, y, z] = q;
  const r: Vec3 = [
    (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2],
    2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2],
    2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2],
  ];
  return r;
}

describe("extended derivatives", () => {
  const config = makeHex();

  test("zero net force and torque at hover", () => {
    const speeds = hoverMotorSpeeds(config);
    const d = multirotorDerivatives(makeState(config), config, speeds);
    vecClose(d.body.velocity, [0, 0, 0]);
    vecClose(d.body.angularVelocity, [0, 0, 0]);
    expect(d.motorSpeeds).toHaveLength(config.motors.length);
    for (const rate of d.motorSpeeds) {
      close(rate, 0);
    }
  });

  test("thrust follows the body z axis when tilted", () => {
    const q = quatFromAxisAngle([1, 0, 0], Math.PI / 2);
    const speeds = hoverMotorSpeeds(config);
    const d = multirotorDerivatives(
      makeState(config, { orientation: q }),
      config,
      speeds,
    );
    // Thrust magnitude mg, rotated to world -y; gravity remains -z.
    vecClose(d.body.velocity, [0, -9.81, -9.81], 1e-9);
  });

  test("force depends on orientation, not quaternion magnitude", () => {
    const speeds = hoverMotorSpeeds(config);
    const q = quatFromAxisAngle([0.2, 0.5, -0.3], 0.7);
    const scaled: Quat = [q[0] * 1.01, q[1] * 1.01, q[2] * 1.01, q[3] * 1.01];
    const unitAcceleration = multirotorDerivatives(
      makeState(config, { orientation: q }),
      config,
      speeds,
    ).body.velocity;
    const scaledAcceleration = multirotorDerivatives(
      makeState(config, { orientation: scaled }),
      config,
      speeds,
    ).body.velocity;
    vecClose(scaledAcceleration, unitAcceleration, 1e-12);
  });
});

describe("extended RK4 integration", () => {
  const dt = 1e-3;

  test("holds a level hover for 10 s at 1 kHz", () => {
    const config = makeHex();
    const speeds = hoverMotorSpeeds(config);
    let state = makeState(config, { position: [0, 0, 5], motorSpeeds: speeds });
    for (let index = 0; index < 10_000; index++) {
      state = stepMultirotor(state, config, speeds, dt);
    }

    vecClose(state.body.position, [0, 0, 5], 1e-8);
    quatClose(state.body.orientation, QUAT_IDENTITY, 1e-8);
    for (const speed of state.motorSpeeds) {
      close(speed, speeds[0] ?? Number.NaN, 1e-12);
    }
    const drift = Math.hypot(...state.body.position.map((v, i) => v - [0, 0, 5][i]!));
    console.log(`  hover drift after 10 s: ${drift.toExponential(3)} m`);
  });

  test("reduces exactly to Milestone 1 when motors and drag are off", () => {
    const config = makeHex({ linearDrag: [0, 0, 0], quadraticDrag: [0, 0, 0] });
    const speeds = stopped(config);
    const body: RigidBodyState = {
      position: [0, 0, 10],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [1, 0.5, 0.2],
    };
    const params = rigidBodyParamsOf(config);

    let rigid = body;
    let multi = makeState(config, { body, motorSpeeds: speeds });
    let maxDifference = 0;
    for (let index = 0; index < 10_000; index++) {
      rigid = step(rigid, params, ZERO_INPUTS, dt);
      multi = stepMultirotor(multi, config, speeds, dt);
      maxDifference = Math.max(
        maxDifference,
        ...vec3Diff(rigid.position, multi.body.position),
        ...vec3Diff(rigid.velocity, multi.body.velocity),
        ...vec3Diff(rigid.angularVelocity, multi.body.angularVelocity),
        ...quatDiff(rigid.orientation, multi.body.orientation),
      );
    }
    expect(maxDifference).toBeLessThan(1e-12);
    console.log(`  max M1-vs-M2 body difference: ${maxDifference.toExponential(3)}`);
  });

  test("linear drag follows v(t) = (mg/c)(1 - exp(-ct/m))", () => {
    const c = 0.4;
    const config = makeHex({ linearDrag: [0, 0, c], quadraticDrag: [0, 0, 0] });
    const speeds = stopped(config);
    const terminal = (config.mass * 9.81) / c;
    const timeConstant = config.mass / c;
    const checkpoints = new Map([
      [1000, 1],
      [2000, 2],
      [4000, 4],
      [8000, 8],
    ]);

    let state = makeState(config, { position: [0, 0, 1000], motorSpeeds: speeds });
    for (let index = 1; index <= 8000; index++) {
      state = stepMultirotor(state, config, speeds, dt);
      const time = checkpoints.get(index);
      if (time !== undefined) {
        close(
          state.body.velocity[2],
          -terminal * (1 - Math.exp(-time / timeConstant)),
          1e-6,
        );
      }
    }
  });

  test("quadratic drag follows v(t) = v_t tanh(gt/v_t)", () => {
    const c2 = 0.03;
    const config = makeHex({ linearDrag: [0, 0, 0], quadraticDrag: [0, 0, c2] });
    const speeds = stopped(config);
    const terminal = Math.sqrt((config.mass * 9.81) / c2);
    const checkpoints = new Map([
      [1000, 1],
      [2000, 2],
      [4000, 4],
      [20000, 20],
    ]);

    let state = makeState(config, { position: [0, 0, 1000], motorSpeeds: speeds });
    for (let index = 1; index <= 20_000; index++) {
      state = stepMultirotor(state, config, speeds, dt);
      const time = checkpoints.get(index);
      if (time !== undefined) {
        close(
          state.body.velocity[2],
          -terminal * Math.tanh((9.81 * time) / terminal),
          1e-6,
        );
      }
    }
    close(state.body.velocity[2], -terminal, 1e-6);
  });

  test("wind advects a hovering vehicle to the air velocity", () => {
    const config = makeHex({ quadraticDrag: [0, 0, 0] });
    const speeds = hoverMotorSpeeds(config);
    const wind: Environment = { windVelocity: [3, 0, 0] };
    const checkpoints = new Map([
      [2000, 2],
      [6000, 6],
      [12000, 12],
    ]);

    let state = makeState(config, { position: [0, 0, 5], motorSpeeds: speeds });
    for (let index = 1; index <= 12_000; index++) {
      state = stepMultirotor(state, config, speeds, dt, wind);
      const time = checkpoints.get(index);
      if (time !== undefined) {
        close(state.body.velocity[0], 3 * (1 - Math.exp(-(0.25 * time) / 1.5)), 1e-6);
        close(state.body.velocity[2], 0, 1e-6);
      }
    }
  });

  test("mechanical energy decreases under drag", () => {
    const config = makeHex({ linearDrag: [0.25, 0.25, 0.4] });
    const speeds = stopped(config);
    const params = rigidBodyParamsOf(config);
    let state = makeState(config, { position: [0, 0, 50], motorSpeeds: speeds });
    let previous = totalEnergy(state.body, params);
    for (let index = 0; index < 2000; index++) {
      state = stepMultirotor(state, config, speeds, dt);
      const energy = totalEnergy(state.body, params);
      expect(energy).toBeLessThanOrEqual(previous + 1e-9);
      previous = energy;
    }
  });

  test("is deterministic and keeps the quaternion normalized", () => {
    const config = makeHex();
    const speeds = hoverMotorSpeeds(config);
    const commands = speeds.map((speed, index) => {
      if (index === 1 || index === 2) return speed + 20;
      if (index === 4 || index === 5) return speed - 20;
      return speed;
    });
    const initial = makeState(config, { position: [0, 0, 5], motorSpeeds: speeds });

    const runOnce = (): MultirotorState => stepN(initial, config, commands, 5000, dt);
    expect(runOnce()).toEqual(runOnce());

    let state = initial;
    let maxNormError = 0;
    for (let index = 0; index < 5000; index++) {
      state = stepMultirotor(state, config, commands, dt);
      maxNormError = Math.max(maxNormError, Math.abs(quatNorm(state.body.orientation) - 1));
      for (const speed of state.motorSpeeds) {
        expect(speed).toBeGreaterThanOrEqual(0);
        expect(speed).toBeLessThanOrEqual(1000);
      }
    }
    expect(maxNormError).toBeLessThan(1e-12);
  }, 10_000);

  test("default environment matches explicit still air", () => {
    const config = makeHex();
    const speeds = hoverMotorSpeeds(config);
    const initial = makeState(config, { motorSpeeds: speeds });
    expect(stepN(initial, config, speeds, 100, dt)).toEqual(
      stepN(initial, config, speeds, 100, dt, STILL_AIR),
    );
  });

  test("does not mutate its inputs", () => {
    const config = makeHex();
    const state = makeState(config, {
      position: [1, 2, 3],
      velocity: [0.5, 0, 0],
      motorSpeeds: hoverMotorSpeeds(config),
    });
    const before = JSON.stringify(state);
    stepMultirotor(state, config, state.motorSpeeds, dt);
    expect(JSON.stringify(state)).toBe(before);
  });
});

describe("motor lag integration", () => {
  const config = makeHex();
  const tau = 0.05;
  const command = config.motors.map(() => 5000); // clamped to maxOmega = 1000
  const analytic = (time: number): number => 1000 * (1 - Math.exp(-time / tau));

  test("matches the first-order analytic response, including 63.2% at t = τ", () => {
    const dt = 1e-4;
    const checkpoints = new Map([
      [100, 0.01],
      [500, tau],
      [1000, 0.1],
      [2500, 0.25],
      [5000, 0.5],
    ]);

    let state = makeState(config, { motorSpeeds: stopped(config) });
    for (let index = 1; index <= 5000; index++) {
      state = stepMultirotor(state, config, command, dt);
      const time = checkpoints.get(index);
      if (time !== undefined) {
        for (const speed of state.motorSpeeds) {
          close(speed, analytic(time), 1e-6);
        }
      }
      if (index === 500) {
        close(state.motorSpeeds[0] ?? Number.NaN, 1000 * (1 - Math.exp(-1)), 1e-6);
      }
    }
  });

  test("respects commanded speed limits in both directions", () => {
    const limited = makeHex({ minOmega: 200 });
    let state = makeState(limited, { motorSpeeds: limited.motors.map(() => 600) });

    state = stepN(state, limited, limited.motors.map(() => 0), 2000, 1e-3);
    for (const speed of state.motorSpeeds) {
      close(speed, 200, 1e-6);
    }

    state = stepN(state, limited, limited.motors.map(() => 5000), 2000, 1e-3);
    for (const speed of state.motorSpeeds) {
      close(speed, 1000, 1e-6);
    }
  });

  test("converges at fourth order when dt is halved", () => {
    const endTime = 0.2;
    const errors: number[] = [];
    for (const dt of [8e-3, 4e-3, 2e-3]) {
      const steps = Math.round(endTime / dt);
      const state = stepN(
        makeState(config, { motorSpeeds: stopped(config) }),
        config,
        command,
        steps,
        dt,
      );
      errors.push(Math.abs((state.motorSpeeds[0] ?? Number.NaN) - analytic(endTime)));
    }

    const firstRatio = (errors[0] ?? Number.NaN) / (errors[1] ?? Number.NaN);
    const secondRatio = (errors[1] ?? Number.NaN) / (errors[2] ?? Number.NaN);
    expect(firstRatio).toBeGreaterThan(12);
    expect(firstRatio).toBeLessThan(20);
    expect(secondRatio).toBeGreaterThan(12);
    expect(secondRatio).toBeLessThan(20);
    console.log(
      `  lag errors: ${errors.map((error) => error.toExponential(3)).join(", ")}; ratios ${firstRatio.toFixed(2)}, ${secondRatio.toFixed(2)}`,
    );
  });
});

describe("validation", () => {
  const config = makeHex();
  const speeds = hoverMotorSpeeds(config);

  test("rejects bad dt", () => {
    const state = makeState(config);
    for (const dt of [0, -1e-3, Number.NaN, Number.POSITIVE_INFINITY]) {
      expect(() => stepMultirotor(state, config, speeds, dt)).toThrow(RangeError);
    }
  });

  test("rejects wrong command and speed lengths", () => {
    expect(() =>
      stepMultirotor(makeState(config), config, speeds.slice(1), 1e-3),
    ).toThrow(RangeError);
    expect(() =>
      stepMultirotor(
        makeState(config, { motorSpeeds: speeds.slice(1) }),
        config,
        speeds,
        1e-3,
      ),
    ).toThrow(RangeError);
    expect(() =>
      multirotorDerivatives(makeState(config), config, [...speeds, 0]),
    ).toThrow(RangeError);
  });

  test("rejects non-finite commands, speeds and wind", () => {
    const badCommands = [...speeds];
    badCommands[0] = Number.NaN;
    expect(() => stepMultirotor(makeState(config), config, badCommands, 1e-3)).toThrow(
      RangeError,
    );

    const badSpeeds = [...speeds];
    badSpeeds[0] = Number.NaN;
    expect(() =>
      stepMultirotor(makeState(config, { motorSpeeds: badSpeeds }), config, speeds, 1e-3),
    ).toThrow(RangeError);

    expect(() =>
      stepMultirotor(makeState(config), config, speeds, 1e-3, {
        windVelocity: [0, Number.NaN, 0],
      }),
    ).toThrow(RangeError);
  });

  test("rejects speeds outside [0, maxOmega]", () => {
    const negative = [...speeds];
    negative[0] = -1;
    expect(() =>
      stepMultirotor(makeState(config, { motorSpeeds: negative }), config, speeds, 1e-3),
    ).toThrow(RangeError);

    const over = [...speeds];
    over[0] = 1001;
    expect(() =>
      stepMultirotor(makeState(config, { motorSpeeds: over }), config, speeds, 1e-3),
    ).toThrow(RangeError);
  });

  test("rejects an invalid config", () => {
    expect(() =>
      stepMultirotor(makeState(config), { ...config, mass: 0 }, speeds, 1e-3),
    ).toThrow(RangeError);
  });
});

function vec3Diff(a: Vec3, b: Vec3): number[] {
  return [Math.abs(a[0] - b[0]), Math.abs(a[1] - b[1]), Math.abs(a[2] - b[2])];
}

function quatDiff(a: Quat, b: Quat): number[] {
  return [
    Math.abs(a[0] - b[0]),
    Math.abs(a[1] - b[1]),
    Math.abs(a[2] - b[2]),
    Math.abs(a[3] - b[3]),
  ];
}
