/**
 * Milestone 2 acceptance matrix.
 *
 * Every test crosses at least two modules: a wrench is allocated by the mixer,
 * then consumed by the motor model and the rigid-body/integrator stack. Unit
 * tests for each module live beside that module; this file proves they compose
 * into a physically correct vehicle.
 */

import { describe, expect, test } from "vitest";

import {
  QUAT_IDENTITY,
  quatFromAxisAngle,
  quatMultiply,
  quatRotateVector,
  quatToEulerZYX,
  type Quat,
} from "../src/core/math/quat";
import {
  hoverMotorSpeeds,
  makeHex,
  makeOcto,
  makeQuad,
  type MultirotorConfig,
} from "../src/core/multirotor/config";
import {
  motorWrench,
  multirotorDerivatives,
  stepMultirotor,
  totalThrust,
  type Environment,
  type MultirotorState,
} from "../src/core/multirotor/dynamics";
import {
  mix,
  mixDetailed,
  mixerMatrix,
  type Wrench,
} from "../src/core/multirotor/mixer";
import {
  at,
  close,
  hoverWrench,
  makeState,
  meanSpeed,
  quatClose,
  quatDot,
  quatNorm,
  stepN,
  torqueWrench,
  vecClose,
} from "./helpers/multirotor";

describe("M2 integration matrix", () => {
  test("1. hover equilibrium through mix and derivatives", () => {
    for (const config of [makeQuad(), makeHex(), makeOcto()]) {
      const wrench = hoverWrench(config);
      const mixed = mixDetailed(config, wrench);
      vecClose(mixed.motorSpeeds, hoverMotorSpeeds(config), 1e-9);
      vecClose(mixed.achieved, wrench, 1e-9);
      expect(mixed.saturated).toBe(false);

      const state = makeState(config, { motorSpeeds: mixed.motorSpeeds });
      const derivative = multirotorDerivatives(state, config, mixed.motorSpeeds);
      vecClose(derivative.body.velocity, [0, 0, 0], 1e-9);
      vecClose(derivative.body.angularVelocity, [0, 0, 0], 1e-12);
      for (const rate of derivative.motorSpeeds) {
        close(rate, 0, 1e-12);
      }
      close(motorWrench(state, config).forceBody[2], config.mass * 9.81, 1e-9);
      close(totalThrust(state, config), config.mass * 9.81, 1e-9);
    }
  });

  test("2. holds a level hover for 10 s at 1 kHz", () => {
    const config = makeHex();
    const commands = mix(config, hoverWrench(config));
    let state = makeState(config, {
      position: [0, 0, 5],
      motorSpeeds: commands,
    });

    for (let index = 0; index < 10_000; index++) {
      state = stepMultirotor(state, config, commands, 1e-3);
    }

    vecClose(state.body.position, [0, 0, 5], 1e-8);
    quatClose(state.body.orientation, QUAT_IDENTITY, 1e-8);
    vecClose(state.body.velocity, [0, 0, 0], 1e-9);
    for (const [index, speed] of state.motorSpeeds.entries()) {
      close(speed, at(commands, index), 1e-12);
    }
    const drift = Math.hypot(
      state.body.position[0],
      state.body.position[1],
      state.body.position[2] - 5,
    );
    expect(drift).toBeLessThan(1e-8);
    console.log(`  hover drift after 10 s: ${drift.toExponential(3)} m`);
  });

  test("3. climbs at 1.1x hover thrust with constant acceleration", () => {
    const config = makeHex({ linearDrag: [0, 0, 0], quadraticDrag: [0, 0, 0] });
    // 1.1× hover *thrust*: the speed is sqrt(1.1) × hover speed (~519.4 rad/s),
    // not 1.1 × hover speed.
    const wrench: Wrench = [1.1 * config.mass * 9.81, 0, 0, 0];
    const mixed = mixDetailed(config, wrench);
    expect(mixed.saturated).toBe(false);
    close(mixed.achieved[0], 1.1 * config.mass * 9.81, 1e-9);

    let state = makeState(config, { motorSpeeds: mixed.motorSpeeds });
    const derivative = multirotorDerivatives(state, config, mixed.motorSpeeds);
    vecClose(derivative.body.velocity, [0, 0, 0.981], 1e-9);
    for (const rate of derivative.motorSpeeds) {
      close(rate, 0, 1e-12);
    }

    const checkpoints = new Map([
      [1000, 1],
      [2000, 2],
    ]);
    for (let index = 1; index <= 2000; index++) {
      state = stepMultirotor(state, config, mixed.motorSpeeds, 1e-3);
      const time = checkpoints.get(index);
      if (time !== undefined) {
        close(state.body.velocity[2], 0.981 * time, 1e-9);
        close(state.body.position[2], 0.4905 * time * time, 1e-9);
      }
    }
    vecClose(state.body.velocity, [0, 0, 1.962], 1e-9);
  });

  /** Assert that a small pure body torque produces the expected principal-axis rotation. */
  function assertAxisResponse(axis: 1 | 2 | 3, torque: number): void {
    const config = makeHex();
    const inertiaIndex = axis === 1 ? 0 : axis === 2 ? 4 : 8;
    const inertia = at(config.inertia, inertiaIndex);
    const wrench = torqueWrench(config, axis, torque);
    const mixed = mixDetailed(config, wrench);
    expect(mixed.saturated).toBe(false);
    vecClose(mixed.achieved, wrench, 1e-9);

    const state = makeState(config, { motorSpeeds: mixed.motorSpeeds });
    const derivative = multirotorDerivatives(state, config, mixed.motorSpeeds);
    vecClose(derivative.body.velocity, [0, 0, 0], 1e-9);

    const alpha = torque / inertia;
    close(at(derivative.body.angularVelocity, axis - 1), alpha, 1e-9);
    for (const other of [0, 1, 2]) {
      if (other !== axis - 1) {
        close(at(derivative.body.angularVelocity, other), 0, 1e-12);
      }
    }

    let integrated = state;
    for (let index = 0; index < 100; index++) {
      integrated = stepMultirotor(integrated, config, mixed.motorSpeeds, 1e-3);
    }
    close(at(integrated.body.angularVelocity, axis - 1), alpha * 0.1, 1e-9);
    const euler = quatToEulerZYX(integrated.body.orientation);
    close(at(euler, axis - 1), 0.5 * alpha * 0.1 * 0.1, 1e-8);
    expect(at(euler, axis - 1)).toBeGreaterThan(0);
  }

  test("4. positive roll torque rolls right", () => {
    assertAxisResponse(1, 0.05);
  });

  test("5. positive pitch torque pitches up", () => {
    assertAxisResponse(2, 0.05);
  });

  test("6. positive yaw torque yaws left and speeds CW rotors", () => {
    assertAxisResponse(3, 0.01);
    const config = makeHex();
    const mixed = mixDetailed(config, torqueWrench(config, 3, 0.01));
    expect(meanSpeed(config, mixed.motorSpeeds, -1)).toBeGreaterThan(
      meanSpeed(config, mixed.motorSpeeds, 1),
    );
  });

  test("6b. lagged torque commands still turn the right way", () => {
    // Tests 4–6 start at the mixed speeds, so the 0.05 s motor lag never enters.
    // The PID in Milestone 3 will instead hold hover and step a torque command,
    // so exercise that path: start at hover, apply the mixed command, check the
    // angular-rate sign after 0.2 s (4 time constants). Only the sign is pinned;
    // the exact magnitude is deliberately not asserted during the transient.
    const config = makeHex();
    const hover = hoverMotorSpeeds(config);
    const cases: ReadonlyArray<readonly [1 | 2 | 3, number]> = [
      [1, 0.05],
      [2, 0.05],
      [3, 0.01],
    ];

    for (const [axis, torque] of cases) {
      const commands = mix(config, torqueWrench(config, axis, torque));
      let state = makeState(config, { motorSpeeds: hover });

      for (let index = 0; index < 200; index++) {
        state = stepMultirotor(state, config, commands, 1e-3);
      }

      expect(at(state.body.angularVelocity, axis - 1)).toBeGreaterThan(0);
      const inTransit = state.motorSpeeds.some(
        (speed, index) =>
          Math.abs(speed - at(hover, index)) > 1e-6 &&
          Math.abs(speed - at(commands, index)) > 1e-6,
      );
      expect(inTransit).toBe(true);
    }
  });

  test("7. all-CCW rotors yaw negative and the mixer is singular", () => {
    const base = makeHex();
    const allCcw: MultirotorConfig = {
      ...base,
      motors: base.motors.map((motor) => ({ ...motor, spin: 1 as const })),
    };
    const speeds = allCcw.motors.map(() => 300);
    const state = makeState(allCcw, { motorSpeeds: speeds });

    close(motorWrench(state, allCcw).torqueBody[2], -0.081, 1e-9);
    close(
      multirotorDerivatives(state, allCcw, speeds).body.angularVelocity[2],
      -0.864,
      1e-9,
    );
    // Every reaction row is a scalar multiple of the thrust row: rank 3.
    expect(() => mixerMatrix(allCcw)).toThrow(RangeError);
  });

  describe("8. drag through the full pipeline", () => {
    test("8a. linear drag reaches v_t = mg/c", () => {
      const config = makeHex({ quadraticDrag: [0, 0, 0] });
      const commands = mix(config, [0, 0, 0, 0]);
      expect(commands.every((speed) => speed === 0)).toBe(true);

      // Start at the mixed ZERO-speed commands, not at makeState's hover default:
      // a hover start would inject a thrust transient through the 0.05 s motor
      // lag during the first ~0.25 s and spoil the analytic drag solution.
      let state = makeState(config, {
        position: [0, 0, 1000],
        motorSpeeds: commands,
      });
      expect(state.motorSpeeds.every((speed) => speed === 0)).toBe(true);
      const terminal = (config.mass * 9.81) / 0.4;
      const timeConstant = config.mass / 0.4;
      const checkpoints = new Map([
        [1000, 1],
        [2000, 2],
        [4000, 4],
        [8000, 8],
      ]);

      for (let index = 1; index <= 8000; index++) {
        state = stepMultirotor(state, config, commands, 1e-3);
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

    test("8b. quadratic drag reaches v_t = sqrt(mg/c2)", () => {
      const config = makeHex({ linearDrag: [0, 0, 0] });
      const commands = mix(config, [0, 0, 0, 0]);
      // Same as 8a: start at the mixed zero-speed commands so no motor lag
      // transient contaminates the analytic tanh solution.
      let state = makeState(config, {
        position: [0, 0, 1000],
        motorSpeeds: commands,
      });
      expect(state.motorSpeeds.every((speed) => speed === 0)).toBe(true);
      const terminal = Math.sqrt((config.mass * 9.81) / 0.03);
      const checkpoints = new Map([
        [1000, 1],
        [2000, 2],
        [4000, 4],
        [30000, 30],
      ]);

      for (let index = 1; index <= 30000; index++) {
        state = stepMultirotor(state, config, commands, 1e-3);
        const time = checkpoints.get(index);
        if (time !== undefined) {
          close(
            state.body.velocity[2],
            -terminal * Math.tanh((9.81 * time) / terminal),
            1e-6,
          );
        }
      }
      // tanh(g·t/v_t) approaches 1 only asymptotically: at 20 s it is still
      // ~2e-8 short, leaving ~9e-7 of velocity error. Running to 30 s reduces
      // that to ~1e-10, and every checkpoint above uses the full tanh formula.
      // The looser 1e-5 terminal tolerance documents the asymptotic limit.
      close(state.body.velocity[2], -terminal, 1e-5);
    });

    test("8c. wind advects a hovering vehicle to the air velocity", () => {
      const config = makeHex({ quadraticDrag: [0, 0, 0] });
      const commands = mix(config, hoverWrench(config));
      const wind: Environment = { windVelocity: [3, 0, 0] };
      let state = makeState(config, {
        position: [0, 0, 5],
        motorSpeeds: commands,
      });
      const checkpoints = new Map([
        [2000, 2],
        [6000, 6],
        [12000, 12],
      ]);

      for (let index = 1; index <= 12000; index++) {
        state = stepMultirotor(state, config, commands, 1e-3, wind);
        const time = checkpoints.get(index);
        if (time !== undefined) {
          close(state.body.velocity[0], 3 * (1 - Math.exp(-(0.25 * time) / 1.5)), 1e-6);
          close(state.body.velocity[2], 0, 1e-6);
        }
      }
    });
  });

  test("9. motor lag follows a first-order response to a mixer step", () => {
    const config = makeHex();
    const timeConstant = 0.05;
    const hover = hoverMotorSpeeds(config);
    // 1.2× hover *thrust*: speed is sqrt(1.2) × hover speed (~542.5 rad/s).
    const mixed = mixDetailed(config, [1.2 * config.mass * 9.81, 0, 0, 0]);
    expect(mixed.saturated).toBe(false);

    let state = makeState(config, { motorSpeeds: hover });
    const checkpoints = new Map([
      [500, timeConstant],
      [2500, 5 * timeConstant],
      [5000, 10 * timeConstant],
    ]);

    for (let index = 1; index <= 5000; index++) {
      state = stepMultirotor(state, config, mixed.motorSpeeds, 1e-4);
      const time = checkpoints.get(index);
      if (time !== undefined) {
        for (const [motorIndex, command] of mixed.motorSpeeds.entries()) {
          const initial = at(hover, motorIndex);
          close(
            at(state.motorSpeeds, motorIndex),
            command + (initial - command) * Math.exp(-time / timeConstant),
            1e-6,
          );
        }
      }
      if (index === 500) {
        const command = at(mixed.motorSpeeds, 0);
        const initial = at(hover, 0);
        const fraction = (initial - at(state.motorSpeeds, 0)) / (initial - command);
        close(fraction, 0.6321205588285577, 1e-6);
      }
    }
  });

  test("10. saturation sheds yaw first and leaves roll authority", () => {
    const config = makeHex();
    const wrench: Wrench = [config.mass * 9.81, 100, 0, 100];
    const mixed = mixDetailed(config, wrench);
    expect(mixed.saturated).toBe(true);
    close(mixed.achieved[0], config.mass * 9.81, 1e-9);
    close(mixed.achieved[3], 0, 1e-9);
    expect(mixed.achieved[1]).toBeGreaterThan(0);
    for (const speed of mixed.motorSpeeds) {
      expect(speed).toBeGreaterThanOrEqual(0);
      expect(speed).toBeLessThanOrEqual(1000);
    }

    const derivative = multirotorDerivatives(
      makeState(config, { motorSpeeds: mixed.motorSpeeds }),
      config,
      mixed.motorSpeeds,
    );
    close(derivative.body.angularVelocity[2], 0, 1e-9);
    expect(derivative.body.angularVelocity[0]).toBeGreaterThan(0);
    vecClose(derivative.body.velocity, [0, 0, 0], 1e-9);
  });

  test.skip("11. hex motor-out authority (no motor-failure model)", () => {
    // There is no fault flag on MotorConfig, no per-motor command override in
    // the dynamics, and the mixer assumes every rotor is available. Re-enable
    // when a motor-failure model exists; the expected check is that a hex with
    // one dead rotor still produces bounded yaw/roll authority and descends.
  });

  test("12. is deterministic, pure and limit-respecting", () => {
    const config = makeHex();
    const wrench: Wrench = [config.mass * 9.81, 0.03, -0.02, 0.01];
    const commands = mix(config, wrench);
    const initial = makeState(config, {
      position: [0, 0, 5],
      motorSpeeds: commands,
    });

    const runOnce = (): MultirotorState =>
      stepN(initial, config, commands, 5000, 1e-3);
    expect(runOnce()).toEqual(runOnce());
    expect(mix(config, wrench)).toEqual(mix(config, wrench));

    const configBefore = JSON.stringify(config);
    const wrenchBefore = JSON.stringify(wrench);
    const initialBefore = JSON.stringify(initial);

    let state = initial;
    let maxNormError = 0;
    for (let index = 0; index < 5000; index++) {
      state = stepMultirotor(state, config, commands, 1e-3);
      maxNormError = Math.max(
        maxNormError,
        Math.abs(quatNorm(state.body.orientation) - 1),
      );
      for (const [motorIndex, speed] of state.motorSpeeds.entries()) {
        const motor = at(config.motors, motorIndex);
        expect(Number.isFinite(speed)).toBe(true);
        expect(speed).toBeGreaterThanOrEqual(motor.minOmega);
        expect(speed).toBeLessThanOrEqual(motor.maxOmega);
      }
    }
    expect(maxNormError).toBeLessThan(1e-12);
    expect(JSON.stringify(config)).toBe(configBefore);
    expect(JSON.stringify(wrench)).toBe(wrenchBefore);
    expect(JSON.stringify(initial)).toBe(initialBefore);
    console.log(`  max quaternion norm error: ${maxNormError.toExponential(3)}`);
  });

  test("12b. quaternion sign does not change the physical trajectory", () => {
    const config = makeHex();
    const commands = mix(config, hoverWrench(config));
    const q = quatFromAxisAngle([0.3, -0.4, 0.5], 1.1);
    const base = makeState(config, {
      velocity: [0.2, 0, 0],
      orientation: q,
      angularVelocity: [0.1, -0.05, 0.02],
      motorSpeeds: commands,
    });
    const negated: MultirotorState = {
      ...base,
      body: {
        ...base.body,
        orientation: [-q[0], -q[1], -q[2], -q[3]] as Quat,
      },
    };

    const a = stepMultirotor(base, config, commands, 1e-3);
    const b = stepMultirotor(negated, config, commands, 1e-3);
    vecClose(a.body.position, b.body.position, 1e-12);
    vecClose(a.body.velocity, b.body.velocity, 1e-12);
    vecClose(a.body.angularVelocity, b.body.angularVelocity, 1e-12);
    expect(
      Math.abs(quatDot(a.body.orientation, b.body.orientation)),
    ).toBeGreaterThan(1 - 1e-12);
  });

  test("13a. mixer achieved wrench equals the dynamics motor wrench", () => {
    for (const config of [makeQuad(), makeHex(), makeOcto()]) {
      const wrench: Wrench = [config.mass * 9.81, 0.04, -0.02, 0.01];
      const result = mixDetailed(config, wrench);
      vecClose(result.achieved, wrench, 1e-9);

      const state = makeState(config, { motorSpeeds: result.motorSpeeds });
      const motorOnly = motorWrench(state, config);
      close(motorOnly.forceBody[2], result.achieved[0], 1e-12);
      vecClose(
        motorOnly.torqueBody,
        [result.achieved[1], result.achieved[2], result.achieved[3]],
        1e-12,
      );
      close(totalThrust(state, config), result.achieved[0], 1e-12);

      const recomputed = config.motors.map((motor, index) => {
        const omega = at(result.motorSpeeds, index);
        return motor.kT * omega * omega;
      });
      vecClose(result.motorThrusts, recomputed, 1e-12);
    }
  });

  test("13b. a yaw-rotated scenario follows the rotated trajectory", () => {
    const config = makeHex();
    const wind: Environment = { windVelocity: [1.5, -0.5, 0.2] };
    const wrench: Wrench = [config.mass * 9.81, 0.02, -0.01, 0.005];
    const commands = mix(config, wrench);
    const yaw = quatFromAxisAngle([0, 0, 1], 0.7);
    const baseOrientation = quatFromAxisAngle([0.2, 0.1, 0.9], 0.4);

    const base = makeState(config, {
      position: [1, 2, 5],
      velocity: [0.3, -0.2, 0.1],
      orientation: baseOrientation,
      angularVelocity: [0.01, -0.02, 0.03],
      motorSpeeds: commands,
    });
    const rotated = makeState(config, {
      position: quatRotateVector(yaw, base.body.position),
      velocity: quatRotateVector(yaw, base.body.velocity),
      orientation: quatMultiply(yaw, base.body.orientation),
      angularVelocity: base.body.angularVelocity,
      motorSpeeds: commands,
    });
    const rotatedWind: Environment = {
      windVelocity: quatRotateVector(yaw, wind.windVelocity),
    };

    // Keep the covariance test non-degenerate: drag and the ω × Iω term must
    // both be active, otherwise a frame-sign bug could hide behind zeros.
    expect(base.body.velocity.some((value) => value !== 0)).toBe(true);
    expect(base.body.angularVelocity.some((value) => value !== 0)).toBe(true);
    expect(wind.windVelocity.some((value) => value !== 0)).toBe(true);

    const baseEnd = stepN(base, config, commands, 1000, 1e-3, wind);
    const rotatedEnd = stepN(rotated, config, commands, 1000, 1e-3, rotatedWind);

    vecClose(
      rotatedEnd.body.position,
      quatRotateVector(yaw, baseEnd.body.position),
      1e-9,
    );
    vecClose(
      rotatedEnd.body.velocity,
      quatRotateVector(yaw, baseEnd.body.velocity),
      1e-9,
    );
    vecClose(rotatedEnd.body.angularVelocity, baseEnd.body.angularVelocity, 1e-9);
    const expectedOrientation = quatMultiply(yaw, baseEnd.body.orientation);
    expect(
      Math.abs(quatDot(rotatedEnd.body.orientation, expectedOrientation)),
    ).toBeGreaterThan(1 - 1e-12);
  });

  test("13c. thrust follows body +z through the mixer", () => {
    const config = makeHex();
    const commands = mix(config, hoverWrench(config));
    const tilted = quatFromAxisAngle([1, 0, 0], Math.PI / 2);
    const state = makeState(config, {
      orientation: tilted,
      motorSpeeds: commands,
    });
    const derivative = multirotorDerivatives(state, config, commands);
    vecClose(derivative.body.velocity, [0, -9.81, -9.81], 1e-9);
    vecClose(derivative.body.angularVelocity, [0, 0, 0], 1e-12);
  });
});
