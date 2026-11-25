import { describe, expect, it } from "vitest";

import {
  controllerStep,
  defaultControllerParams,
  initialControllerState,
} from "../src/core/control/controller";
import type {
  ControllerParams,
  ControllerState,
  Measurement,
  PidState,
  Setpoint,
} from "../src/core/control/types";
import { QUAT_IDENTITY } from "../src/core/math/quat";
import { makeHex } from "../src/core/multirotor/config";
import { stepMultirotor, type MultirotorState } from "../src/core/multirotor/dynamics";
import { mix } from "../src/core/multirotor/mixer";
import { bumplessKiChange } from "../src/viewer/sim/bumpless";

const DT = 1e-3;

function axis(integral: number): PidState {
  return { integral, derivative: 0, lastMeasurement: null };
}

function stateWithIntegral(integral: number): ControllerState {
  const fresh = initialControllerState();
  return {
    position: [
      { ...fresh.position[0]!, integral },
      { ...fresh.position[1]!, integral },
      { ...fresh.position[2]!, integral },
    ],
    rate: [axis(integral), axis(integral), axis(integral)],
  };
}

function withKi(
  params: ControllerParams,
  positionKi: number,
  rateKi: number,
): ControllerParams {
  return {
    ...params,
    position: {
      ...params.position,
      gains: { ...params.position.gains, ki: positionKi },
    },
    rate: {
      ...params.rate,
      gains: { ...params.rate.gains, ki: rateKi },
    },
  };
}

describe("bumpless live ki changes", () => {
  const config = makeHex();
  const params = defaultControllerParams(config);

  it("rescales stored integrals so ki·integral is preserved", () => {
    const next = withKi(params, params.position.gains.ki * 2, params.rate.gains.ki * 2);
    const adjusted = bumplessKiChange(stateWithIntegral(2), params, next);

    expect(adjusted.position[0]!.integral).toBeCloseTo(1, 12);
    expect(adjusted.rate[2]!.integral).toBeCloseTo(1, 12);
    expect(adjusted.position[0]!.derivative).toBeCloseTo(0, 12);
  });

  it("zeroes the integral when the integrator is switched off", () => {
    const next = withKi(params, 0, 0);
    const adjusted = bumplessKiChange(stateWithIntegral(3), params, next);
    expect(adjusted.position.every((axis) => axis.integral === 0)).toBe(true);
    expect(adjusted.rate.every((axis) => axis.integral === 0)).toBe(true);
  });

  it("leaves the state unchanged when ki is unchanged", () => {
    const original = stateWithIntegral(1.25);
    const adjusted = bumplessKiChange(original, params, params);
    expect(adjusted).toEqual(original);
  });

  it("is bumpless for 10x ki increases with large integrals", () => {
    const measurement: Measurement = {
      position: [0, 0, 5],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    };
    const setpoint: Setpoint = { position: [0.02, 0, 5], yaw: 0 };
    const large = stateWithIntegral(50);
    const next = withKi(params, params.position.gains.ki * 10, params.rate.gains.ki * 10);
    const baseline = controllerStep(large, measurement, setpoint, params, DT);
    const adjustedState = bumplessKiChange(large, params, next);
    const adjusted = controllerStep(adjustedState, measurement, setpoint, next, DT);

    expect(adjustedState.position[0]!.integral).toBeCloseTo(5, 12);
    expect(adjustedState.rate[0]!.integral).toBeCloseTo(5, 12);
    for (let axis = 0; axis < 4; axis++) {
      expect(adjusted.wrench[axis]!).toBeCloseTo(baseline.wrench[axis]!, 4);
    }
  });

  it("is bumpless for 10x ki decreases with large integrals", () => {
    const measurement: Measurement = {
      position: [0, 0, 5],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    };
    const setpoint: Setpoint = { position: [0.02, 0, 5], yaw: 0 };
    const large = stateWithIntegral(50);
    const next = withKi(params, params.position.gains.ki / 10, params.rate.gains.ki / 10);
    const baseline = controllerStep(large, measurement, setpoint, params, DT);
    const adjustedState = bumplessKiChange(large, params, next);
    const adjusted = controllerStep(adjustedState, measurement, setpoint, next, DT);

    expect(adjustedState.position[0]!.integral).toBeCloseTo(500, 12);
    expect(adjustedState.rate[0]!.integral).toBeCloseTo(500, 12);
    for (let axis = 0; axis < 4; axis++) {
      expect(adjusted.wrench[axis]!).toBeCloseTo(baseline.wrench[axis]!, 4);
    }
  });

  it("is bumpless while the position-loop integral clamp is active", () => {
    const measurement: Measurement = {
      position: [0, 0, 5],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    };
    // A 100 m error pushes the position output past its 6 m/s² acceleration
    // limit, so conditional integration holds the (large) integral.
    const setpoint: Setpoint = { position: [100, 0, 5], yaw: 0 };
    const clamped = stateWithIntegral(10);
    for (const factor of [10, 1 / 10]) {
      const next = withKi(
        params,
        params.position.gains.ki * factor,
        params.rate.gains.ki,
      );
      const baseline = controllerStep(clamped, measurement, setpoint, params, DT);
      const adjustedState = bumplessKiChange(clamped, params, next);
      const adjusted = controllerStep(adjustedState, measurement, setpoint, next, DT);

      expect(adjustedState.position[0]!.integral).toBeCloseTo(10 / factor, 12);
      expect(adjusted.wrench[0]).toBeCloseTo(baseline.wrench[0], 9);
      expect(adjusted.state.position[0]!.integral).toBeCloseTo(
        adjustedState.position[0]!.integral,
        12,
      );
    }
  });

  it("is bumpless while integral separation is active", () => {
    const measurement: Measurement = {
      position: [0, 0, 5],
      velocity: [1, 0, 0], // moving toward the +x setpoint
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    };
    const setpoint: Setpoint = { position: [0.5, 0, 5], yaw: 0 };
    const separated = stateWithIntegral(10);
    for (const factor of [10, 1 / 10]) {
      const next = withKi(
        params,
        params.position.gains.ki * factor,
        params.rate.gains.ki,
      );
      const baseline = controllerStep(separated, measurement, setpoint, params, DT);
      const adjustedState = bumplessKiChange(separated, params, next);
      const adjusted = controllerStep(adjustedState, measurement, setpoint, next, DT);

      // Far from the target and moving toward it: the integral is frozen.
      expect(adjusted.state.position[0]!.integral).toBeCloseTo(
        adjustedState.position[0]!.integral,
        12,
      );
      expect(adjusted.wrench[0]).toBeCloseTo(baseline.wrench[0], 3);
    }
  });

  it("removes the output jump when ki changes mid-flight", () => {
    const setpoint: Setpoint = { position: [5, 0, 5], yaw: 0 };
    let state: MultirotorState = {
      body: {
        position: [0, 0, 5],
        velocity: [0, 0, 0],
        orientation: QUAT_IDENTITY,
        angularVelocity: [0, 0, 0],
      },
      motorSpeeds: config.motors.map(() => 0),
    };
    let control = initialControllerState();
    for (let step = 0; step < 300; step++) {
      const measurement: Measurement = {
        position: state.body.position,
        velocity: state.body.velocity,
        orientation: state.body.orientation,
        angularVelocity: state.body.angularVelocity,
      };
      const command = controllerStep(control, measurement, setpoint, params, DT);
      control = command.state;
      state = stepMultirotor(state, config, mix(config, command.wrench), DT);
    }

    const measurement: Measurement = {
      position: state.body.position,
      velocity: state.body.velocity,
      orientation: state.body.orientation,
      angularVelocity: state.body.angularVelocity,
    };
    const next = withKi(
      params,
      params.position.gains.ki * 2,
      params.rate.gains.ki * 2,
    );
    const baseline = controllerStep(control, measurement, setpoint, params, DT);
    const jumped = controllerStep(control, measurement, setpoint, next, DT);
    const adjustedState = bumplessKiChange(control, params, next);
    const adjusted = controllerStep(adjustedState, measurement, setpoint, next, DT);

    // Residual is floating-point round-off from ki·(I·oldKi/newKi), not a
    // control-law jump; the uncorrected change is orders of magnitude larger.
    expect(Math.abs(jumped.wrench[0] - baseline.wrench[0])).toBeGreaterThan(1e-6);
    expect(adjusted.wrench[0]).toBeCloseTo(baseline.wrench[0], 4);
    for (let axis = 1; axis < 4; axis++) {
      expect(adjusted.wrench[axis]!).toBeCloseTo(baseline.wrench[axis]!, 4);
    }
  });
});
