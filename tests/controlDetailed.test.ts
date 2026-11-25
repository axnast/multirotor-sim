import { describe, expect, it } from "vitest";

import {
  controllerStep,
  controllerStepDetailed,
  defaultControllerParams,
  initialControllerState,
} from "../src/core/control/controller";
import type {
  ControllerState,
  Measurement,
  Setpoint,
} from "../src/core/control/types";
import { QUAT_IDENTITY } from "../src/core/math/quat";
import { makeHex } from "../src/core/multirotor/config";
import { stepMultirotor, type MultirotorState } from "../src/core/multirotor/dynamics";
import { mix } from "../src/core/multirotor/mixer";

function runSteps(
  state: MultirotorState,
  controller: ControllerState,
  setpoint: Setpoint,
  steps: number,
): { state: MultirotorState; controller: ControllerState } {
  const config = makeHex();
  const params = defaultControllerParams(config);
  let current = state;
  let control = controller;
  for (let step = 0; step < steps; step++) {
    const measurement: Measurement = {
      position: current.body.position,
      velocity: current.body.velocity,
      orientation: current.body.orientation,
      angularVelocity: current.body.angularVelocity,
    };
    const command = controllerStep(control, measurement, setpoint, params, 1e-3);
    control = command.state;
    current = stepMultirotor(current, config, mix(config, command.wrench), 1e-3);
  }
  return { state: current, controller: control };
}

describe("controllerStepDetailed", () => {
  const config = makeHex();
  const params = defaultControllerParams(config);
  const setpoint: Setpoint = { position: [5, 0, 5], yaw: 0 };

  it("returns the same state and wrench as controllerStep", () => {
    const progressed = runSteps(
      {
        body: {
          position: [0, 0, 5],
          velocity: [0, 0, 0],
          orientation: QUAT_IDENTITY,
          angularVelocity: [0, 0, 0],
        },
        motorSpeeds: config.motors.map(() => 0),
      },
      initialControllerState(),
      setpoint,
      300,
    );
    const measurement: Measurement = {
      position: progressed.state.body.position,
      velocity: progressed.state.body.velocity,
      orientation: progressed.state.body.orientation,
      angularVelocity: progressed.state.body.angularVelocity,
    };

    const detailed = controllerStepDetailed(
      progressed.controller,
      measurement,
      setpoint,
      params,
      1e-3,
    );
    const stable = controllerStep(
      progressed.controller,
      measurement,
      setpoint,
      params,
      1e-3,
    );

    expect(detailed.state).toEqual(stable.state);
    expect(detailed.wrench).toEqual(stable.wrench);
    expect(Object.keys(stable).sort()).toEqual(["state", "wrench"]);
  });

  it("exposes finite targets from the same cascade computation", () => {
    const measurement: Measurement = {
      position: [0, 0, 5],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    };
    const detailed = controllerStepDetailed(
      initialControllerState(),
      measurement,
      setpoint,
      params,
      1e-3,
    );

    expect(detailed.desiredRate.every(Number.isFinite)).toBe(true);
    expect(detailed.desiredAcceleration.every(Number.isFinite)).toBe(true);
    expect(Number.isFinite(detailed.desiredThrust)).toBe(true);
    const q = detailed.desiredOrientation;
    expect(Math.hypot(q[0], q[1], q[2], q[3])).toBeCloseTo(1, 12);
    // A 5 m x-step tilts body +z toward +x, i.e. a positive pitch rate.
    expect(detailed.desiredRate[1]).toBeGreaterThan(0);
  });

  it("reports zero desired roll/pitch at a settled hover setpoint", () => {
    const detailed = controllerStepDetailed(
      initialControllerState(),
      {
        position: [0, 0, 5],
        velocity: [0, 0, 0],
        orientation: QUAT_IDENTITY,
        angularVelocity: [0, 0, 0],
      },
      { position: [0, 0, 5], yaw: 0 },
      params,
      1e-3,
    );

    expect(detailed.desiredRate[0]).toBeCloseTo(0, 12);
    expect(detailed.desiredRate[1]).toBeCloseTo(0, 12);
    expect(detailed.desiredThrust).toBeCloseTo(
      config.mass * Math.abs(config.gravity[2]),
      9,
    );
  });
});
