import { describe, expect, it } from "vitest";

import {
  defaultControllerParams,
  initialControllerState,
} from "../src/core/control/controller";
import { QUAT_IDENTITY } from "../src/core/math/quat";
import { makeHex } from "../src/core/multirotor/config";
import { simulateStep, type CoreState } from "../src/core/simulation/loop";
import { DT } from "../src/core/tuning/cost";

const config = makeHex({ groundPlane: false });
const params = defaultControllerParams(config);
const plant = { config, params };

/**
 * Feed a known 1 m/step position ramp through `simulateStep` and record the
 * exact measurement the controller receives. With latency N the controller
 * must see `max(0, step - N)`.
 */
function observedPositions(latency: number, steps: number): number[] {
  let state: CoreState = {
    multirotor: {
      body: {
        position: [0, 0, 5],
        velocity: [0, 0, 0],
        orientation: QUAT_IDENTITY,
        angularVelocity: [0, 0, 0],
      },
      motorSpeeds: config.motors.map(() => 0),
    },
    controller: initialControllerState(),
    disturbance: null,
    disturbanceParams: null,
    diagnostics: null,
    trueConfig: config,
  };
  const seen: number[] = [];
  for (let step = 0; step < steps; step++) {
    const truth: CoreState = {
      ...state,
      multirotor: {
        ...state.multirotor,
        body: {
          ...state.multirotor.body,
          position: [step, 0, 5],
          velocity: [0, 0, 0],
          angularVelocity: [0, 0, 0],
        },
      },
    };
    const result = simulateStep(
      truth,
      plant,
      { position: [0, 0, 5], yaw: 0 },
      DT,
      null,
      latency,
    );
    seen.push(result.controllerMeasurement.position[0]!);
    state = {
      ...truth,
      multirotor: result.state,
      controller: result.controller,
      trueConfig: result.trueConfig,
      ...(result.measurementHistory === undefined
        ? {}
        : { measurementHistory: result.measurementHistory }),
    };
  }
  return seen;
}

describe("measurement latency", () => {
  it("delivers the sensor measurement exactly N steps late", () => {
    const steps = 24;
    for (const latency of [0, 1, 3, 5, 10]) {
      const expected = Array.from({ length: steps }, (_, step) =>
        Math.max(0, step - latency),
      );
      expect(observedPositions(latency, steps)).toEqual(expected);
    }
  });

  it("is opt-in: zero latency returns the current measurement", () => {
    const seen = observedPositions(0, 6);
    expect(seen).toEqual([0, 1, 2, 3, 4, 5]);
  });
});
