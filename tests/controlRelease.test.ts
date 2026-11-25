import { describe, expect, it } from "vitest";

import { defaultControllerParams } from "../src/core/control/controller";
import { positionStep } from "../src/core/control/position";
import type { ControllerState, Setpoint } from "../src/core/control/types";
import { disturbanceParamsFor } from "../src/core/disturbance/scenario";
import { makeHex } from "../src/core/multirotor/config";
import {
  constantWind,
  DESIGN,
  hoverMeasurement,
  latchedState,
  plant,
  run,
  withRelease,
} from "./helpers/releaseRun";

const stepSetpoint: Setpoint = { position: [5, 0, 5], yaw: 0 };

describe("persistent-error integral release (fast)", () => {
  it("dwell=0 is the M3 baseline: the release never arms under constant wind", () => {
    const result = run(
      withRelease(plant(), { integralDwell: 0 }),
      60_000,
      constantWind("calm", 1, true),
    );
    expect(result.releaseFraction).toBe(0);
    expect(result.meanTrueError).toBeGreaterThan(2);
    expect(result.integral[0]).toBeLessThan(0);
  }, 120_000);

  it("the setpoint-age guard clears the latch on a fresh command", () => {
    const config = makeHex({ groundPlane: true });
    const params = defaultControllerParams(config);
    const state: ControllerState = latchedState();
    const next = positionStep(state.position, hoverMeasurement, stepSetpoint, params, 0.001);
    expect(next.state[0]!.released).toBe(false);
    expect(next.state[0]!.setpointAge).toBe(0);
  });

  it("stays dormant during a 5 m position step", () => {
    const result = run(withRelease(plant(), {}), 25_000, disturbanceParamsFor("calm", 3), [5, 0, 5]);
    expect(result.releaseFraction).toBe(0);
    expect(result.finalTrueError).toBeLessThan(0.01);
  }, 60_000);

  it("has a documented default parameter set", () => {
    expect(DESIGN).toEqual({
      integralDwell: 5,
      integralTrimGain: 3,
      integralReleaseGuard: 2,
    });
  });
});
