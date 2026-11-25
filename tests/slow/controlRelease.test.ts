import { describe, expect, it } from "vitest";

import { disturbanceParamsFor } from "../../src/core/disturbance/scenario";
import {
  constantWind,
  plant,
  run,
  withRelease,
} from "../helpers/releaseRun";

describe("persistent-error integral release (slow)", () => {
  it("releases a constant-wind offset with sensor noise and trims it below 0.05 m by 120 s", () => {
    const result = run(withRelease(plant(), {}), 120_000, constantWind("moderate", 1, true));
    expect(result.releaseFraction).toBeGreaterThan(0);
    expect(result.meanTrueError).toBeLessThan(0.05);
    expect(result.integral[0]).toBeLessThan(-100);
  }, 300_000);

  it("recovers from a centred +20% payload that previously froze on the ground", () => {
    const params = disturbanceParamsFor("calm", 7, {
      payload: { atTime: 0, payload: { mass: 0.3, position: [0, 0, 0] } },
    });
    const result = run(withRelease(plant(), {}), 120_000, params);
    expect(result.releaseFraction).toBeGreaterThan(0);
    expect(result.finalTrueError).toBeLessThan(0.1);
    expect(result.meanTrueError).toBeLessThan(0.1);
  }, 300_000);

  it("recovers from the centred 10 g payload that previously froze at 0.2 m", () => {
    const params = disturbanceParamsFor("calm", 7, {
      payload: { atTime: 0, payload: { mass: 0.01, position: [0, 0, 0] } },
    });
    const result = run(withRelease(plant(), {}), 120_000, params);
    expect(result.finalTrueError).toBeLessThan(0.1);
  }, 300_000);
});
