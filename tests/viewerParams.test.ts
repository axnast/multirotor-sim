import { describe, expect, it } from "vitest";

import {
  defaultControllerParams,
  validateControllerParams,
} from "../src/core/control/controller";
import { makeHex } from "../src/core/multirotor/config";
import {
  applyParamValue,
  clampSliderValue,
  isKiSlider,
  PARAM_SLIDERS,
  type ParamKey,
} from "../src/viewer/ui/paramSpecs";

const config = makeHex();
const defaults = defaultControllerParams(config);

describe("viewer gain slider specs", () => {
  it("contains every documented control and no duplicates", () => {
    const ids = PARAM_SLIDERS.map((spec) => spec.id);
    expect(new Set(ids).size).toBe(ids.length);
    expect(ids.length).toBe(18);
    expect(ids).toContain("position.integralSeparation");
    expect(ids).toContain("rate.maxAngularAccelerationZ");
  });

  it("has defaults inside every slider range", () => {
    for (const spec of PARAM_SLIDERS) {
      const value = spec.get(defaults);
      expect(value).toBeGreaterThanOrEqual(spec.min);
      expect(value).toBeLessThanOrEqual(spec.max);
      expect(Number.isFinite(value)).toBe(true);
    }
  });

  it("applies a value immutably and validates the result", () => {
    const next = applyParamValue(defaults, "position.kp", 0.5);
    expect(next).not.toBe(defaults);
    expect(next.position.gains.kp).toBeCloseTo(0.5, 12);
    expect(defaults.position.gains.kp).toBeCloseTo(0.25, 12);
    expect(() => validateControllerParams(next)).not.toThrow();
  });

  it("clamps out-of-range values", () => {
    const low = applyParamValue(defaults, "rate.kp", -5);
    const high = applyParamValue(defaults, "rate.kp", 99);
    expect(low.rate.gains.kp).toBe(0.5);
    expect(high.rate.gains.kp).toBe(10);
    const spec = PARAM_SLIDERS.find((candidate) => candidate.id === "rate.kp")!;
    expect(clampSliderValue(spec, Number.NaN)).toBe(Number.NaN);
    expect(() => applyParamValue(defaults, "rate.kp", Number.NaN)).toThrow(RangeError);
  });

  it("stores max tilt in radians", () => {
    const next = applyParamValue(defaults, "position.maxTilt", 45);
    expect(next.position.maxTilt).toBeCloseTo(Math.PI / 4, 12);
  });

  it("updates roll/pitch together and yaw separately", () => {
    const xy = applyParamValue(defaults, "rate.maxAngularAccelerationXY", 30);
    expect(xy.rate.maxAngularAcceleration).toEqual([30, 30, defaults.rate.maxAngularAcceleration[2]]);
    const z = applyParamValue(defaults, "rate.maxAngularAccelerationZ", 1);
    expect(z.rate.maxAngularAcceleration[0]).toBe(defaults.rate.maxAngularAcceleration[0]);
    expect(z.rate.maxAngularAcceleration[2]).toBe(1);
  });

  it("identifies the two ki sliders for bumpless changes", () => {
    expect(PARAM_SLIDERS.filter((spec) => isKiSlider(spec.id)).map((spec) => spec.id as ParamKey)).toEqual([
      "position.ki",
      "rate.ki",
    ]);
  });

  it("keeps all extreme slider settings valid", () => {
    for (const spec of PARAM_SLIDERS) {
      for (const value of [spec.min, spec.max]) {
        expect(() => validateControllerParams(applyParamValue(defaults, spec.id, value))).not.toThrow();
      }
    }
  });
});
