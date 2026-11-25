import { describe, expect, it } from "vitest";

import { positionErrorM } from "../src/viewer/sim/metrics";

describe("shared position-error metric", () => {
  it("uses the 3D Euclidean distance to the setpoint", () => {
    expect(positionErrorM([1, 2, 3], [4, 6, 3])).toBeCloseTo(5, 12);
    expect(positionErrorM([0, 0, 0], [0, 0, 5])).toBeCloseTo(5, 12);
    expect(positionErrorM([1, 2, 3], [1, 2, 3])).toBe(0);
  });

  it("is symmetric", () => {
    const a = [1.25, -2.5, 3.75] as const;
    const b = [-0.5, 1.75, 2] as const;
    expect(positionErrorM(a, b)).toBeCloseTo(positionErrorM(b, a), 12);
  });

  it("matches the explicit component formula", () => {
    const position = [0.3, -1.1, 4.9] as const;
    const target = [3.2, 0.7, 5.1] as const;
    const expected = Math.hypot(
      position[0] - target[0],
      position[1] - target[1],
      position[2] - target[2],
    );
    expect(positionErrorM(position, target)).toBeCloseTo(expected, 12);
  });

  it("gives consistent rounded values for the same sample and both precisions", () => {
    // The status line uses 2 decimals, the HUD uses 3; both consume this helper.
    const position = [2.13, -0.4, 6.02] as const;
    const target = [0.5, 1.7, 4.1] as const;
    const exact = positionErrorM(position, target);
    expect(positionErrorM(position, target)).toBe(exact);
    expect(Number(exact.toFixed(2))).toBeCloseTo(Number(exact.toFixed(3)), 2);
  });
});
