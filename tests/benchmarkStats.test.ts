import { describe, expect, it } from "vitest";

import {
  fractionImproved,
  geometricMeanRatio,
  mean,
  pairedCI,
  percentile,
} from "../src/core/tuning/stats";

describe("benchmark bootstrap statistics", () => {
  it("computes means and nearest-rank percentiles", () => {
    expect(mean([1, 2, 3])).toBe(2);
    expect(percentile([1, 2, 3, 4], 0.5)).toBe(3);
    expect(percentile([1, 2, 3, 4], 0)).toBe(1);
    expect(percentile([1, 2, 3, 4], 1)).toBe(4);
  });

  it("has a known paired CI when every difference is identical", () => {
    const ci = pairedCI([0.25, 0.25, 0.25, 0.25], 2_000, 7);
    expect(ci).toEqual([0.25, 0.25]);
  });

  it("brackets the sample mean for a non-degenerate paired sample", () => {
    const differences = [1, 2, 3, 4];
    const ci = pairedCI(differences, 2_000, 11);
    expect(ci[0]).toBeLessThanOrEqual(mean(differences));
    expect(ci[1]).toBeGreaterThanOrEqual(mean(differences));
    expect(ci[0]).toBeGreaterThanOrEqual(1);
    expect(ci[1]).toBeLessThanOrEqual(4);
  });

  it("counts the improved fraction exactly", () => {
    expect(fractionImproved([-2, -1, 0, 1])).toBe(0.5);
    expect(fractionImproved([1, 2])).toBe(0);
  });

  it("returns a degenerate headline CI when every seed has the same ratio", () => {
    // Two scenarios with constant ratios 2 and 3 -> sqrt(2*3) = sqrt(6).
    const result = geometricMeanRatio(
      [[2, 2], [3, 3]],
      [[1, 1], [1, 1]],
      2_000,
      13,
    );
    expect(result.ratio).toBeCloseTo(Math.sqrt(6), 12);
    expect(result.ci[0]).toBeCloseTo(Math.sqrt(6), 12);
    expect(result.ci[1]).toBeCloseTo(Math.sqrt(6), 12);
  });

  it("handles a one-scenario headline exactly", () => {
    const result = geometricMeanRatio([[4, 4]], [[1, 1]], 1_000, 17);
    expect(result.ratio).toBeCloseTo(4, 12);
    expect(result.ci).toEqual([4, 4]);
  });

  it("rejects invalid scenario/seed shapes", () => {
    expect(() => geometricMeanRatio([], [])).toThrow();
    expect(() => geometricMeanRatio([[1]], [[1], [1]])).toThrow();
    expect(() => geometricMeanRatio([[1, 2]], [[1]])).toThrow();
    expect(() => geometricMeanRatio([[]], [[]])).toThrow();
  });
});
