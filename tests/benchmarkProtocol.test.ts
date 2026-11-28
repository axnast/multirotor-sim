import { describe, expect, it } from "vitest";

import {
  BENCHMARK_SCENARIOS,
  DETERMINISTIC_SCENARIOS,
  HEADLINE_SCENARIOS,
} from "../src/core/tuning/protocol";

describe("benchmark protocol scenario lists", () => {
  it("pins the pre-held-out headline to the four gust scenarios", () => {
    expect([...HEADLINE_SCENARIOS]).toEqual([
      "light",
      "moderate",
      "gusty",
      "strong",
    ]);
    expect(HEADLINE_SCENARIOS).not.toContain("constant5");
    expect(HEADLINE_SCENARIOS).not.toContain("mass20");
  });

  it("keeps all six scenarios and the two deterministic secondary rows", () => {
    expect([...BENCHMARK_SCENARIOS]).toEqual([
      "light",
      "moderate",
      "gusty",
      "strong",
      "constant5",
      "mass20",
    ]);
    expect([...DETERMINISTIC_SCENARIOS]).toEqual(["constant5", "mass20"]);
  });
});
