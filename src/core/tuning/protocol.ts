/**
 * Benchmark scenario lists fixed by the M6 protocol and its pre-held-out
 * amendment. Exported as data so a test can assert the headline cannot change
 * silently.
 */

import type { CostKey } from "./cost";

/** All six scored scenarios, in the documented order. */
export const BENCHMARK_SCENARIOS = [
  "light",
  "moderate",
  "gusty",
  "strong",
  "constant5",
  "mass20",
] as const satisfies readonly CostKey[];

/**
 * Headline scenarios (amendment before the held-out run): the four stochastic
 * gust scenarios. constant5 and mass20 are reported separately because they
 * are deterministic apart from sensor noise.
 */
export const HEADLINE_SCENARIOS = [
  "light",
  "moderate",
  "gusty",
  "strong",
] as const satisfies readonly CostKey[];

/** Deterministic scenarios reported as separate secondary rows. */
export const DETERMINISTIC_SCENARIOS = ["constant5", "mass20"] as const satisfies readonly CostKey[];
