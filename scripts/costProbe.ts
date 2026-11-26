/**
 * One full-length default-gain cost evaluation under the pre-registered noisy
 * conditions. Not a tuning run: used to verify the 1.000 normalization, get
 * the per-term breakdown and measure real seconds per evaluation.
 */

import { defaultControllerParams } from "../src/core/control/controller";
import { makeHex } from "../src/core/multirotor/config";
import { createCostSuite, DEV_SEEDS } from "../src/core/tuning/cost";

const config = makeHex({ groundPlane: true });
const params = defaultControllerParams(config);
const suite = createCostSuite(config, params);
const started = performance.now();
const result = suite.evaluate(params, DEV_SEEDS);
const baselineAndCandidateSeconds = (performance.now() - started) / 1000;
const secondStart = performance.now();
suite.evaluate(params, DEV_SEEDS);
const seconds = (performance.now() - secondStart) / 1000;

console.log(`default total = ${result.total.toFixed(12)} (seeds ${result.seeds.join(",")})`);
console.log("term | weight | ratio | weighted ln");
for (const term of result.terms) {
  console.log(
    `${term.key.padEnd(13)} | ${term.weight.toFixed(4)} | ${term.normalized.toFixed(6)} | ` +
      `${(term.weight * Math.log(term.normalized)).toFixed(6)}` +
      (term.reason === null ? "" : ` | ${term.reason}`),
  );
}
const dominant = [...result.terms].sort(
  (a, b) => b.weight * Math.log(b.normalized) - a.weight * Math.log(a.normalized),
)[0]!;
console.log(`dominant weighted ln contribution: ${dominant.key}`);
console.log(`wall time (baseline + one evaluation): ${baselineAndCandidateSeconds.toFixed(2)} s`);
console.log(`wall time for one cached-baseline evaluation: ${seconds.toFixed(2)} s`);
