/**
 * Score default + both ZN baselines once each through the pre-registered cost
 * on dev seeds 1-5. No tuning and no held-out seeds. Output is printed only.
 */

import { readFileSync } from "node:fs";

import { defaultControllerParams } from "../src/core/control/controller";
import type { ControllerParams } from "../src/core/control/types";
import { makeHex } from "../src/core/multirotor/config";
import {
  COST_KEYS,
  DEV_SEEDS,
  FULL_STEPS,
  createBaselineTable,
  createCostSuite,
} from "../src/core/tuning/cost";

const config = makeHex({ groundPlane: true });
const base = defaultControllerParams(config);
const zn = JSON.parse(readFileSync("data/znGains.json", "utf8")) as {
  ultimate: Pick<ControllerParams, "position" | "attitude" | "rate">;
  reactionPID: Pick<ControllerParams, "position" | "attitude" | "rate">;
  reactionPI: Pick<ControllerParams, "position" | "attitude" | "rate">;
};
const baselines = createBaselineTable([DEV_SEEDS], config, base, FULL_STEPS);
const suite = createCostSuite(config, base, FULL_STEPS, baselines);

const variants = [
  { label: "M5 default", params: base },
  { label: "ZN ultimate-gain", params: { ...base, ...zn.ultimate } },
  { label: "ZN reaction-curve PID", params: { ...base, ...zn.reactionPID } },
  { label: "ZN reaction-curve PI", params: { ...base, ...zn.reactionPI } },
];

for (const variant of variants) {
  const result = suite.evaluate(variant.params, DEV_SEEDS);
  console.log(`\n${variant.label} | total=${result.total.toFixed(6)}`);
  console.log("term | ratio | reason");
  for (const term of result.terms) {
    console.log(
      `${term.key.padEnd(13)} | ${term.normalized.toFixed(6)} | ${term.reason ?? "ok"}`,
    );
  }
  const failures = result.terms.filter((term) => term.reason !== null);
  console.log(`hard failures: ${failures.length === 0 ? "none" : failures.map((term) => `${term.key}: ${term.reason}`).join("; ")}`);
  console.log(`terms: ${COST_KEYS.length}`);
}
