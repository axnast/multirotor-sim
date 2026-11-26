/** Worker body for scripts/autotune.ts: one full-length cost evaluation. */

import { parentPort, workerData } from "node:worker_threads";

import { defaultControllerParams } from "../../src/core/control/controller";
import { makeHex } from "../../src/core/multirotor/config";
import {
  createCostSuite,
  type BaselineTable,
} from "../../src/core/tuning/cost";
import { withGains } from "../../src/core/tuning/params";

interface Job {
  readonly id: number;
  readonly vector: readonly number[];
  readonly seeds: readonly number[];
}

const data = workerData as { baselineTable: BaselineTable; steps: number };
const config = makeHex({ groundPlane: true });
const params = defaultControllerParams(config);
const suite = createCostSuite(config, params, data.steps, data.baselineTable);

parentPort?.on("message", (job: Job) => {
  const result = suite.evaluate(withGains(params, job.vector), job.seeds);
  const failed = result.terms.some(
    (term) =>
      term.reason !== null &&
      !(term.key === "mass20" && term.reason === "ground contact in flight"),
  );
  parentPort?.postMessage({
    id: job.id,
    total: result.total,
    failed,
    terms: result.terms.map((term) => ({
      key: term.key,
      normalized: term.normalized,
      reason: term.reason,
    })),
  });
});
