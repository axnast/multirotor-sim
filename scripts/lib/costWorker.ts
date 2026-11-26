/** Worker body for scripts/autotune.ts: one full-length cost evaluation. */

import { parentPort, workerData } from "node:worker_threads";

import { defaultControllerParams } from "../../src/core/control/controller";
import { makeHex } from "../../src/core/multirotor/config";
import {
  createCostSuite,
  FULL_STEPS,
  type BaselineTable,
} from "../../src/core/tuning/cost";
import { withGains } from "../../src/core/tuning/params";

interface Job {
  readonly id: number;
  readonly vector: readonly number[];
  readonly seeds: readonly number[];
}

const data = workerData as { baselineTable: BaselineTable };
const config = makeHex({ groundPlane: true });
const params = defaultControllerParams(config);
const suite = createCostSuite(config, params, FULL_STEPS, data.baselineTable);

parentPort?.on("message", (job: Job) => {
  const result = suite.evaluate(withGains(params, job.vector), job.seeds);
  parentPort?.postMessage({
    id: job.id,
    total: result.total,
    terms: result.terms.map((term) => ({
      key: term.key,
      normalized: term.normalized,
      reason: term.reason,
    })),
  });
});
