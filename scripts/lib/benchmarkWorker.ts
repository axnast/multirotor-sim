/** Worker body for scripts/benchmark.ts: one held-out full-length rollout. */

import { parentPort } from "node:worker_threads";

import { makeHex } from "../../src/core/multirotor/config";
import type { ControllerParams } from "../../src/core/control/types";
import { rollout, type CostKey } from "../../src/core/tuning/cost";

interface Job {
  readonly id: number;
  readonly params: ControllerParams;
  readonly key: CostKey;
  readonly seed: number;
  readonly steps: number;
}

const config = makeHex({ groundPlane: true });
parentPort?.on("message", (job: Job) => {
  const metrics = rollout(config, job.params, job.seed, job.key, job.steps);
  parentPort?.postMessage({ id: job.id, metrics });
});
