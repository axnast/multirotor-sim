/**
 * Worker body for scripts/robustness.ts: one full-length rollout under an
 * opt-in exploratory condition (latency, sensor scale, wind direction or plant
 * mismatch). Uses the same `rollout` and `simulateStep` as the tuning harness.
 */

import { parentPort, workerData } from "node:worker_threads";

import type { ControllerParams } from "../../src/core/control/types";
import { makeHex } from "../../src/core/multirotor/config";
import {
  FULL_STEPS,
  rollout,
  type CostKey,
  type RolloutMetrics,
  type RolloutOptions,
} from "../../src/core/tuning/cost";
import type { PlantOverride } from "../../src/core/simulation/loop";

interface ExploreJob {
  readonly id: number;
  readonly params: ControllerParams;
  readonly seed: number;
  readonly key: CostKey;
  readonly options: RolloutOptions;
  readonly override: PlantOverride | null;
}

const job = workerData as ExploreJob;
const config = makeHex({ groundPlane: true });
const metrics: RolloutMetrics = rollout(
  config,
  job.params,
  job.seed,
  job.key,
  FULL_STEPS,
  job.override,
  job.options,
);
parentPort?.postMessage({ id: job.id, metrics });
