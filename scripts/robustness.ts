/**
 * Exploratory robustness grid on DEV seeds 1-5 only. Post-hoc, not
 * pre-registered, not used for any decision about the candidate. Conditions:
 * measurement latency 5/10 ms, sensor sigmas x2, mean wind from +y / 45 deg,
 * and the existing kT/tau and mass mismatches. The flag rule is pre-specified
 * in docs/tuning.md and is applied after the grid is computed.
 *
 * No held-out seeds, no tuning, no gains changes.
 */

import { readFileSync } from "node:fs";
import { Worker } from "node:worker_threads";

import { defaultControllerParams } from "../src/core/control/controller";
import type { ControllerParams } from "../src/core/control/types";
import { makeHex } from "../src/core/multirotor/config";
import {
  DEV_SEEDS,
  type CostKey,
  type RolloutMetrics,
  type RolloutOptions,
} from "../src/core/tuning/cost";
import { withGains } from "../src/core/tuning/params";
import type { PlantOverride } from "../src/core/simulation/loop";

const WORKERS = 8;
const KEYS = ["step", "light", "gusty", "strong"] as const;
const WIND_KEYS = ["light", "gusty", "strong"] as const;

interface Condition {
  readonly id: string;
  readonly label: string;
  readonly options: RolloutOptions;
  readonly override: PlantOverride | null;
}

const CONDITIONS: readonly Condition[] = [
  { id: "nominal", label: "nominal", options: {}, override: null },
  { id: "lat5", label: "latency 5 ms", options: { measurementLatencySteps: 5 }, override: null },
  { id: "lat10", label: "latency 10 ms", options: { measurementLatencySteps: 10 }, override: null },
  { id: "lat50", label: "latency 50 ms", options: { measurementLatencySteps: 50 }, override: null },
  { id: "lat100", label: "latency 100 ms", options: { measurementLatencySteps: 100 }, override: null },
  { id: "sensor2", label: "sensor sigma x2", options: { sensorScale: 2 }, override: null },
  { id: "wind90", label: "wind from +y (90 deg)", options: { windDirectionDeg: 90 }, override: null },
  { id: "wind45", label: "wind from 45 deg", options: { windDirectionDeg: 45 }, override: null },
  { id: "ktau", label: "kT -10%, tau +30%", options: {}, override: { kTScale: 0.9, tauScale: 1.3 } },
  { id: "mass", label: "mass +10%", options: {}, override: { massScale: 1.1 } },
];

const config = makeHex({ groundPlane: true });
const base = defaultControllerParams(config);
const tuned = withGains(
  base,
  (JSON.parse(readFileSync("data/tunedGains.json", "utf8")) as { vector: readonly number[] }).vector,
);

const CONTROLLERS = [
  { id: "m3", label: "M3 gate", params: { ...base, position: { ...base.position, integralDwell: 0 } } },
  { id: "default", label: "M5 default", params: base },
  { id: "tuned", label: "autotuned", params: tuned },
] as const;

interface Job {
  readonly id: number;
  readonly condition: string;
  readonly controller: string;
  readonly key: CostKey;
  readonly seed: number;
  readonly params: ControllerParams;
  readonly options: RolloutOptions;
  readonly override: PlantOverride | null;
}

const jobs: Job[] = [];
let id = 0;
for (const condition of CONDITIONS) {
  for (const controller of CONTROLLERS) {
    for (const key of KEYS) {
      for (const seed of DEV_SEEDS) {
        jobs.push({
          id: id++,
          condition: condition.id,
          controller: controller.id,
          key,
          seed,
          params: controller.params,
          options: condition.options,
          override: condition.override,
        });
      }
    }
  }
}

const results = new Map<number, RolloutMetrics>();
const total = jobs.length;
let done = 0;
console.log(`running ${total} dev-seed exploratory rollouts on ${WORKERS} workers ...`);

await new Promise<void>((resolve, reject) => {
  const queue = [...jobs];
  let active = 0;
  const dispatch = (): void => {
    while (active < WORKERS && queue.length > 0) {
      const job = queue.shift()!;
      active++;
      const worker = new Worker(new URL("./lib/exploreWorker.ts", import.meta.url), {
        workerData: { ...job },
        execArgv: ["--import", "tsx"],
      });
      worker.on("message", (message: { id: number; metrics: RolloutMetrics }) => {
        results.set(message.id, message.metrics);
        done++;
        if (done % 40 === 0 || done === total) console.log(`  ${done}/${total}`);
        void worker.terminate();
        active--;
        if (done === total) resolve();
        else dispatch();
      });
      worker.on("error", reject);
    }
  };
  dispatch();
});

function rowsFor(condition: string, controller: string, key: CostKey): RolloutMetrics[] {
  return jobs
    .filter((job) => job.condition === condition && job.controller === controller && job.key === key)
    .map((job) => results.get(job.id)!);
}

const mean = (values: readonly number[]): number =>
  values.reduce((sum, value) => sum + value, 0) / values.length;
const max = (values: readonly number[]): number => Math.max(...values);

interface Summary {
  readonly rms: Readonly<Record<(typeof WIND_KEYS)[number], number>>;
  readonly overshoot: number;
  readonly settling: number;
  readonly maxTiltDeg: number;
  readonly maxJitter: number;
  readonly maxPeakMotorRate: number;
  readonly sustainedSaturation: number;
  readonly failures: readonly string[];
}

function summarize(condition: string, controller: string): Summary {
  const step = rowsFor(condition, controller, "step");
  const all = KEYS.flatMap((key) => rowsFor(condition, controller, key));
  const failures = [...new Set(all.map((row) => row.failure).filter((value): value is string => value !== null))];
  return {
    rms: {
      light: mean(rowsFor(condition, controller, "light").map((row) => row.rms)),
      gusty: mean(rowsFor(condition, controller, "gusty").map((row) => row.rms)),
      strong: mean(rowsFor(condition, controller, "strong").map((row) => row.rms)),
    },
    overshoot: mean(step.map((row) => row.overshoot)),
    settling: mean(step.map((row) => row.settling)),
    maxTiltDeg: (max(all.map((row) => row.maxTilt)) * 180) / Math.PI,
    maxJitter: max(all.map((row) => row.jitter)),
    maxPeakMotorRate: max(all.map((row) => row.peakMotorRate)),
    sustainedSaturation: max(all.map((row) => row.sustainedSaturation)),
    failures,
  };
}

const summaries = new Map<string, Summary>();
for (const condition of CONDITIONS) {
  for (const controller of CONTROLLERS) {
    summaries.set(`${condition.id}|${controller.id}`, summarize(condition.id, controller.id));
  }
}

const nominalDefault = summaries.get("nominal|default")!;
const nominalTuned = summaries.get("nominal|tuned")!;

console.log("\n## Exploratory robustness (dev seeds 1-5)");
console.log(
  "Post-hoc, not pre-registered, not used for any decision about the candidate. " +
    "Latency is measurement (sensor-to-controller) latency only; actuator latency is zero.",
);
console.log(
  "Max tilt, jitter and motor rate are maxima over the step/light/gusty/strong rollouts and seeds " +
    "(jitter rad/s, motor rate rad/s^2).",
);
console.log("\n| condition | controller | light RMS | gusty RMS | strong RMS | step overshoot | step settling | max tilt | max jitter | max motor rate | sustained sat | failures |");
console.log("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|");
for (const condition of CONDITIONS) {
  for (const controller of CONTROLLERS) {
    const summary = summaries.get(`${condition.id}|${controller.id}`)!;
    console.log(
      `| ${condition.label} | ${controller.label} | ${summary.rms.light.toFixed(3)} | ` +
        `${summary.rms.gusty.toFixed(3)} | ${summary.rms.strong.toFixed(3)} | ` +
        `${summary.overshoot.toFixed(3)} | ${summary.settling.toFixed(2)} | ` +
        `${summary.maxTiltDeg.toFixed(2)} deg | ${summary.maxJitter.toFixed(3)} | ` +
        `${summary.maxPeakMotorRate.toFixed(0)} | ${summary.sustainedSaturation.toFixed(4)} | ` +
        `${summary.failures.length === 0 ? "none" : `${summary.failures.length}: ${summary.failures.join("; ")}`} |`,
    );
  }
}

console.log("\n### Flag rule check (tuned vs default)");
console.log("Flag if the tuned controller hard-fails/diverges, or a tuned/default RMS ratio exceeds 2x its nominal ratio.");
console.log("\n| condition | scenario | default RMS | tuned RMS | ratio | nominal ratio | ratio/x nominal | flag |");
console.log("|---|---|---:|---:|---:|---:|---:|---|");
let anyFlag = false;
for (const condition of CONDITIONS) {
  const def = summaries.get(`${condition.id}|default`)!;
  const tun = summaries.get(`${condition.id}|tuned`)!;
  for (const key of WIND_KEYS) {
    const ratio = tun.rms[key] / def.rms[key];
    const nominalRatio = nominalTuned.rms[key] / nominalDefault.rms[key];
    const relative = ratio / nominalRatio;
    const flagged = ratio > 2 * nominalRatio || tun.failures.length > 0;
    if (flagged) anyFlag = true;
    if (condition.id !== "nominal" || flagged) {
      console.log(
        `| ${condition.label} | ${key} | ${def.rms[key].toFixed(3)} | ${tun.rms[key].toFixed(3)} | ` +
          `${ratio.toFixed(3)} | ${nominalRatio.toFixed(3)} | ${relative.toFixed(2)}x | ${flagged ? "FLAG" : "-"} |`,
      );
    }
  }
  if (tun.failures.length > 0) {
    console.log(`| ${condition.label} | any | - | - | - | - | - | FLAG: ${tun.failures.join("; ")} |`);
  }
}
console.log(`\noverall: ${anyFlag ? "at least one condition flagged" : "no condition flagged"}`);
