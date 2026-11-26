/**
 * Held-out benchmark (long job; run ONCE per final candidate, never from an
 * agent shell): `npm run benchmark`. Uses seeds 1001-1020 only, full 120 s
 * noisy scenarios, all baselines under identical conditions. Outputs
 * out/benchmark.csv and a printed markdown table with the pre-registered
 * headline first. No tuning or re-selection happens here.
 */

import { mkdirSync, writeFileSync, readFileSync } from "node:fs";
import { Worker } from "node:worker_threads";

import { defaultControllerParams } from "../src/core/control/controller";
import type { ControllerParams } from "../src/core/control/types";
import { makeHex } from "../src/core/multirotor/config";
import {
  FULL_STEPS,
  rollout,
  type CostKey,
  type RolloutMetrics,
} from "../src/core/tuning/cost";
import { withGains } from "../src/core/tuning/params";
import { nextUint32, seedRng, type RngState } from "../src/core/tuning/rng";

const HELD_OUT = Array.from({ length: 20 }, (_, index) => 1001 + index);
const SCENARIOS: readonly CostKey[] = [
  "light", "moderate", "gusty", "strong", "constant5", "mass20",
];
const STOCHASTIC = SCENARIOS.slice(0, 5);
const WORKERS = 8;
const RESAMPLES = 10_000;
const BOOTSTRAP_SEED = 424_242;
const SECONDARY = ["step", "square"] as const;

const config = makeHex({ groundPlane: true });
const base = defaultControllerParams(config);
const zn = JSON.parse(readFileSync("data/znGains.json", "utf8")) as {
  ultimate: Pick<ControllerParams, "position" | "attitude" | "rate">;
  reaction: Pick<ControllerParams, "position" | "attitude" | "rate">;
};
const tuned = JSON.parse(readFileSync("data/tunedGains.json", "utf8")) as {
  vector: readonly number[];
};

const VARIANTS = [
  { label: "M3 gate", params: { ...base, position: { ...base.position, integralDwell: 0 } } },
  { label: "M5 default", params: base },
  { label: "gate off (reference)", params: { ...base, position: { ...base.position, integralSeparation: 1e9 } } },
  { label: "ZN ultimate-gain", params: { ...base, ...zn.ultimate } },
  { label: "ZN reaction-curve", params: { ...base, ...zn.reaction } },
  { label: "autotuned", params: withGains(base, tuned.vector) },
] as const;

type MetricsMap = Map<string, RolloutMetrics>;
const results: MetricsMap[] = VARIANTS.map(() => new Map());
const keyOf = (variant: number, key: CostKey, seed: number) => `${variant}|${key}|${seed}`;

interface Job {
  readonly id: number;
  readonly variant: number;
  readonly key: CostKey;
  readonly seed: number;
}

function runJobs(jobs: readonly Job[]): Promise<void> {
  const queue = [...jobs];
  let active = 0;
  let done = 0;
  return new Promise<void>((resolve, reject) => {
    const dispatch = () => {
      while (active < WORKERS && queue.length > 0) {
        const job = queue.shift()!;
        active++;
        const worker = new Worker(new URL("./lib/benchmarkWorker.ts", import.meta.url), {
          workerData: {},
          execArgv: ["--import", "tsx"],
        });
        worker.on("message", (message: { id: number; metrics: RolloutMetrics }) => {
          const found = jobs.find((candidate) => candidate.id === message.id)!;
          results[found.variant]!.set(keyOf(found.variant, found.key, found.seed), message.metrics);
          void worker.terminate();
          active--;
          done++;
          if (done === jobs.length) resolve();
          else dispatch();
        });
        worker.on("error", reject);
        worker.postMessage({
          id: job.id,
          params: VARIANTS[job.variant]!.params,
          key: job.key,
          seed: job.seed,
        });
      }
    };
    dispatch();
  });
}

function mean(values: readonly number[]): number {
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

function percentile(sorted: readonly number[], fraction: number): number {
  return sorted[Math.min(sorted.length - 1, Math.floor(fraction * sorted.length))]!;
}

function bootstrapIndices(rng: RngState): number[] {
  return Array.from({ length: HELD_OUT.length }, () => nextUint32(rng).value % HELD_OUT.length);
}

function pairedCI(differences: readonly number[]): readonly [number, number] {
  let rng = seedRng(BOOTSTRAP_SEED);
  const samples: number[] = [];
  for (let resample = 0; resample < RESAMPLES; resample++) {
    const indices = bootstrapIndices(rng);
    rng = nextUint32(rng).state;
    samples.push(mean(indices.map((index) => differences[index]!)));
  }
  samples.sort((a, b) => a - b);
  return [percentile(samples, 0.025), percentile(samples, 0.975)];
}

function headline(tunedIndex: number, baseIndex: number): {
  ratio: number;
  ci: readonly [number, number];
} {
  const rms = (variant: number, key: CostKey) =>
    HELD_OUT.map((seed) => results[variant]!.get(keyOf(variant, key, seed))!.rms);
  let rng = seedRng(BOOTSTRAP_SEED + 1);
  const compute = (indices: readonly number[]) => {
    let product = 1;
    for (const key of STOCHASTIC) {
      product *= mean(indices.map((index) => rms(tunedIndex, key)[index]!)) /
        mean(indices.map((index) => rms(baseIndex, key)[index]!));
    }
    return product ** (1 / STOCHASTIC.length);
  };
  const identity = HELD_OUT.map((_, index) => index);
  const ratio = compute(identity);
  const samples: number[] = [];
  for (let resample = 0; resample < RESAMPLES; resample++) {
    samples.push(compute(bootstrapIndices(rng)));
    rng = nextUint32(rng).state;
  }
  samples.sort((a, b) => a - b);
  return { ratio, ci: [percentile(samples, 0.025), percentile(samples, 0.975)] };
}

async function main(): Promise<void> {
  mkdirSync("out", { recursive: true });
  const jobs: Job[] = [];
  let id = 0;
  for (let variant = 0; variant < VARIANTS.length; variant++) {
    for (const key of [...SCENARIOS, ...SECONDARY]) {
      for (const seed of HELD_OUT) jobs.push({ id: id++, variant, key, seed });
    }
  }
  console.log(`evaluating ${jobs.length} held-out rollouts on seeds 1001-1020 ...`);
  await runJobs(jobs);

  const tunedIndex = VARIANTS.length - 1;
  const csv = ["variant,scenario,seed,rms_m,rms_20_60,rms_90_120,overshoot_m,settling_s,ise,max_tilt_rad,saturation_steps,jitter,contact_steps"];
  for (let variant = 0; variant < VARIANTS.length; variant++) {
    for (const key of [...SCENARIOS, ...SECONDARY]) {
      for (const seed of HELD_OUT) {
        const m = results[variant]!.get(keyOf(variant, key, seed))!;
        csv.push([
          VARIANTS[variant]!.label, key, seed,
          m.rms.toFixed(6), m.earlyRms.toFixed(6), m.lateRms.toFixed(6),
          m.overshoot.toFixed(6), m.settling.toFixed(4), m.ise.toFixed(6),
          m.maxTilt.toFixed(6), m.saturationSteps, m.jitter.toFixed(6), m.contactSteps,
        ].join(","));
      }
    }
  }
  writeFileSync("out/benchmark.csv", `${csv.join("\n")}\n`);

  console.log("\n## Headline: geometric mean tuned/baseline RMS, five stochastic scenarios");
  console.log("| baseline | ratio | bootstrap 95% CI |");
  console.log("|---|---:|---:|");
  for (let baseIndex = 0; baseIndex < VARIANTS.length - 1; baseIndex++) {
    const { ratio, ci } = headline(tunedIndex, baseIndex);
    console.log(`| ${VARIANTS[baseIndex]!.label} | ${ratio.toFixed(4)} | [${ci[0].toFixed(4)}, ${ci[1].toFixed(4)}] |`);
  }

  console.log("\n## Paired per-seed differences (tuned - baseline, m)");
  console.log("| baseline | scenario | mean | SE | bootstrap CI | improved fraction |");
  console.log("|---|---|---:|---:|---:|---:|");
  for (let baseIndex = 0; baseIndex < VARIANTS.length - 1; baseIndex++) {
    for (const key of SCENARIOS) {
      const differences = HELD_OUT.map((seed) =>
        results[tunedIndex]!.get(keyOf(tunedIndex, key, seed))!.rms -
        results[baseIndex]!.get(keyOf(baseIndex, key, seed))!.rms,
      );
      const average = mean(differences);
      const se = Math.sqrt(differences.reduce((sum, value) => sum + (value - average) ** 2, 0) / (differences.length - 1)) / Math.sqrt(differences.length);
      const ci = pairedCI(differences);
      const improved = differences.filter((value) => value < 0).length / differences.length;
      console.log(
        `| ${VARIANTS[baseIndex]!.label} | ${key} | ${average.toFixed(4)} | ${se.toFixed(4)} | ` +
          `[${ci[0].toFixed(4)}, ${ci[1].toFixed(4)}] | ${(improved * 100).toFixed(0)}% |`,
      );
    }
  }

  console.log("\n## Detail: mean RMS [20,120] / [20,60] / [90,120] (m)");
  console.log("| scenario | " + VARIANTS.map((variant) => variant.label).join(" | ") + " |");
  console.log("|---|" + VARIANTS.map(() => "---:").join("|") + "|");
  for (const key of SCENARIOS) {
    const cells = VARIANTS.map((_, variant) => {
      const rows = HELD_OUT.map((seed) => results[variant]!.get(keyOf(variant, key, seed))!);
      return `${mean(rows.map((row) => row.rms)).toFixed(3)} / ${mean(rows.map((row) => row.earlyRms)).toFixed(3)} / ${mean(rows.map((row) => row.lateRms)).toFixed(3)}`;
    });
    console.log(`| ${key} | ${cells.join(" | ")} |`);
  }

  console.log("\n## Secondary: step and square (mean over seeds)");
  console.log("| variant | scenario | overshoot | settling | ISE | max tilt | saturation steps | jitter | contacts |");
  console.log("|---|---|---:|---:|---:|---:|---:|---:|---:|");
  for (let variant = 0; variant < VARIANTS.length; variant++) {
    for (const key of SECONDARY) {
      const rows = HELD_OUT.map((seed) => results[variant]!.get(keyOf(variant, key, seed))!);
      console.log(
        `| ${VARIANTS[variant]!.label} | ${key} | ${mean(rows.map((row) => row.overshoot)).toFixed(4)} | ` +
          `${mean(rows.map((row) => row.settling)).toFixed(3)} | ${mean(rows.map((row) => row.ise)).toFixed(4)} | ` +
          `${mean(rows.map((row) => row.maxTilt)).toFixed(4)} | ${mean(rows.map((row) => row.saturationSteps)).toFixed(0)} | ` +
          `${mean(rows.map((row) => row.jitter)).toFixed(3)} | ${mean(rows.map((row) => row.contactSteps)).toFixed(0)} |`,
      );
    }
  }

  console.log("\n## Robustness (reported, not tuned): tuned-gain degradation vs nominal step RMS");
  const tunedParams = VARIANTS[tunedIndex]!.params;
  const nominal = mean(HELD_OUT.slice(0, 5).map((seed) =>
    rollout(config, tunedParams, seed, "step", FULL_STEPS).rms,
  ));
  const mismatches = [
    { label: "mass +10%", override: { massScale: 1.1 } },
    { label: "mass -10%", override: { massScale: 0.9 } },
    { label: "kT -10%", override: { kTScale: 0.9 } },
    { label: "kT +10%", override: { kTScale: 1.1 } },
    { label: "tau +30%", override: { tauScale: 1.3 } },
  ] as const;
  for (const mismatch of mismatches) {
    const rms = mean(HELD_OUT.slice(0, 5).map((seed) =>
      rollout(config, tunedParams, seed, "step", FULL_STEPS, mismatch.override).rms,
    ));
    console.log(`| ${mismatch.label} | ${(rms / nominal).toFixed(3)}x |`);
  }
  console.log(`\nwrote out/benchmark.csv; bootstrap resamples = ${RESAMPLES}, paired by seed`);
}

main().catch((error: unknown) => {
  console.error(error);
  process.exitCode = 1;
});
