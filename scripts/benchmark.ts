/**
 * Held-out benchmark (long job; run ONCE per final candidate, never from an
 * agent shell): `npm run benchmark`. Uses seeds 1001-1020 only, full 120 s
 * noisy scenarios, all baselines under identical conditions. Outputs
 * out/benchmark.csv and a printed markdown table with the pre-registered
 * headline first. No tuning or re-selection happens here.
 *
 * `--dev --steps N` is a pre-flight dry run on dev seeds 1-5 with a short
 * horizon. It writes out/benchmark-dev.csv, never touches the held-out lock,
 * and is the only mode where `--steps` is allowed.
 *
 * One-shot guard: a completed held-out run writes out/benchmark.lock with the
 * SHA-256 of data/tunedGains.json and the git commit. A later held-out run
 * refuses to start unless `--allow-rerun` is passed.
 */

import { execFileSync } from "node:child_process";
import { createHash } from "node:crypto";
import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { Worker } from "node:worker_threads";

import { defaultControllerParams } from "../src/core/control/controller";
import type { ControllerParams } from "../src/core/control/types";
import { makeHex } from "../src/core/multirotor/config";
import {
  DEV_SEEDS,
  FULL_STEPS,
  rollout,
  type CostKey,
  type RolloutMetrics,
} from "../src/core/tuning/cost";
import { withGains } from "../src/core/tuning/params";
import {
  BENCHMARK_SCENARIOS,
  DETERMINISTIC_SCENARIOS,
  HEADLINE_SCENARIOS,
} from "../src/core/tuning/protocol";
import {
  fractionImproved,
  geometricMeanRatio,
  mean,
  pairedCI,
} from "../src/core/tuning/stats";

const HELD_OUT = Array.from({ length: 20 }, (_, index) => 1001 + index);
const WORKERS = 8;
const RESAMPLES = 10_000;
const BOOTSTRAP_SEED = 424_242;
const SECONDARY = ["step", "square"] as const;
const LOCK_PATH = "out/benchmark.lock";

interface Cli {
  readonly dev: boolean;
  readonly allowRerun: boolean;
  readonly steps: number;
}

function parseArgs(argv: readonly string[]): Cli {
  const dev = argv.includes("--dev");
  const allowRerun = argv.includes("--allow-rerun");
  let steps = FULL_STEPS;
  const stepsIndex = argv.indexOf("--steps");
  if (stepsIndex >= 0) {
    const raw = argv[stepsIndex + 1];
    if (raw === undefined) throw new Error("--steps needs an integer value");
    steps = Number(raw);
    if (!Number.isSafeInteger(steps) || steps <= 0) {
      throw new Error("--steps must be a positive integer");
    }
  }
  if (!dev && steps !== FULL_STEPS) {
    throw new Error("--steps is dev-only; held-out runs must use the full 120 s horizon");
  }
  return { dev, allowRerun, steps };
}

const cli = parseArgs(process.argv.slice(2));
const SEEDS: readonly number[] = cli.dev ? [...DEV_SEEDS] : HELD_OUT;
const OUTPUT_CSV = cli.dev ? "out/benchmark-dev.csv" : "out/benchmark.csv";

function sha256File(path: string): string {
  return createHash("sha256").update(readFileSync(path)).digest("hex");
}

function currentCommit(): string {
  try {
    return execFileSync("git", ["rev-parse", "HEAD"], { encoding: "utf8" }).trim();
  } catch {
    return "unknown";
  }
}

const config = makeHex({ groundPlane: true });
const base = defaultControllerParams(config);
const zn = JSON.parse(readFileSync("data/znGains.json", "utf8")) as {
  ultimate: Pick<ControllerParams, "position" | "attitude" | "rate">;
  reactionPID: Pick<ControllerParams, "position" | "attitude" | "rate">;
  reactionPI: Pick<ControllerParams, "position" | "attitude" | "rate">;
  reactionPI_rateOnly: Pick<ControllerParams, "position" | "attitude" | "rate">;
};
const tuned = JSON.parse(readFileSync("data/tunedGains.json", "utf8")) as {
  vector: readonly number[];
};

const VARIANTS = [
  { label: "M3 gate", params: { ...base, position: { ...base.position, integralDwell: 0 } } },
  { label: "M5 default", params: base },
  { label: "gate off (reference)", params: { ...base, position: { ...base.position, integralSeparation: 1e9 } } },
  { label: "ZN ultimate-gain", params: { ...base, ...zn.ultimate } },
  { label: "ZN reaction-curve PID", params: { ...base, ...zn.reactionPID } },
  { label: "ZN reaction-curve PI", params: { ...base, ...zn.reactionPI } },
  { label: "ZN reaction-curve PI (rate only)", params: { ...base, ...zn.reactionPI_rateOnly } },
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
          steps: cli.steps,
        });
      }
    };
    dispatch();
  });
}

async function main(): Promise<void> {
  mkdirSync("out", { recursive: true });
  if (!cli.dev && existsSync(LOCK_PATH) && !cli.allowRerun) {
    throw new Error(
      `${LOCK_PATH} exists: refusing to run held-out seeds again without --allow-rerun`,
    );
  }

  console.log(
    `mode=${cli.dev ? "dev pre-flight" : "HELD-OUT"}; ` +
      `seeds=${SEEDS[0]}-${SEEDS[SEEDS.length - 1]} (${SEEDS.length}); steps=${cli.steps}`,
  );
  if (!cli.dev) {
    console.log(
      `gains sha256=${sha256File("data/tunedGains.json")}; git commit=${currentCommit()}`,
    );
  }

  const jobs: Job[] = [];
  let id = 0;
  for (let variant = 0; variant < VARIANTS.length; variant++) {
    for (const key of [...BENCHMARK_SCENARIOS, ...SECONDARY]) {
      for (const seed of SEEDS) jobs.push({ id: id++, variant, key, seed });
    }
  }
  console.log(`evaluating ${jobs.length} rollouts ...`);
  await runJobs(jobs);

  const tunedIndex = VARIANTS.length - 1;
  // A baseline that hard-fails has no valid RMS: it is printed as such and
  // excluded from ratios and the geometric-mean headline entirely.
  const valid = VARIANTS.map((_, variant) =>
    [...BENCHMARK_SCENARIOS, ...SECONDARY].every((key) =>
      SEEDS.every((seed) => results[variant]!.get(keyOf(variant, key, seed))!.failure === null),
    ),
  );
  const INVALID_TEXT = "fails (no valid RMS)";
  const csv = ["variant,scenario,seed,rms_m,rms_20_60,rms_90_120,overshoot_m,settling_s,ise,max_tilt_rad,saturation_steps,jitter,contact_steps,failure"];
  for (let variant = 0; variant < VARIANTS.length; variant++) {
    for (const key of [...BENCHMARK_SCENARIOS, ...SECONDARY]) {
      for (const seed of SEEDS) {
        const m = results[variant]!.get(keyOf(variant, key, seed))!;
        csv.push([
          VARIANTS[variant]!.label, key, seed,
          m.rms.toFixed(6), m.earlyRms.toFixed(6), m.lateRms.toFixed(6),
          m.overshoot.toFixed(6), m.settling.toFixed(4), m.ise.toFixed(6),
          m.maxTilt.toFixed(6), m.saturationSteps, m.jitter.toFixed(6), m.contactSteps,
          m.failure ?? "",
        ].join(","));
      }
    }
  }
  writeFileSync(OUTPUT_CSV, `${csv.join("\n")}\n`);

  const rms = (variant: number, key: CostKey): number[] =>
    SEEDS.map((seed) => results[variant]!.get(keyOf(variant, key, seed))!.rms);
  const headlineFor = (baseIndex: number, keys: readonly CostKey[]) =>
    geometricMeanRatio(
      keys.map((key) => rms(tunedIndex, key)),
      keys.map((key) => rms(baseIndex, key)),
      RESAMPLES,
      BOOTSTRAP_SEED + 1,
    );

  console.log(
    "\n## Headline (pre-held-out amendment): geometric mean tuned/baseline RMS, " +
      "four stochastic gust scenarios (light, moderate, gusty, strong)",
  );
  console.log("| baseline | ratio | bootstrap 95% CI |");
  console.log("|---|---:|---:|");
  for (let baseIndex = 0; baseIndex < VARIANTS.length - 1; baseIndex++) {
    if (!valid[baseIndex]) {
      console.log(`| ${VARIANTS[baseIndex]!.label} | ${INVALID_TEXT} | excluded |`);
      continue;
    }
    const { ratio, ci } = headlineFor(baseIndex, HEADLINE_SCENARIOS);
    console.log(`| ${VARIANTS[baseIndex]!.label} | ${ratio.toFixed(4)} | [${ci[0].toFixed(4)}, ${ci[1].toFixed(4)}] |`);
  }

  console.log("\n## Secondary headline: all six scenarios (geometric mean tuned/baseline RMS)");
  console.log("| baseline | ratio | bootstrap 95% CI |");
  console.log("|---|---:|---:|");
  for (let baseIndex = 0; baseIndex < VARIANTS.length - 1; baseIndex++) {
    if (!valid[baseIndex]) {
      console.log(`| ${VARIANTS[baseIndex]!.label} | ${INVALID_TEXT} | excluded |`);
      continue;
    }
    const { ratio, ci } = headlineFor(baseIndex, BENCHMARK_SCENARIOS);
    console.log(`| ${VARIANTS[baseIndex]!.label} | ${ratio.toFixed(4)} | [${ci[0].toFixed(4)}, ${ci[1].toFixed(4)}] |`);
  }

  console.log("\n## Secondary per-scenario: constant5 and mass20");
  console.log(
    "These are deterministic apart from sensor noise and dominated by integrator trim speed; " +
      "they are reported separately, not in the headline.",
  );
  console.log("| baseline | scenario | tuned/default RMS ratio | ratio CI | mean paired diff (m) | paired diff CI | improved fraction |");
  console.log("|---|---|---:|---:|---:|---:|---:|");
  for (let baseIndex = 0; baseIndex < VARIANTS.length - 1; baseIndex++) {
    for (const key of DETERMINISTIC_SCENARIOS) {
      if (!valid[baseIndex]) {
        console.log(`| ${VARIANTS[baseIndex]!.label} | ${key} | ${INVALID_TEXT} | excluded | - | - | - |`);
        continue;
      }
      const tunedRms = rms(tunedIndex, key);
      const baseRms = rms(baseIndex, key);
      const ratio = geometricMeanRatio([tunedRms], [baseRms], RESAMPLES, BOOTSTRAP_SEED + 2);
      const differences = tunedRms.map((value, index) => value - baseRms[index]!);
      const diffCI = pairedCI(differences, RESAMPLES, BOOTSTRAP_SEED + 3);
      console.log(
        `| ${VARIANTS[baseIndex]!.label} | ${key} | ${ratio.ratio.toFixed(4)} | ` +
          `[${ratio.ci[0].toFixed(4)}, ${ratio.ci[1].toFixed(4)}] | ${mean(differences).toFixed(4)} | ` +
          `[${diffCI[0].toFixed(4)}, ${diffCI[1].toFixed(4)}] | ${(fractionImproved(differences) * 100).toFixed(0)}% |`,
      );
    }
  }

  console.log("\n## Paired per-seed differences (tuned - baseline, m)");
  console.log("| baseline | scenario | mean | SE | bootstrap CI | improved fraction |");
  console.log("|---|---|---:|---:|---:|---:|");
  for (let baseIndex = 0; baseIndex < VARIANTS.length - 1; baseIndex++) {
    if (!valid[baseIndex]) {
      console.log(`| ${VARIANTS[baseIndex]!.label} | all | ${INVALID_TEXT} | - | excluded | - |`);
      continue;
    }
    for (const key of BENCHMARK_SCENARIOS) {
      const differences = SEEDS.map((seed) =>
        results[tunedIndex]!.get(keyOf(tunedIndex, key, seed))!.rms -
        results[baseIndex]!.get(keyOf(baseIndex, key, seed))!.rms,
      );
      const average = mean(differences);
      const se = Math.sqrt(differences.reduce((sum, value) => sum + (value - average) ** 2, 0) / (differences.length - 1)) / Math.sqrt(differences.length);
      const ci = pairedCI(differences, RESAMPLES, BOOTSTRAP_SEED + 4);
      console.log(
        `| ${VARIANTS[baseIndex]!.label} | ${key} | ${average.toFixed(4)} | ${se.toFixed(4)} | ` +
          `[${ci[0].toFixed(4)}, ${ci[1].toFixed(4)}] | ${(fractionImproved(differences) * 100).toFixed(0)}% |`,
      );
    }
  }

  console.log("\n## Detail: mean RMS [20,120] / [20,60] / [90,120] (m)");
  console.log("| scenario | " + VARIANTS.map((variant) => variant.label).join(" | ") + " |");
  console.log("|---|" + VARIANTS.map(() => "---:").join("|") + "|");
  for (const key of BENCHMARK_SCENARIOS) {
    const cells = VARIANTS.map((_, variant) => {
      if (!valid[variant]) return INVALID_TEXT;
      const rows = SEEDS.map((seed) => results[variant]!.get(keyOf(variant, key, seed))!);
      return `${mean(rows.map((row) => row.rms)).toFixed(3)} / ${mean(rows.map((row) => row.earlyRms)).toFixed(3)} / ${mean(rows.map((row) => row.lateRms)).toFixed(3)}`;
    });
    console.log(`| ${key} | ${cells.join(" | ")} |`);
  }

  console.log("\n## Secondary: step and square (mean over seeds)");
  console.log("| variant | scenario | overshoot | settling | ISE | max tilt | saturation steps | jitter | contacts |");
  console.log("|---|---|---:|---:|---:|---:|---:|---:|---:|");
  for (let variant = 0; variant < VARIANTS.length; variant++) {
    for (const key of SECONDARY) {
      if (!valid[variant]) {
        console.log(`| ${VARIANTS[variant]!.label} | ${key} | ${INVALID_TEXT} | - | - | - | - | - | - |`);
        continue;
      }
      const rows = SEEDS.map((seed) => results[variant]!.get(keyOf(variant, key, seed))!);
      console.log(
        `| ${VARIANTS[variant]!.label} | ${key} | ${mean(rows.map((row) => row.overshoot)).toFixed(4)} | ` +
          `${mean(rows.map((row) => row.settling)).toFixed(3)} | ${mean(rows.map((row) => row.ise)).toFixed(4)} | ` +
          `${mean(rows.map((row) => row.maxTilt)).toFixed(4)} | ${mean(rows.map((row) => row.saturationSteps)).toFixed(0)} | ` +
          `${mean(rows.map((row) => row.jitter)).toFixed(3)} | ${mean(rows.map((row) => row.contactSteps)).toFixed(0)} |`,
      );
    }
  }

  console.log("\n## Robustness (reported, not tuned): tuned-gain degradation vs nominal step RMS");
  console.log("| mismatch | tuned RMS / nominal |");
  console.log("|---|---:|");
  const tunedParams = VARIANTS[tunedIndex]!.params;
  const robustnessSeeds = SEEDS.slice(0, 5);
  const nominal = mean(robustnessSeeds.map((seed) =>
    rollout(config, tunedParams, seed, "step", cli.steps).rms,
  ));
  const mismatches = [
    { label: "mass +10%", override: { massScale: 1.1 } },
    { label: "mass -10%", override: { massScale: 0.9 } },
    { label: "kT -10%", override: { kTScale: 0.9 } },
    { label: "kT +10%", override: { kTScale: 1.1 } },
    { label: "tau +30%", override: { tauScale: 1.3 } },
  ] as const;
  for (const mismatch of mismatches) {
    const value = mean(robustnessSeeds.map((seed) =>
      rollout(config, tunedParams, seed, "step", cli.steps, mismatch.override).rms,
    ));
    console.log(`| ${mismatch.label} | ${(value / nominal).toFixed(3)}x |`);
  }

  if (!cli.dev) {
    const lock = {
      version: 1,
      gainsSha256: sha256File("data/tunedGains.json"),
      gitCommit: currentCommit(),
      seeds: SEEDS,
      steps: cli.steps,
      headlineScenarios: HEADLINE_SCENARIOS,
      scenarios: BENCHMARK_SCENARIOS,
      output: OUTPUT_CSV,
      bootstrap: { resamples: RESAMPLES, seed: BOOTSTRAP_SEED },
      completedAt: new Date().toISOString(),
    };
    writeFileSync(LOCK_PATH, `${JSON.stringify(lock, null, 2)}\n`);
    console.log(`\nwrote ${LOCK_PATH}; a rerun now requires --allow-rerun`);
  }
  console.log(`\nwrote ${OUTPUT_CSV}; bootstrap resamples = ${RESAMPLES}, paired by seed`);
}

main().catch((error: unknown) => {
  console.error(error);
  process.exitCode = 1;
});
