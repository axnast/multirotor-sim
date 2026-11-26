/**
 * M6 autotune runner (long job; do not run from an agent shell).
 *
 *   npm run tune                 start, 750 evaluations
 *   npm run tune:resume          resume from out/autotune_checkpoint.json
 *   npm run tune:extend          resume and allow the pre-registered extension
 *
 * Eight worker_threads evaluate each generation in parallel. The CMA-ES state
 * (including RNG) is checkpointed after every generation; a Ctrl+C mid-generation
 * leaves the previous checkpoint valid. All evaluations use full 120 s noisy
 * scenarios with rotating dev-seed subsets. Held-out seeds never appear here.
 */

import { createHash } from "node:crypto";
import { mkdirSync, writeFileSync, readFileSync, existsSync, appendFileSync } from "node:fs";
import { Worker } from "node:worker_threads";

import { defaultControllerParams } from "../src/core/control/controller";
import { makeHex } from "../src/core/multirotor/config";
import {
  createBaselineTable,
  createCostSuite,
  DEV_SEEDS,
  FULL_STEPS,
  TERM_WEIGHTS,
  type BaselineTable,
} from "../src/core/tuning/cost";
import {
  GAIN_BOUNDS,
  GAIN_KEYS,
  decode,
  defaultsToVector,
  withGains,
} from "../src/core/tuning/params";
import {
  ask,
  createState,
  deserializeState,
  serializeState,
  tell,
  type CmaesState,
} from "../src/core/tuning/cmaes";

const MODE = process.argv.includes("--resume")
  ? "resume"
  : process.argv.includes("--extend")
    ? "extend"
    : "start";
function argument(name: string): number | null {
  const index = process.argv.indexOf(name);
  if (index < 0) return null;
  const value = Number(process.argv[index + 1]);
  if (!Number.isSafeInteger(value) || value <= 0) throw new RangeError(`${name} needs a positive integer`);
  return value;
}
/** Smoke-only overrides; never used for reported costs or the real tuning run. */
const MAX_EVALS = argument("--max-evals");
const SMOKE_STEPS = argument("--steps");
const BUDGET = MAX_EVALS ?? (MODE === "extend" ? 1_000 : 750);
const WORKERS = 8;
const SEED = 20_261_126;
const ROTATIONS = [[1, 2], [2, 3], [3, 4], [4, 5], [5, 1]] as const;
const CHECKPOINT = "out/autotune_checkpoint.json";
const HISTORY = "out/autotune_history.csv";
const RESULT = "data/tunedGains.json";
const STEPS = SMOKE_STEPS ?? FULL_STEPS;

interface Evaluation {
  readonly id: number;
  readonly total: number;
}

interface HistoryRow {
  readonly generation: number;
  readonly evaluations: number;
  readonly best: number;
  readonly sigma: number;
  readonly seeds: readonly number[];
  /** Fixed dev-seed 1-5 re-scores at generations 60 and 75 (extension rule). */
  readonly rescoreGeneration: number | null;
  readonly rescoreCost: number | null;
}

function makePool(baselineTable: BaselineTable, steps: number): {
  run: (job: { id: number; vector: readonly number[]; seeds: readonly number[] }) => Promise<Evaluation>;
  close: () => void;
} {
  interface Slot {
    worker: Worker;
    busy: boolean;
    resolve: ((value: Evaluation) => void) | null;
  }
  const slots: Slot[] = [];
  const queue: { job: { id: number; vector: readonly number[]; seeds: readonly number[] }; resolve: (value: Evaluation) => void }[] = [];
  for (let index = 0; index < WORKERS; index++) {
    const worker = new Worker(new URL("./lib/costWorker.ts", import.meta.url), {
      workerData: { baselineTable, steps },
      execArgv: ["--import", "tsx"],
    });
    const slot: Slot = { worker, busy: false, resolve: null };
    worker.on("message", (result: Evaluation) => {
      slot.busy = false;
      slot.resolve?.(result);
      slot.resolve = null;
      pump();
    });
    slots.push(slot);
  }
  function pump(): void {
    while (queue.length > 0) {
      const slot = slots.find((candidate) => !candidate.busy);
      if (slot === undefined) return;
      const task = queue.shift()!;
      slot.busy = true;
      slot.resolve = task.resolve;
      slot.worker.postMessage(task.job);
    }
  }
  return {
    run: (job) => new Promise<Evaluation>((resolve) => { queue.push({ job, resolve }); pump(); }),
    close: () => { for (const slot of slots) void slot.worker.terminate(); },
  };
}

/**
 * Pre-run amendment: in-run costs use rotating seed subsets and are not
 * comparable across generations. Only the fixed dev-seed 1-5 re-scores of the
 * incumbent at generations 60 and 75 decide the extension.
 */
function extensionAllowed(history: readonly HistoryRow[]): boolean {
  const at60 = history.find((row) => row.rescoreGeneration === 60)?.rescoreCost;
  const at75 = history.find((row) => row.rescoreGeneration === 75)?.rescoreCost;
  if (at60 === undefined || at75 === undefined || at60 === null || at75 === null) return false;
  return at60 > 0 && (at60 - at75) / at60 > 0.02;
}

async function main(): Promise<void> {
  mkdirSync("out", { recursive: true });
  mkdirSync("data", { recursive: true });
  const config = makeHex({ groundPlane: true });
  const base = defaultControllerParams(config);
  const baselineTable = createBaselineTable(
    [...ROTATIONS, DEV_SEEDS as readonly number[]],
    config,
    base,
    STEPS,
  );

  const suite = createCostSuite(config, base, STEPS, baselineTable);
  const bounds = {
    lower: GAIN_KEYS.map((key) => Math.log(GAIN_BOUNDS[key].lower)),
    upper: GAIN_KEYS.map((key) => Math.log(GAIN_BOUNDS[key].upper)),
  };
  const startVector = defaultsToVector(base);
  let state: CmaesState;
  let history: HistoryRow[] = [];
  let incumbent: readonly number[] = startVector;
  let incumbentCost = Infinity;
  if (MODE === "start" || !existsSync(CHECKPOINT)) {
    state = createState({ dimension: 9, seed: SEED, lambda: 10, bounds }, startVector);
    writeFileSync(
      HISTORY,
      "generation,evaluations,best_inrun_cost,sigma,seeds,rescore_generation,rescore_dev_cost\n",
    );
  } else {
    const saved = JSON.parse(readFileSync(CHECKPOINT, "utf8")) as {
      state: string;
      history: HistoryRow[];
      incumbent?: readonly number[];
      incumbentCost?: number;
    };
    state = deserializeState(saved.state);
    history = saved.history;
    if (saved.incumbent !== undefined) incumbent = saved.incumbent;
    if (saved.incumbentCost !== undefined) incumbentCost = saved.incumbentCost;
  }
  if (MODE === "extend" && !extensionAllowed(history)) {
    console.log("extension rule not satisfied (75 vs 60 fixed dev-seed re-score <= 2%); stopping");
    return;
  }

  const pool = makePool(baselineTable, STEPS);
  const candidates: { vector: readonly number[]; cost: number }[] = [];
  let nextId = 0;
  process.on("SIGINT", () => {
    pool.close();
    console.log(`\nstopped safely; checkpoint at generation ${state.generation} is valid`);
    process.exit(0);
  });

  while (state.evaluations < BUDGET) {
    const seeds = ROTATIONS[state.generation % ROTATIONS.length] as readonly number[];
    const pending = ask(state);
    const jobs = pending.candidates.map((vector) => ({ id: nextId++, vector, seeds }));
    const results = await awaitPool(pool, jobs);
    results.sort((a, b) => a.id - b.id);
    for (let index = 0; index < pending.candidates.length; index++) {
      candidates.push({ vector: pending.candidates[index]!, cost: results[index]!.total });
    }
    for (let index = 0; index < results.length; index++) {
      if (results[index]!.total < incumbentCost) {
        incumbentCost = results[index]!.total;
        incumbent = pending.candidates[index]!;
      }
    }
    state = tell(state, pending, results.map((result) => result.total));
    let rescoreGeneration: number | null = null;
    let rescoreCost: number | null = null;
    if (state.generation === 60 || state.generation === 75) {
      rescoreGeneration = state.generation;
      rescoreCost = suite.evaluate(withGains(base, incumbent), DEV_SEEDS).total;
    }
    const row: HistoryRow = {
      generation: state.generation,
      evaluations: state.evaluations,
      best: state.best?.f ?? Infinity,
      sigma: state.sigma,
      seeds,
      rescoreGeneration,
      rescoreCost,
    };
    history = [...history, row];
    appendFileSync(
      HISTORY,
      `${row.generation},${row.evaluations},${row.best},${row.sigma},${seeds.join("|")},` +
        `${rescoreGeneration ?? ""},${rescoreCost ?? ""}\n`,
    );
    writeFileSync(
      CHECKPOINT,
      JSON.stringify({ state: serializeState(state), history, incumbent, incumbentCost }),
    );
    console.log(
      `gen ${row.generation} | evals ${row.evaluations} | best ${row.best.toFixed(6)} | ` +
        `sigma ${row.sigma.toExponential(2)}` +
        (rescoreCost === null ? "" : ` | fixed dev re-score ${rescoreCost.toFixed(6)}`),
    );
  }
  pool.close();

  // Winner selection: full dev-seed 1-5 re-score of the top five, never the
  // in-run rotating-seed cost.
  const ranked = [...candidates, { vector: incumbent, cost: incumbentCost }]
    .sort((a, b) => a.cost - b.cost)
    .slice(0, 5);
  let bestEntry: { vector: readonly number[]; cost: number } | null = null;
  for (const entry of ranked) {
    const cost = suite.evaluate(withGains(base, entry.vector), DEV_SEEDS).total;
    if (bestEntry === null || cost < bestEntry.cost) bestEntry = { vector: entry.vector, cost };
  }
  if (bestEntry === null) throw new Error("no candidates collected");
  const gains = decode(bestEntry.vector);
  const digest = createHash("sha256").update(JSON.stringify({ gains, weights: TERM_WEIGHTS })).digest("hex");
  writeFileSync(
    RESULT,
    JSON.stringify(
      {
        version: 1,
        seed: SEED,
        lambda: 10,
        evaluations: state.evaluations,
        generations: state.generation,
        weights: TERM_WEIGHTS,
        bounds,
        selection: "top-5 full dev-seed 1-5 re-score",
        devCost: bestEntry.cost,
        vector: bestEntry.vector,
        gains,
        digest,
      },
      null,
      2,
    ),
  );
  console.log(`wrote ${RESULT} (dev cost ${bestEntry.cost.toFixed(6)})`);
}

function awaitPool(
  pool: ReturnType<typeof makePool>,
  jobs: readonly { id: number; vector: readonly number[]; seeds: readonly number[] }[],
): Promise<Evaluation[]> {
  return Promise.all(jobs.map((job) => pool.run(job)));
}

main();
