/**
 * Milestone 5 demo: disturbance scenario table with three controller variants.
 *
 * True position error (never the noisy measurement), five fixed seeds, 120 s
 * runs, RMS over [20,60] and [90,120] seconds. Each cell prints mean [min,max]
 * across seeds. Runtime is roughly ten minutes (105 runs x 120k steps).
 *
 * Run with: npm run demo:disturbance
 * Output:   out/disturbance.csv
 */

import { mkdirSync, writeFileSync } from "node:fs";

import { defaultControllerParams } from "../src/core/control/controller";
import type { ControllerParams } from "../src/core/control/types";
import {
  disturbanceParamsFor,
  SCENARIO_IDS,
} from "../src/core/disturbance/scenario";
import type { DisturbanceParams } from "../src/core/disturbance/types";
import { makeHex } from "../src/core/multirotor/config";
import { MISSIONS } from "../src/viewer/sim/mission";
import { createSimulation, stepOnce } from "../src/viewer/sim/runner";

const STEPS = 120_000;
const DT = 0.001;
const SEEDS = [1, 2, 3, 4, 5] as const;
const WINDOWS = [[20, 60], [90, 120]] as const;
const OUTPUT_PATH = "out/disturbance.csv";

interface Variant {
  readonly label: string;
  readonly params: ControllerParams;
}

interface Row {
  readonly label: string;
  readonly disturbance: (seed: number) => DisturbanceParams;
}

interface Cell {
  readonly mean: number;
  readonly min: number;
  readonly max: number;
}

const config = makeHex({ groundPlane: true });
const base = defaultControllerParams(config);
const VARIANTS: readonly Variant[] = [
  {
    label: "M3 gate",
    // The pre-fix gate: bit-identical to M3 on the flight profile.
    params: { ...base, position: { ...base.position, integralDwell: 0 } },
  },
  {
    label: "default",
    params: base,
  },
  {
    label: "gate off (reference only)",
    // Plain conditional integration. NOT the M3 baseline: overshoots a 5 m
    // step by 12.7%. Included only to show the unlimited-integral reference.
    params: { ...base, position: { ...base.position, integralSeparation: 1e9 } },
  },
];

function constantWind(seed: number): DisturbanceParams {
  const calm = disturbanceParamsFor("calm", seed);
  return {
    ...calm,
    wind: {
      ...calm.wind,
      enabled: true,
      mean: [5, 0, 0],
      gustSigma: [0, 0, 0],
    },
  };
}

function heavyPayload(seed: number): DisturbanceParams {
  return disturbanceParamsFor("calm", seed, {
    payload: {
      atTime: 0,
      payload: { mass: 0.3, position: [0, 0, 0] },
    },
  });
}

const ROWS: readonly Row[] = [
  ...SCENARIO_IDS.map((id) => ({
    label: id,
    disturbance: (seed: number) => disturbanceParamsFor(id, seed),
  })),
  { label: "constant wind", disturbance: constantWind },
  { label: "+20% mass", disturbance: heavyPayload },
];

function cell(params: ControllerParams, disturbance: DisturbanceParams, start: number, end: number): number {
  const plant = { config, params };
  // A tiny trail keeps this long batch cheap; telemetry writes are unaffected.
  let state = createSimulation(plant, MISSIONS.hover, 16, disturbance);
  let sumSquared = 0;
  let count = 0;
  for (let step = 0; step < STEPS; step++) {
    state = stepOnce(state, plant);
    const time = (step + 1) * DT;
    if (time >= start && time <= end) {
      const position = state.multirotor.body.position;
      const error = Math.hypot(position[0], position[1], position[2] - 5);
      sumSquared += error * error;
      count++;
    }
  }
  return Math.sqrt(sumSquared / count);
}

function stats(values: readonly number[]): Cell {
  return {
    mean: values.reduce((a, b) => a + b, 0) / values.length,
    min: Math.min(...values),
    max: Math.max(...values),
  };
}

function fmt(value: Cell): string {
  return `${value.mean.toFixed(4)} [${value.min.toFixed(4)},${value.max.toFixed(4)}]`;
}

function main(): void {
  const started = Date.now();
  const csv: string[] = ["row,variant,window_s,mean_m,min_m,max_m"];
  const results: string[] = [];

  for (const row of ROWS) {
    for (const variant of VARIANTS) {
      for (const [start, end] of WINDOWS) {
        const values = SEEDS.map((seed) =>
          cell(variant.params, row.disturbance(seed), start, end),
        );
        const s = stats(values);
        csv.push(`${row.label},${variant.label},${start}-${end},${s.mean.toFixed(6)},${s.min.toFixed(6)},${s.max.toFixed(6)}`);
        results.push(
          `${row.label.padEnd(14)} | ${variant.label.padEnd(26)} | ${`${start}-${end}`.padEnd(7)} | ${fmt(s)}`,
        );
        console.log(results[results.length - 1]);
      }
    }
  }

  mkdirSync("out", { recursive: true });
  writeFileSync(OUTPUT_PATH, `${csv.join("\n")}\n`);
  const elapsed = (Date.now() - started) / 1000;
  console.log(`\nTrue-position RMS (m), mean [min,max] over seeds 1-5, 120 s runs.`);
  console.log(`Wrote ${OUTPUT_PATH}; elapsed ${elapsed.toFixed(1)} s (${(elapsed / 60).toFixed(1)} min).`);
}

main();
