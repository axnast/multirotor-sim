/**
 * Reproducible CMA-ES calibration (fixed seeds, lambda = 10). Run with
 * `npm run tune:calibrate`. Reports evaluations to target on a 10-D sphere,
 * on a 10-D quadratic with condition number 1e3, and a noisy-sphere trace.
 * These are optimizer-only measurements: no simulator work is involved.
 */

import {
  createState,
  stepGeneration,
  type CmaesState,
} from "../src/core/tuning/cmaes";

const DIMENSION = 10;
const LAMBDA = 10;
const BOUNDS = { lower: Array(DIMENSION).fill(-5), upper: Array(DIMENSION).fill(5) };
const MAX_GENERATIONS = 2_000;
const START = [2, -3, 1.5, -1, 2.5, -2, 1, -1.5, 3, -2.5];

function options(seed: number) {
  return { dimension: DIMENSION, seed, lambda: LAMBDA, bounds: BOUNDS };
}

function sphere(x: readonly number[]): number {
  return x.reduce((sum, value) => sum + value * value, 0);
}

function illConditioned(x: readonly number[]): number {
  return x.reduce((sum, value, index) => sum + 10 ** ((3 * index) / 9) * value ** 2, 0);
}

/** Deterministic, optimizer-invisible jitter: a smooth high-frequency ripple. */
function noisySphere(x: readonly number[]): number {
  const ripple = Math.sin(17.3 * x.reduce((sum, value, i) => sum + (i + 1) * value, 0));
  return sphere(x) + 0.05 * ripple;
}

function run(
  objective: (x: readonly number[]) => number,
  start: readonly number[],
  seed: number,
  until: (state: CmaesState, initial: number) => boolean,
): { state: CmaesState; initial: number } {
  const initial = objective(start);
  let state = createState(options(seed), start);
  for (let generation = 0; generation < MAX_GENERATIONS; generation++) {
    if (until(state, initial)) break;
    state = stepGeneration(state, objective).state;
  }
  return { state, initial };
}

function main(): void {
  const sphereResult = run(
    sphere,
    START,
    1,
    (state) => state.best !== null && state.best.f < 1e-10,
  );
  console.log(
    `sphere10 f0=${sphereResult.initial.toFixed(4)} reached f<1e-10 after ` +
      `${sphereResult.state.evaluations} evaluations ` +
      `(${sphereResult.state.generation} generations), f=${sphereResult.state.best?.f.toExponential(3)}`,
  );

  const quadraticResult = run(
    illConditioned,
    Array(DIMENSION).fill(1),
    2,
    (state, initial) =>
      state.best !== null && (state.best.f < initial / 10 || state.best.f < initial / 100) &&
      state.best.f >= initial / 100,
  );
  console.log(
    `quadratic10 kappa=1e3 f0=${quadraticResult.initial.toFixed(4)} ` +
      `10x after ${quadraticResult.state.evaluations} evaluations ` +
      `(${quadraticResult.state.generation} generations)`,
  );
  const quadratic100 = run(
    illConditioned,
    Array(DIMENSION).fill(1),
    2,
    (state, initial) => state.best !== null && state.best.f < initial / 100,
  );
  console.log(
    `quadratic10 kappa=1e3 100x after ${quadratic100.state.evaluations} evaluations ` +
      `(${quadratic100.state.generation} generations), f=${quadratic100.state.best?.f.toExponential(3)}`,
  );

  let state = createState(options(3), START);
  const checkpoints = new Set([0, 10, 25, 50, 100, 200, 500]);
  console.log("noisy sphere10 (seed 3, lambda 10) generation | evaluations | best f | sigma");
  for (let generation = 0; generation <= 500; generation++) {
    if (checkpoints.has(generation)) {
      console.log(
        `${String(generation).padStart(4)} | ${String(state.evaluations).padStart(5)} | ` +
          `${(state.best?.f ?? Infinity).toExponential(3)} | ${state.sigma.toExponential(3)}`,
      );
    }
    if (generation < 500) state = stepGeneration(state, noisySphere).state;
  }
}

main();
