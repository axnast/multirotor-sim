import { describe, expect, it } from "vitest";

import {
  ask,
  createState,
  deserializeState,
  resolveOptions,
  serializeState,
  stepGeneration,
  tell,
  type CmaesState,
} from "../src/core/tuning/cmaes";

const BOUNDS = { lower: Array(10).fill(-5), upper: Array(10).fill(5) };

function sphere(x: readonly number[]): number {
  return x.reduce((sum, value) => sum + value * value, 0);
}

function rosenbrock(x: readonly number[]): number {
  let sum = 0;
  for (let i = 0; i < x.length - 1; i++) {
    sum += 100 * (x[i + 1]! - x[i]! ** 2) ** 2 + (1 - x[i]!) ** 2;
  }
  return sum;
}

function noisySphere(x: readonly number[]): number {
  const ripple = Math.sin(17.3 * x.reduce((sum, value, i) => sum + (i + 1) * value, 0));
  return sphere(x) + 0.05 * ripple;
}

function run(
  state: CmaesState,
  objective: (x: readonly number[]) => number,
  generations: number,
): CmaesState {
  for (let index = 0; index < generations; index++) {
    state = stepGeneration(state, objective).state;
  }
  return state;
}

describe("CMA-ES", () => {
  it("uses the documented population size by default and honours an override", () => {
    expect(resolveOptions({ dimension: 10, seed: 1 }).lambda).toBe(10);
    expect(resolveOptions({ dimension: 10, seed: 1, lambda: 20 }).lambda).toBe(20);
  });

  it("minimises a 10-D sphere", () => {
    const start = [2, -3, 1.5, -1, 2.5, -2, 1, -1.5, 3, -2.5];
    const state = run(
      createState({ dimension: 10, seed: 1, lambda: 10, bounds: BOUNDS }, start),
      sphere,
      200,
    );
    expect(state.best?.f).toBeLessThan(1e-8);
  });

  it("makes progress on a 4-D Rosenbrock function", () => {
    const state = run(
      createState(
        {
          dimension: 4,
          seed: 5,
          lambda: 10,
          bounds: { lower: [-5, -5, -5, -5], upper: [5, 5, 5, 5] },
        },
        [-2, 2, -2, 2],
      ),
      rosenbrock,
      400,
    );
    expect(state.best?.f).toBeLessThan(1);
  });

  it("keeps descending on a noisy sphere", () => {
    const start = [2, -3, 1.5, -1, 2.5, -2, 1, -1.5, 3, -2.5];
    const options = { dimension: 10, seed: 3, lambda: 10, bounds: BOUNDS };
    const early = run(createState(options, start), noisySphere, 10);
    const late = run(createState(options, start), noisySphere, 60);
    expect(late.best!.f).toBeLessThan(early.best!.f);
    expect(late.best!.f).toBeLessThan(0.5);
  });

  it("keeps every candidate inside the box", () => {
    const bounds = { lower: Array(4).fill(-1), upper: Array(4).fill(1) };
    let state = createState(
      { dimension: 4, seed: 7, lambda: 10, bounds },
      [0, 0, 0, 0],
    );
    for (let generation = 0; generation < 30; generation++) {
      const pending = ask(state);
      for (const candidate of pending.candidates) {
        for (let i = 0; i < 4; i++) {
          expect(candidate[i]!).toBeGreaterThanOrEqual(bounds.lower[i]!);
          expect(candidate[i]!).toBeLessThanOrEqual(bounds.upper[i]!);
        }
      }
      const values = pending.candidates.map(sphere);
      state = tell(state, pending, values);
    }
    expect(state.best!.x.every((value) => Math.abs(value) <= 1)).toBe(true);
  });

  it("is deterministic for a seed and different for another seed", () => {
    const options = { dimension: 6, seed: 11, lambda: 10 };
    const a = run(createState(options), sphere, 5);
    const b = run(createState(options), sphere, 5);
    const c = run(createState({ ...options, seed: 12 }), sphere, 5);
    expect(serializeState(a)).toBe(serializeState(b));
    expect(serializeState(a)).not.toBe(serializeState(c));
  });

  it("resumes from a full JSON checkpoint bit-identically", () => {
    const options = { dimension: 10, seed: 1, lambda: 10, bounds: BOUNDS };
    const start = [2, -3, 1.5, -1, 2.5, -2, 1, -1.5, 3, -2.5];
    const uninterrupted = run(createState(options, start), sphere, 24);
    const halfway = run(createState(options, start), sphere, 12);
    const restored = deserializeState(serializeState(halfway));
    expect(serializeState(restored)).toBe(serializeState(halfway));
    const resumed = run(restored, sphere, 12);
    expect(serializeState(resumed)).toBe(serializeState(uninterrupted));

    // The next generation's candidates must also match exactly.
    expect(ask(resumed).candidates).toEqual(ask(uninterrupted).candidates);
  });
});
