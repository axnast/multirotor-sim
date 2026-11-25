import { describe, expect, it } from "vitest";

import {
  createPrng,
  nextGaussian,
  nextUint32,
  nextUniform,
} from "../src/core/disturbance/prng";

const DRAW_COUNT = 200_000;

describe("seeded xoshiro128** PRNG", () => {
  it("matches pinned sequences (known-answer regression)", () => {
    const firstFive = (seed: number): number[] => {
      let state = createPrng(seed);
      const values: number[] = [];
      for (let i = 0; i < 5; i++) {
        const next = nextUint32(state);
        values.push(next.value);
        state = next.state;
      }
      return values;
    };
    expect(firstFive(1)).toEqual([
      393288148, 2174103013, 3814759091, 2092745082, 1865176206,
    ]);
    expect(firstFive(12345)).toEqual([
      1093274547, 203003357, 3741353573, 3803725158, 4178738660,
    ]);
  });

  it("is reproducible for a seed and different for another seed", () => {
    let a = createPrng(99);
    let b = createPrng(99);
    let c = createPrng(100);
    const first: number[] = [];
    for (let i = 0; i < 100; i++) {
      const na = nextUint32(a);
      const nb = nextUint32(b);
      const nc = nextUint32(c);
      a = na.state;
      b = nb.state;
      c = nc.state;
      first.push(na.value);
      expect(nb.value).toBe(na.value);
    }
    expect(nextUint32(c).value).not.toBe(first[0]!);
  });

  it("draws uniform values with the expected mean and variance", () => {
    // SE(mean)=sigma/sqrt(N)=0.2887/447.2=0.000645 (4 sigma = 0.0026).
    // Var(sample variance)=(mu4-sigma^4)/N=(1/80-1/144)/N -> SE=1.67e-4
    // (4 sigma = 6.7e-4). Fixed seed, so these bounds cannot flake.
    let state = createPrng(2024);
    let sum = 0;
    let sumSquares = 0;
    let minimum = Infinity;
    let maximum = -Infinity;
    for (let i = 0; i < DRAW_COUNT; i++) {
      const next = nextUniform(state);
      state = next.state;
      sum += next.value;
      sumSquares += next.value * next.value;
      minimum = Math.min(minimum, next.value);
      maximum = Math.max(maximum, next.value);
    }
    expect(minimum).toBeGreaterThan(0);
    expect(maximum).toBeLessThan(1);
    const mean = sum / DRAW_COUNT;
    const variance = sumSquares / DRAW_COUNT - mean * mean;
    expect(Math.abs(mean - 0.5)).toBeLessThan(0.0026);
    expect(Math.abs(variance - 1 / 12)).toBeLessThan(0.0007);
  });

  it("draws Gaussians with the expected mean and variance, caching one spare", () => {
    // SE(mean)=1/sqrt(N)=0.00224 (4 sigma = 0.009);
    // SE(std)=1/sqrt(2N)=0.00158 (4 sigma = 0.0063).
    const first = nextGaussian(createPrng(7));
    expect(first.value).toBeCloseTo(1.2426813089235123, 12);
    const second = nextGaussian(first.state);
    expect(second.value).toBeCloseTo(2.118306297949288, 12);
    // The second sample consumes only the cached spare, not new PRNG words.
    expect(second.state.s).toEqual(first.state.s);
    expect(second.state.hasSpare).toBe(false);
    const third = nextGaussian(second.state);
    expect(third.value).toBeCloseTo(0.3642670921515022, 12);
    expect(third.state.hasSpare).toBe(true);

    let state = createPrng(31);
    let sum = 0;
    let sumSquares = 0;
    for (let i = 0; i < DRAW_COUNT; i++) {
      const next = nextGaussian(state);
      state = next.state;
      sum += next.value;
      sumSquares += next.value * next.value;
    }
    const mean = sum / DRAW_COUNT;
    const std = Math.sqrt(sumSquares / DRAW_COUNT - mean * mean);
    expect(Math.abs(mean)).toBeLessThan(0.009);
    expect(Math.abs(std - 1)).toBeLessThan(0.0063);
  });
});
