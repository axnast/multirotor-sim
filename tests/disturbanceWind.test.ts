import { describe, expect, it } from "vitest";

import {
  cosineGustAt,
  initialWindState,
  windStep,
} from "../src/core/disturbance/wind";
import type { WindParams } from "../src/core/disturbance/types";

const CORRELATION_TIME = 2;

function ouParams(gustSigma: readonly [number, number, number]): WindParams {
  return {
    enabled: true,
    mean: [0, 0, 0],
    gustSigma: [...gustSigma],
    correlationTime: CORRELATION_TIME,
    cosineGust: null,
  };
}

describe("Ornstein-Uhlenbeck wind", () => {
  it("reproduces the stationary mean, variance and lag-1 autocorrelation", () => {
    // dt = 0.05 s, N = 4e6 samples after a 1000-step burn-in.
    // For an AR(1) process rho = exp(-dt/Tc), the variance of the sample mean
    // is (sigma^2/N)*(1+rho)/(1-rho), i.e. N_eff = N*(1-rho)/(1+rho).
    // Here rho=0.975309912, N_eff=49,997:
    //   SE(mean) = 1/sqrt(N_eff) = 0.00447    -> 4 sigma = 0.0179
    //   SE(var)  = sigma^2*sqrt(2/N_eff) = 0.00632 -> 4 sigma = 2.53%
    //   SE(rho1) = (1-rho1^2)/sqrt(M) = 0.00109 for M=2000 independent pairs
    //              (pairs 100 s = 50 Tc apart, bivariate-normal formula)
    //              -> 4 sigma = 0.00436.
    const dt = 0.05;
    const count = 4_000_000;
    const burnIn = 1_000;
    const params = ouParams([1, 1, 1]);
    let state = initialWindState(12345);
    const samples = new Float64Array(count);

    for (let i = 0; i < burnIn + count; i++) {
      const next = windStep(state, params, dt);
      state = next.state;
      if (i >= burnIn) samples[i - burnIn] = next.wind[0]!;
    }

    let sum = 0;
    for (const value of samples) sum += value;
    const mean = sum / count;
    let sumSquares = 0;
    for (const value of samples) {
      const delta = value - mean;
      sumSquares += delta * delta;
    }
    const variance = sumSquares / count;
    expect(Math.abs(mean)).toBeLessThan(0.018);

    // Block-pair estimator: pairs (x_i, x_{i+1}) from blocks 2000 steps apart.
    const block = 2_000;
    const pairs = Math.floor(count / block);
    let sx = 0;
    let sy = 0;
    let sxx = 0;
    let syy = 0;
    let sxy = 0;
    for (let index = 0; index < pairs; index++) {
      const x = samples[index * block]!;
      const y = samples[index * block + 1]!;
      sx += x;
      sy += y;
      sxx += x * x;
      syy += y * y;
      sxy += x * y;
    }
    const rho1 = (sxy - sx * sy / pairs) /
      Math.sqrt((sxx - sx * sx / pairs) * (syy - sy * sy / pairs));

    expect(Math.abs(variance - 1)).toBeLessThan(0.026);
    expect(Math.abs(rho1 - Math.exp(-dt / CORRELATION_TIME))).toBeLessThan(0.005);
  }, 60_000);

  it("has the same conditional distribution at 1 ms, 5 ms and 50 ms", () => {
    // Exact discretisation: starting at x0 = 2 sigma, after K steps
    //   E[x_K] = a^K x0, Var[x_K] = sigma^2 (1 - a^(2K)).
    // This is exact for every dt, so it can be tested tightly with M=100k
    // independent paths (no autocorrelation penalty):
    //   SE(mean) = sd/sqrt(M), SE(var) = sd^2*sqrt(2/M), bounds = 4 sigma.
    // Pre-chosen, disjoint seed ranges per dt (fixed before measuring):
    //   dt=1 ms -> seeds 1,000,000..1,099,999
    //   dt=5 ms -> seeds 2,000,000..2,099,999
    //   dt=50 ms -> seeds 3,000,000..3,099,999
    // so the three ensembles are independent rather than shifted copies.
    const paths = 100_000;
    const steps = 20;
    const x0 = 2;
    const params = ouParams([1, 0, 0]);
    const ensembles: readonly (readonly [number, number])[] = [
      [0.001, 1_000_000],
      [0.005, 2_000_000],
      [0.05, 3_000_000],
    ];

    for (const [dt, seedBase] of ensembles) {
      const a = Math.exp(-dt / CORRELATION_TIME);
      const expectedMean = Math.pow(a, steps) * x0;
      const expectedVariance = 1 - Math.pow(a, 2 * steps);
      let sum = 0;
      let sumSquares = 0;
      for (let path = 0; path < paths; path++) {
        let state = initialWindState(seedBase + path, [x0, 0, 0]);
        for (let step = 0; step < steps; step++) {
          state = windStep(state, params, dt).state;
        }
        const value = state.gust[0]!;
        sum += value;
        sumSquares += value * value;
      }
      const mean = sum / paths;
      const variance = sumSquares / paths - mean * mean;
      const seMean = Math.sqrt(expectedVariance / paths);
      const seVariance = expectedVariance * Math.sqrt(2 / paths);
      expect(Math.abs(mean - expectedMean)).toBeLessThan(4 * seMean);
      expect(Math.abs(variance - expectedVariance)).toBeLessThan(4 * seVariance);
    }
  }, 120_000);

  it("uses the exact 1-cosine gust shape and resets the PRNG when disabled", () => {
    const gust = {
      startTime: 10,
      duration: 4,
      amplitude: 2,
      direction: [1, 0, 0] as const,
    };
    expect(cosineGustAt(9.999, gust)).toBe(0);
    expect(cosineGustAt(10, gust)).toBe(0);
    expect(cosineGustAt(11, gust)).toBeCloseTo(1, 12);
    expect(cosineGustAt(12, gust)).toBeCloseTo(2, 12);
    expect(cosineGustAt(13, gust)).toBeCloseTo(1, 12);
    expect(cosineGustAt(14, gust)).toBe(0);

    const params: WindParams = {
      ...ouParams([0, 0, 0]),
      mean: [0, 0, 0],
      cosineGust: gust,
    };
    let state = initialWindState(5);
    for (let step = 0; step < 12; step++) {
      state = windStep(state, params, 1).state;
    }
    // Zero sigma keeps the OU process at exactly zero while the PRNG still
    // advances, so the total wind isolates the 1-cosine event.
    expect(state.time).toBeCloseTo(12, 12);
    expect(state.gust).toEqual([0, 0, 0]);
    const withEvent = windStep(state, params, 0.01);
    expect(withEvent.wind[0]).toBeCloseTo(cosineGustAt(12, gust), 12);
    expect(withEvent.wind[0]).toBeCloseTo(2, 12);

    const disabled: WindParams = { ...params, enabled: false };
    const untouched = initialWindState(5);
    const result = windStep(untouched, disabled, 0.01);
    expect(result.state).toBe(untouched);
    expect(result.wind).toEqual([0, 0, 0]);
  });
});
