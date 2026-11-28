/**
 * Pure statistics for the M6 benchmark: bootstrap confidence intervals and the
 * geometric-mean ratio headline. Kept out of the script so the resampling can
 * be unit-tested on synthetic data with known answers.
 */

import { nextUint32, seedRng, type RngState } from "./rng";

export function mean(values: readonly number[]): number {
  if (values.length === 0) throw new RangeError("mean of an empty array");
  return values.reduce((sum, value) => sum + value, 0) / values.length;
}

/** Nearest-rank percentile of an already ascending array. */
export function percentile(sorted: readonly number[], fraction: number): number {
  if (sorted.length === 0) throw new RangeError("percentile of an empty array");
  if (!(fraction >= 0 && fraction <= 1)) throw new RangeError("fraction must be in [0,1]");
  return sorted[Math.min(sorted.length - 1, Math.floor(fraction * sorted.length))]!;
}

/** Fraction of paired differences below zero (tuned better than baseline). */
export function fractionImproved(differences: readonly number[]): number {
  if (differences.length === 0) throw new RangeError("no differences");
  return differences.filter((value) => value < 0).length / differences.length;
}

function drawIndices(
  rng: RngState,
  count: number,
): { indices: number[]; state: RngState } {
  let state = rng;
  const indices: number[] = [];
  for (let draw = 0; draw < count; draw++) {
    const next = nextUint32(state);
    indices.push(next.value % count);
    state = next.state;
  }
  return { indices, state };
}

/** Paired bootstrap CI of the mean difference (resamples seed indices). */
export function pairedCI(
  differences: readonly number[],
  resamples = 10_000,
  seed = 424_242,
): readonly [number, number] {
  if (differences.length === 0) throw new RangeError("no differences");
  let rng = seedRng(seed);
  const samples: number[] = [];
  for (let resample = 0; resample < resamples; resample++) {
    const draw = drawIndices(rng, differences.length);
    rng = draw.state;
    samples.push(mean(draw.indices.map((index) => differences[index]!)));
  }
  samples.sort((a, b) => a - b);
  return [percentile(samples, 0.025), percentile(samples, 0.975)];
}

export interface HeadlineResult {
  readonly ratio: number;
  readonly ci: readonly [number, number];
}

/**
 * Geometric mean over scenarios of tuned/baseline RMS, resampling seed indices
 * jointly across all scenarios (paired by seed). `tuned[s][i]` is scenario s on
 * seed i; every scenario must have the same number of seeds.
 */
export function geometricMeanRatio(
  tuned: readonly (readonly number[])[],
  baseline: readonly (readonly number[])[],
  resamples = 10_000,
  seed = 424_243,
): HeadlineResult {
  if (tuned.length === 0 || tuned.length !== baseline.length) {
    throw new RangeError("tuned and baseline must have the same non-zero scenario count");
  }
  const seedCount = tuned[0]!.length;
  if (seedCount === 0) throw new RangeError("at least one seed is required");
  if (tuned.some((values) => values.length !== seedCount) ||
      baseline.some((values) => values.length !== seedCount)) {
    throw new RangeError("every scenario must have the same number of seeds");
  }
  const compute = (indices: readonly number[]): number => {
    let product = 1;
    for (let scenario = 0; scenario < tuned.length; scenario++) {
      const tunedMean = mean(indices.map((index) => tuned[scenario]![index]!));
      const baselineMean = mean(indices.map((index) => baseline[scenario]![index]!));
      product *= tunedMean / baselineMean;
    }
    return product ** (1 / tuned.length);
  };
  const identity = tuned[0]!.map((_, index) => index);
  const ratio = compute(identity);
  let rng = seedRng(seed);
  const samples: number[] = [];
  for (let resample = 0; resample < resamples; resample++) {
    const draw = drawIndices(rng, seedCount);
    rng = draw.state;
    samples.push(compute(draw.indices));
  }
  samples.sort((a, b) => a - b);
  return { ratio, ci: [percentile(samples, 0.025), percentile(samples, 0.975)] };
}
