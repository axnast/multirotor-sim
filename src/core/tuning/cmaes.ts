/**
 * A small, deterministic CMA-ES implementation (Hansen's standard
 * parameters), written from scratch: no optimizer dependency, no hidden state
 * and no DOM. Intended for tuning controller gains in `src/core/tuning`.
 *
 * Coordinates are ordinary reals. Positive parameters are log-scaled by the
 * caller's encode/decode layer; CMA-ES itself is unconstrained except for the
 * optional box. Box handling is CLIPPING: a candidate outside the box is
 * projected onto it before evaluation, and the clipped displacement
 * `(x - oldMean) / sigma` is what feeds the covariance update. This keeps
 * every evaluation inside the documented bounds, at the cost of a slight bias
 * near a bound. All constants and the PRNG state live in `CmaesState`, so a
 * JSON checkpoint at a generation boundary resumes bit-identically.
 */

import { nextGaussian, seedRng, type RngState } from "./rng";

export interface CmaesBounds {
  readonly lower: readonly number[];
  readonly upper: readonly number[];
}

export interface CmaesOptions {
  readonly dimension: number;
  readonly seed: number;
  /** Default: 4 + floor(3 ln n) (10 for the 10-D gain vector). */
  readonly lambda?: number;
  /** Initial step size in search coordinates. Default: 0.25. */
  readonly sigma0?: number;
  readonly bounds?: CmaesBounds;
}

/** All constants resolved once, stored as plain data in the state. */
export interface ResolvedOptions {
  readonly dimension: number;
  readonly seed: number;
  readonly lambda: number;
  readonly mu: number;
  readonly sigma0: number;
  readonly bounds: CmaesBounds | null;
  readonly weights: readonly number[];
  readonly mueff: number;
  readonly cc: number;
  readonly cs: number;
  readonly c1: number;
  readonly cmu: number;
  readonly damps: number;
  readonly chiN: number;
}

export interface CmaesBest {
  readonly x: readonly number[];
  readonly f: number;
}

export interface CmaesState {
  readonly options: ResolvedOptions;
  readonly generation: number;
  readonly evaluations: number;
  readonly mean: readonly number[];
  readonly sigma: number;
  readonly covariance: readonly (readonly number[])[];
  readonly pathSigma: readonly number[];
  readonly pathCovariance: readonly number[];
  /** PRNG state AFTER the most recent ask, so resume reproduces the next ask. */
  readonly rng: RngState;
  readonly best: CmaesBest | null;
}

export interface AskResult {
  readonly candidates: readonly (readonly number[])[];
  /** Clipped displacements used by the covariance update. */
  readonly displacements: readonly (readonly number[])[];
  readonly previousMean: readonly number[];
  readonly rng: RngState;
}

function zeros(n: number): number[] {
  return Array.from({ length: n }, () => 0);
}

function identity(n: number): number[][] {
  return Array.from({ length: n }, (_, row) =>
    Array.from({ length: n }, (_, column) => (row === column ? 1 : 0)),
  );
}

export function resolveOptions(options: CmaesOptions): ResolvedOptions {
  const n = options.dimension;
  if (!Number.isSafeInteger(n) || n < 2) throw new RangeError("dimension must be >= 2");
  if (!Number.isSafeInteger(options.seed)) throw new RangeError("seed must be an integer");
  const lambda = options.lambda ?? 4 + Math.floor(3 * Math.log(n));
  if (!Number.isSafeInteger(lambda) || lambda < 4) {
    throw new RangeError("lambda must be an integer >= 4");
  }
  const mu = Math.floor(lambda / 2);
  const sigma0 = options.sigma0 ?? 0.25;
  if (!Number.isFinite(sigma0) || sigma0 <= 0) throw new RangeError("sigma0 must be positive");

  let bounds: CmaesBounds | null = null;
  if (options.bounds !== undefined) {
    const { lower, upper } = options.bounds;
    if (lower.length !== n || upper.length !== n) {
      throw new RangeError("bounds must have one lower and upper value per dimension");
    }
    for (let i = 0; i < n; i++) {
      const lo = lower[i]!;
      const hi = upper[i]!;
      if (!Number.isFinite(lo) || !Number.isFinite(hi) || lo >= hi) {
        throw new RangeError(`invalid bounds at dimension ${i}`);
      }
    }
    bounds = { lower: [...lower], upper: [...upper] };
  }

  const raw = Array.from({ length: mu }, (_, index) =>
    Math.log(mu + 0.5) - Math.log(index + 1),
  );
  const sum = raw.reduce((acc, value) => acc + value, 0);
  const weights = raw.map((value) => value / sum);
  const mueff = 1 / weights.reduce((acc, value) => acc + value * value, 0);
  const cc = (4 + mueff / n) / (n + 4 + (2 * mueff) / n);
  const cs = (mueff + 2) / (n + mueff + 5);
  const c1 = 2 / ((n + 1.3) ** 2 + mueff);
  const cmu = Math.min(
    1 - c1,
    (2 * (mueff - 2 + 1 / mueff)) / ((n + 2) ** 2 + mueff),
  );
  const damps =
    1 +
    2 * Math.max(0, Math.sqrt((mueff - 1) / (n + 1)) - 1) +
    cs;
  const chiN = Math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n));

  return {
    dimension: n,
    seed: options.seed,
    lambda,
    mu,
    sigma0,
    bounds,
    weights,
    mueff,
    cc,
    cs,
    c1,
    cmu,
    damps,
    chiN,
  };
}

export function createState(options: CmaesOptions, mean?: readonly number[]): CmaesState {
  const resolved = resolveOptions(options);
  const start = mean ?? zeros(resolved.dimension);
  if (start.length !== resolved.dimension) {
    throw new RangeError("mean must have one value per dimension");
  }
  if (resolved.bounds !== null) {
    for (let i = 0; i < start.length; i++) {
      const value = start[i]!;
      const lo = resolved.bounds.lower[i]!;
      const hi = resolved.bounds.upper[i]!;
      if (value < lo || value > hi) {
        throw new RangeError(`initial mean out of bounds at dimension ${i}`);
      }
    }
  }
  return {
    options: resolved,
    generation: 0,
    evaluations: 0,
    mean: [...start],
    sigma: resolved.sigma0,
    covariance: identity(resolved.dimension),
    pathSigma: zeros(resolved.dimension),
    pathCovariance: zeros(resolved.dimension),
    rng: seedRng(resolved.seed),
    best: null,
  };
}

interface EigenDecomposition {
  readonly values: readonly number[];
  /** `vectors[i][k]` is component i of eigenvector k. */
  readonly vectors: readonly (readonly number[])[];
}

/** Cyclic Jacobi eigen-decomposition of a symmetric n×n matrix (n is small). */
function jacobiEigen(matrix: readonly (readonly number[])[]): EigenDecomposition {
  const n = matrix.length;
  const a = matrix.map((row) => [...row]);
  const vectors = identity(n);
  for (let sweep = 0; sweep < 100 * n * n; sweep++) {
    let p = 0;
    let q = 1;
    let largest = 0;
    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        const magnitude = Math.abs(a[i]![j]!);
        if (magnitude > largest) {
          largest = magnitude;
          p = i;
          q = j;
        }
      }
    }
    if (largest < 1e-14) break;
    const app = a[p]![p]!;
    const aqq = a[q]![q]!;
    const apq = a[p]![q]!;
    const tau = (aqq - app) / (2 * apq);
    const t = tau === 0 ? 1 : Math.sign(tau) / (Math.abs(tau) + Math.sqrt(1 + tau * tau));
    const cosine = 1 / Math.sqrt(1 + t * t);
    const sine = t * cosine;

    for (let k = 0; k < n; k++) {
      if (k === p || k === q) continue;
      const akp = a[k]![p]!;
      const akq = a[k]![q]!;
      a[k]![p] = a[p]![k] = cosine * akp - sine * akq;
      a[k]![q] = a[q]![k] = sine * akp + cosine * akq;
    }
    a[p]![p] = cosine * cosine * app - 2 * cosine * sine * apq + sine * sine * aqq;
    a[q]![q] = sine * sine * app + 2 * cosine * sine * apq + cosine * cosine * aqq;
    a[p]![q] = 0;
    a[q]![p] = 0;
    for (let k = 0; k < n; k++) {
      const vkp = vectors[k]![p]!;
      const vkq = vectors[k]![q]!;
      vectors[k]![p] = cosine * vkp - sine * vkq;
      vectors[k]![q] = sine * vkp + cosine * vkq;
    }
  }
  return {
    values: Array.from({ length: n }, (_, index) => Math.max(a[index]![index]!, 1e-14)),
    vectors,
  };
}

/** Symmetrize and eigen-decompose C, returning sqrt(C) and invsqrt(C). */
function covarianceRoots(
  covariance: readonly (readonly number[])[],
): { sqrt: number[][]; inverseSqrt: number[][] } {
  const n = covariance.length;
  const symmetric = Array.from({ length: n }, (_, i) =>
    Array.from({ length: n }, (_, j) =>
      0.5 * (covariance[i]![j]! + covariance[j]![i]!),
    ),
  );
  const eigen = jacobiEigen(symmetric);
  const sqrt = identity(n);
  const inverseSqrt = identity(n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      let s = 0;
      let inv = 0;
      for (let k = 0; k < n; k++) {
        const factor = eigen.vectors[i]![k]! * eigen.vectors[j]![k]!;
        s += factor * Math.sqrt(eigen.values[k]!);
        inv += factor / Math.sqrt(eigen.values[k]!);
      }
      sqrt[i]![j] = s;
      inverseSqrt[i]![j] = inv;
    }
  }
  return { sqrt, inverseSqrt };
}

function clip(value: number, lower: number, upper: number): number {
  return Math.min(upper, Math.max(lower, value));
}

/** Draw one generation of candidates. Pure: the input state is untouched. */
export function ask(state: CmaesState): AskResult {
  const { options } = state;
  const n = options.dimension;
  const { sqrt } = covarianceRoots(state.covariance);
  const candidates: number[][] = [];
  const displacements: number[][] = [];
  let rng = state.rng;

  for (let candidate = 0; candidate < options.lambda; candidate++) {
    const y = zeros(n);
    for (let i = 0; i < n; i++) {
      const draw = nextGaussian(rng);
      rng = draw.state;
      y[i] = draw.value;
    }
    const z = zeros(n);
    for (let i = 0; i < n; i++) {
      let sum = 0;
      for (let j = 0; j < n; j++) sum += sqrt[i]![j]! * y[j]!;
      z[i] = sum;
    }
    const point = zeros(n);
    for (let i = 0; i < n; i++) {
      let raw = state.mean[i]! + state.sigma * z[i]!;
      if (options.bounds !== null) {
        raw = clip(raw, options.bounds.lower[i]!, options.bounds.upper[i]!);
      }
      point[i] = raw;
      // Covariance sees the clipped displacement, not the unclipped sample.
      z[i] = (raw - state.mean[i]!) / state.sigma;
    }
    candidates.push(point);
    displacements.push(z);
  }

  return {
    candidates,
    displacements,
    previousMean: [...state.mean],
    rng,
  };
}

function matVec(matrix: readonly (readonly number[])[], vector: readonly number[]): number[] {
  const n = vector.length;
  return Array.from({ length: n }, (_, i) => {
    let sum = 0;
    for (let j = 0; j < n; j++) sum += matrix[i]![j]! * vector[j]!;
    return sum;
  });
}

/** Incorporate one generation's objective values and update all state. */
export function tell(
  state: CmaesState,
  pending: AskResult,
  values: readonly number[],
): CmaesState {
  const { options } = state;
  if (values.length !== options.lambda) {
    throw new RangeError("one objective value per candidate is required");
  }
  if (values.some((value) => !Number.isFinite(value))) {
    throw new RangeError("objective values must be finite");
  }

  const order = Array.from({ length: options.lambda }, (_, index) => index);
  order.sort((a, b) => values[a]! - values[b]! || a - b);
  const selected = order.slice(0, options.mu);

  const step = zeros(options.dimension);
  for (let i = 0; i < options.mu; i++) {
    const z = pending.displacements[selected[i]!]!;
    const weight = options.weights[i]!;
    for (let j = 0; j < options.dimension; j++) step[j]! += weight * z[j]!;
  }
  const mean = state.mean.map((value, index) => value + state.sigma * step[index]!);

  const { inverseSqrt } = covarianceRoots(state.covariance);
  const whitened = matVec(inverseSqrt, step);
  const { cc, cs, c1, cmu, damps, chiN, weights, mueff } = options;
  const pathSigma = state.pathSigma.map((value, index) =>
    (1 - cs) * value + Math.sqrt(cs * (2 - cs) * mueff) * whitened[index]!,
  );
  const pathNorm = Math.hypot(...pathSigma);
  const generation = state.generation + 1;
  const hsig =
    pathNorm /
      Math.sqrt(1 - (1 - cs) ** (2 * generation)) /
      chiN <
    1.4 + 2 / (options.dimension + 1);
  const pathCovariance = state.pathCovariance.map((value, index) =>
    (1 - cc) * value + (hsig ? Math.sqrt(cc * (2 - cc) * mueff) * step[index]! : 0),
  );

  // C = (1-c1-cmu) C + c1 (pc pc^T + (1-hsig) cc(2-cc) C)
  //     + cmu Σ w_i z_i z_i^T
  const covariance = Array.from({ length: options.dimension }, () =>
    zeros(options.dimension),
  );
  for (let i = 0; i < options.dimension; i++) {
    for (let j = i; j < options.dimension; j++) {
      let value =
        (1 - c1 - cmu + (hsig ? 0 : c1 * cc * (2 - cc))) *
        state.covariance[i]![j]!;
      value += c1 * pathCovariance[i]! * pathCovariance[j]!;
      for (let k = 0; k < options.mu; k++) {
        const z = pending.displacements[selected[k]!]!;
        value += cmu * weights[k]! * z[i]! * z[j]!;
      }
      covariance[i]![j] = value;
      covariance[j]![i] = value;
    }
  }
  const sigma =
    state.sigma * Math.exp((cs / damps) * (pathNorm / chiN - 1));

  const bestIndex = selected[0]!;
  const candidateBest = {
    x: pending.candidates[bestIndex]!,
    f: values[bestIndex]!,
  };
  const best =
    state.best === null || candidateBest.f < state.best.f ? candidateBest : state.best;

  return {
    options,
    generation,
    evaluations: state.evaluations + options.lambda,
    mean,
    sigma,
    covariance,
    pathSigma,
    pathCovariance,
    rng: pending.rng,
    best,
  };
}

/** Ask + evaluate + tell, the convenient single-threaded stepping function. */
export function stepGeneration(
  state: CmaesState,
  evaluate: (candidate: readonly number[]) => number,
): { state: CmaesState; values: readonly number[] } {
  const pending = ask(state);
  const values = pending.candidates.map(evaluate);
  return { state: tell(state, pending, values), values };
}

/** JSON checkpoint: every field, including the PRNG state, is plain data. */
export function serializeState(state: CmaesState): string {
  return JSON.stringify(state);
}

export function deserializeState(json: string): CmaesState {
  const parsed = JSON.parse(json) as CmaesState;
  const n = parsed.options?.dimension;
  if (
    !Number.isSafeInteger(n) ||
    parsed.mean?.length !== n ||
    parsed.covariance?.length !== n ||
    parsed.pathSigma?.length !== n ||
    parsed.pathCovariance?.length !== n ||
    parsed.rng?.length !== 4 ||
    !Number.isFinite(parsed.sigma) ||
    !Number.isSafeInteger(parsed.generation) ||
    !Number.isSafeInteger(parsed.evaluations)
  ) {
    throw new RangeError("invalid CMA-ES checkpoint");
  }
  return parsed;
}
