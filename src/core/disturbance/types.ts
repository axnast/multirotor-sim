/**
 * Shared plain-data types for deterministic disturbances (Milestone 5).
 *
 * Nothing here is DOM-aware and nothing is hidden in a class: every random
 * stream, filter memory and process state is explicit and serialisable. The
 * same seed and the same sequence of inputs always produce the same bytes.
 *
 * This file is added incrementally: commits 1 and 2 define the PRNG and
 * sensor types; battery, payload, scenarios and the combined disturbance
 * boundary are added in later commits.
 */

import type { Vec3 } from "../math/vec3";

/**
 * xoshiro128** state plus one cached Box–Muller spare.
 *
 * `s` is four unsigned 32-bit words, in the order used by the next-state
 * function. `hasSpare` is true exactly when `spare` holds an unused Gaussian.
 */
export interface PrngState {
  readonly s: readonly [number, number, number, number];
  readonly spare: number;
  readonly hasSpare: boolean;
}

/** Deterministic 1-cosine gust, zero at both ends of its interval. */
export interface CosineGust {
  /** Start time in simulator seconds. */
  readonly startTime: number;
  /** Duration in seconds; must be positive. */
  readonly duration: number;
  /** Peak amplitude in m/s (reached at the midpoint). */
  readonly amplitude: number;
  /** World-frame unit direction. */
  readonly direction: Vec3;
}

/**
 * World-frame wind: a constant mean, a per-axis Ornstein–Uhlenbeck gust and
 * an optional discrete 1-cosine event.
 */
export interface WindParams {
  readonly enabled: boolean;
  /** Steady mean velocity in the world frame (m/s). */
  readonly mean: Vec3;
  /** Stationary standard deviation of the OU process per axis (m/s). */
  readonly gustSigma: Vec3;
  /** OU correlation time in seconds; `a = exp(-dt/Tc)`. */
  readonly correlationTime: number;
  /** null when no discrete gust is scheduled. */
  readonly cosineGust: CosineGust | null;
}

/** Wind process memory: current OU state, PRNG state and process time. */
export interface WindState {
  readonly prng: PrngState;
  /** Current OU gust vector in the world frame (m/s). */
  readonly gust: Vec3;
  /** Start time of the next step, in seconds. */
  readonly time: number;
}

export interface WindStepResult {
  readonly state: WindState;
  /** Total wind velocity to pass to `Environment.windVelocity` (m/s). */
  readonly wind: Vec3;
}
