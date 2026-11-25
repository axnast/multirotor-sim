/**
 * World-frame wind: steady mean + per-axis Ornstein–Uhlenbeck gust + optional
 * discrete 1-cosine event.
 *
 * Exact OU discretisation (zero-order held over one simulator step):
 *
 *     a = exp(-dt / Tc)
 *     x' = a*x + sigma*sqrt(1 - a^2)*N(0,1)
 *
 * so the stationary variance is sigma^2 for every dt. The process is advanced
 * once per core step and the resulting vector is passed to the existing
 * `Environment.windVelocity`; air-relative drag is unchanged.
 */

import { vec3Add, vec3Scale, type Vec3 } from "../math/vec3";
import { createPrng, nextGaussian3 } from "./prng";
import type {
  CosineGust,
  WindParams,
  WindState,
  WindStepResult,
} from "./types";

function assertFinite(name: string, value: number): void {
  if (!Number.isFinite(value)) {
    throw new RangeError(`${name} must be a finite number`);
  }
}

function assertVec3(name: string, v: Vec3): void {
  for (let axis = 0; axis < 3; axis++) {
    assertFinite(`${name}[${axis}]`, v[axis]!);
  }
}

function assertNonNegativeVec3(name: string, v: Vec3): void {
  assertVec3(name, v);
  for (let axis = 0; axis < 3; axis++) {
    if (v[axis]! < 0) {
      throw new RangeError(`${name}[${axis}] must be non-negative`);
    }
  }
}

/** Fresh OU state at `gust = [0,0,0]` and process time zero. */
export function initialWindState(
  seed: number,
  gust: Vec3 = [0, 0, 0],
): WindState {
  return { prng: createPrng(seed), gust: [...gust], time: 0 };
}

export function validateWindParams(params: WindParams): void {
  if (typeof params.enabled !== "boolean") {
    throw new RangeError("wind.enabled must be a boolean");
  }
  assertVec3("wind.mean", params.mean);
  assertNonNegativeVec3("wind.gustSigma", params.gustSigma);
  assertFinite("wind.correlationTime", params.correlationTime);
  if (params.correlationTime <= 0) {
    throw new RangeError("wind.correlationTime must be positive");
  }
  const gust = params.cosineGust;
  if (gust !== null) {
    validateCosineGust(gust);
  }
}

export function validateCosineGust(gust: CosineGust): void {
  assertFinite("cosineGust.startTime", gust.startTime);
  assertFinite("cosineGust.duration", gust.duration);
  assertFinite("cosineGust.amplitude", gust.amplitude);
  assertVec3("cosineGust.direction", gust.direction);
  if (gust.duration <= 0) {
    throw new RangeError("cosineGust.duration must be positive");
  }
}

/**
 * `A/2 * (1 - cos(2*pi*phase))` inside [start, start+duration], else zero.
 * At phase 0 and phase 1 the value is exactly zero.
 */
export function cosineGustAt(time: number, gust: CosineGust): number {
  if (gust.duration <= 0) return 0;
  const elapsed = time - gust.startTime;
  if (elapsed < 0 || elapsed >= gust.duration) return 0;
  const phase = elapsed / gust.duration;
  return 0.5 * gust.amplitude * (1 - Math.cos(2 * Math.PI * phase));
}

/**
 * Advance the wind process by `dt`. Disabled wind returns the *same state*
 * and the shared `STILL_AIR`-equivalent vector without consuming any PRNG
 * draws, so a zero-disturbance flight stays bit-identical to pre-M5 code.
 */
export function windStep(
  state: WindState,
  params: WindParams,
  dt: number,
): WindStepResult {
  validateWindParams(params);
  assertFinite("dt", dt);
  if (dt <= 0) throw new RangeError("dt must be positive");
  if (!params.enabled) {
    return { state, wind: [0, 0, 0] };
  }

  const a = Math.exp(-dt / params.correlationTime);
  const innovation = Math.sqrt(Math.max(0, 1 - a * a));
  const draw = nextGaussian3(state.prng);
  const gust: Vec3 = [
    a * state.gust[0]! + params.gustSigma[0]! * innovation * draw.values[0],
    a * state.gust[1]! + params.gustSigma[1]! * innovation * draw.values[1],
    a * state.gust[2]! + params.gustSigma[2]! * innovation * draw.values[2],
  ];

  const event = params.cosineGust;
  const eventVector: Vec3 = event === null
    ? [0, 0, 0]
    : vec3Scale(event.direction, cosineGustAt(state.time, event));
  return {
    state: { prng: draw.state, gust, time: state.time + dt },
    wind: vec3Add(vec3Add(params.mean, gust), eventVector),
  };
}
