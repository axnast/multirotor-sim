/**
 * The M6 gain parameterization: 9 positive gains, all searched in log space.
 * The release scheme (integralSeparation is the only tunable gate constant),
 * integralDwell, integralTrimGain, integralReleaseGuard, every limit and the
 * rate derivative cutoff are deliberately NOT in this vector. `withGains`
 * copies only these nine fields into a ControllerParams, so all fixed fields
 * remain exactly as `defaultControllerParams` set them.
 */

import type {
  ControllerParams,
  PidGains,
} from "../control/types";

export interface GainBounds {
  readonly lower: number;
  readonly upper: number;
}

/** Order of the nine search dimensions. Keep in sync with GAIN_BOUNDS. */
export const GAIN_KEYS = [
  "position.kp",
  "position.ki",
  "position.kd",
  "attitude.tiltGain",
  "attitude.yawGain",
  "rate.kp",
  "rate.ki",
  "rate.kd",
  "position.integralSeparation",
] as const;

export type GainKey = (typeof GAIN_KEYS)[number];

/** Bounds around `defaultControllerParams` values (documented data). */
export const GAIN_BOUNDS: Readonly<Record<GainKey, GainBounds>> = {
  "position.kp": { lower: 0.05, upper: 1 },
  "position.ki": { lower: 0.001, upper: 0.2 },
  "position.kd": { lower: 0.1, upper: 4 },
  "attitude.tiltGain": { lower: 0.2, upper: 3 },
  "attitude.yawGain": { lower: 0.1, upper: 3 },
  "rate.kp": { lower: 0.5, upper: 10 },
  "rate.ki": { lower: 0.01, upper: 3 },
  "rate.kd": { lower: 0.002, upper: 0.3 },
  "position.integralSeparation": { lower: 0.02, upper: 1 },
};

export const DIMENSION = GAIN_KEYS.length;

function clip(value: number, lower: number, upper: number): number {
  return Math.min(upper, Math.max(lower, value));
}

/** Pure: a vector of log coordinates -> nine bounded, positive gain values. */
export function decode(values: readonly number[]): number[] {
  if (values.length !== DIMENSION) {
    throw new RangeError(`expected ${DIMENSION} gain coordinates`);
  }
  return GAIN_KEYS.map((key, index) => {
    const { lower, upper } = GAIN_BOUNDS[key];
    const coordinate = Number.isFinite(values[index]!) ? values[index]! : 0;
    return clip(Math.exp(coordinate), lower, upper);
  });
}

/** Pure inverse of `decode` for a value inside the documented bounds. */
export function encode(gains: readonly number[]): number[] {
  if (gains.length !== DIMENSION) {
    throw new RangeError(`expected ${DIMENSION} gains`);
  }
  return GAIN_KEYS.map((key, index) => {
    const { lower, upper } = GAIN_BOUNDS[key];
    const value = gains[index]!;
    if (!Number.isFinite(value) || value <= 0) {
      throw new RangeError(`gain ${key} must be finite and positive`);
    }
    return Math.log(clip(value, lower, upper));
  });
}

/** Default gains in log coordinates; a convenient CMA-ES starting point. */
export function defaultsToVector(params: ControllerParams): number[] {
  return encode([
    params.position.gains.kp,
    params.position.gains.ki,
    params.position.gains.kd,
    params.attitude.tiltGain,
    params.attitude.yawGain,
    params.rate.gains.kp,
    params.rate.gains.ki,
    params.rate.gains.kd,
    params.position.integralSeparation,
  ]);
}

/** Apply a decoded vector without touching any fixed field. */
export function withGains(
  params: ControllerParams,
  gains: readonly number[],
): ControllerParams {
  const values = decode(gains);
  const position: PidGains = {
    kp: values[0]!,
    ki: values[1]!,
    kd: values[2]!,
  };
  const rate: PidGains = {
    kp: values[5]!,
    ki: values[6]!,
    kd: values[7]!,
  };
  return {
    ...params,
    position: {
      ...params.position,
      gains: position,
      integralSeparation: values[8]!,
    },
    attitude: {
      ...params.attitude,
      tiltGain: values[3]!,
      yawGain: values[4]!,
    },
    rate: { ...params.rate, gains: rate },
  };
}

/** Fixed fields asserted against the code's defaults (see docs/tuning.md). */
export function assertFixedDefaults(params: ControllerParams): void {
  if (
    params.position.integralDwell !== 5 ||
    params.position.integralTrimGain !== 3 ||
    params.position.integralReleaseGuard !== 2 ||
    params.rate.derivativeCutoff !== 50
  ) {
    throw new RangeError("fixed controller defaults must not be tuned");
  }
}
