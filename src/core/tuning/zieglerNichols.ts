/**
 * Pure Ziegler-Nichols helpers: oscillation detection, ultimate-gain search
 * and open-loop reaction-curve identification. The simulator-specific loops
 * that feed samples into these functions live in scripts/zn.ts.
 *
 * Oscillation detector: after removing the mean, at least three complete
 * cycles (six zero crossings with hysteresis at 10% of the peak amplitude),
 * half-period drift < 5% and amplitude > 1% of the commanded step. These are
 * fixed identification rules, not tuning tolerances.
 */

export interface Oscillation {
  readonly oscillating: boolean;
  readonly halfPeriods: readonly number[];
  readonly period: number;
  readonly amplitude: number;
  /** Late/early half-cycle peak ratio: <1 decaying, >1 growing. */
  readonly amplitudeTrend: number;
}

export interface UltimateGain {
  readonly kp: number;
  readonly tu: number;
  readonly amplitude: number;
  /** Late/early half-cycle peak ratio at the ultimate crossover. */
  readonly trend: number;
  readonly halfPeriods: readonly number[];
}

export interface ReactionCurve {
  /** Steady slope of the response per unit input. */
  readonly slope: number;
  /** Apparent dead time from the tangent back-extrapolation (s). */
  readonly deadTime: number;
}

export interface ZngPid {
  readonly kp: number;
  readonly ki: number;
  readonly kd: number;
}

export function detectOscillation(
  samples: readonly number[],
  dt: number,
  stepScale: number,
): Oscillation {
  if (samples.length < 10 || stepScale <= 0) {
    return { oscillating: false, halfPeriods: [], period: 0, amplitude: 0, amplitudeTrend: 0 };
  }
  // Skip the first third of the response: it contains the startup transient,
  // whose single large overshoot would otherwise masquerade as a cycle.
  const start = Math.floor(samples.length / 3);
  const settled = samples.slice(start);
  const mean = settled.reduce((sum, value) => sum + value, 0) / settled.length;
  let min = Infinity;
  let max = -Infinity;
  for (const sample of settled) {
    if (!Number.isFinite(sample)) {
      return { oscillating: false, halfPeriods: [], period: 0, amplitude: 0, amplitudeTrend: 0 };
    }
    min = Math.min(min, sample);
    max = Math.max(max, sample);
  }
  const amplitude = (max - min) / 2;
  if (!(amplitude > 0.01 * stepScale)) {
    return { oscillating: false, halfPeriods: [], period: 0, amplitude, amplitudeTrend: 0 };
  }

  const hysteresis = 0.1 * amplitude;
  const crossings: number[] = [];
  let sign = 0;
  for (let index = start; index < samples.length; index++) {
    const value = samples[index]! - mean;
    if (value > hysteresis && sign <= 0) {
      crossings.push(index * dt);
      sign = 1;
    } else if (value < -hysteresis && sign >= 0) {
      crossings.push(index * dt);
      sign = -1;
    }
  }
  const halfPeriods: number[] = [];
  // The first crossing is partial (the settle window starts mid-cycle).
  for (let index = 2; index < crossings.length; index++) {
    halfPeriods.push(crossings[index]! - crossings[index - 1]!);
  }
  if (halfPeriods.length < 6) {
    return { oscillating: false, halfPeriods, period: 0, amplitude, amplitudeTrend: 0 };
  }
  const average = halfPeriods.reduce((sum, value) => sum + value, 0) / halfPeriods.length;
  const drift = (Math.max(...halfPeriods) - Math.min(...halfPeriods)) / average;
  if (!(drift < 0.05)) {
    return { oscillating: false, halfPeriods, period: 0, amplitude, amplitudeTrend: 0 };
  }
  // Trend of successive half-cycle peaks: the ultimate gain is where this
  // crosses 1 (decaying below it, growing above it).
  const peaks: number[] = [];
  for (let index = 2; index < crossings.length - 1; index++) {
    let peak = 0;
    for (
      let sample = Math.ceil(crossings[index]! / dt);
      sample < crossings[index + 1]! / dt;
      sample++
    ) {
      peak = Math.max(peak, Math.abs(samples[sample]! - mean));
    }
    peaks.push(peak);
  }
  const earlyPeak = (peaks[0]! + peaks[1]!) / 2;
  const latePeak = (peaks[peaks.length - 2]! + peaks[peaks.length - 1]!) / 2;
  const amplitudeTrend = earlyPeak > 0 ? latePeak / earlyPeak : 0;
  return { oscillating: true, halfPeriods, period: 2 * average, amplitude, amplitudeTrend };
}

/** Geometric sweep then bisection; `evaluate` must return the loop response. */
export function findUltimateGain(
  evaluate: (kp: number) => readonly number[],
  options: {
    readonly minKp: number;
    readonly maxKp: number;
    readonly dt: number;
    readonly stepScale: number;
    readonly sweepSteps?: number;
    readonly refineSteps?: number;
  },
): {
  gain: UltimateGain | null;
  trace: readonly {
    kp: number;
    oscillating: boolean;
    period: number;
    amplitude: number;
    amplitudeTrend: number;
  }[];
} {
  const sweepSteps = options.sweepSteps ?? 48;
  const refineSteps = options.refineSteps ?? 12;
  const trace: {
    kp: number;
    oscillating: boolean;
    period: number;
    amplitude: number;
    amplitudeTrend: number;
  }[] = [];
  let low = options.minKp;
  let high = options.maxKp;
  let found = false;
  const measure = (kp: number) => {
    const response = detectOscillation(evaluate(kp), options.dt, options.stepScale);
    trace.push({
      kp,
      oscillating: response.oscillating,
      period: response.period,
      amplitude: response.amplitude,
      amplitudeTrend: response.amplitudeTrend,
    });
    return response;
  };

  for (let index = 0; index <= sweepSteps; index++) {
    const kp = low * (options.maxKp / low) ** (index / sweepSteps);
    const response = measure(kp);
    if (response.oscillating && response.amplitudeTrend >= 1) {
      high = kp;
      found = true;
      break;
    }
    low = kp;
  }
  if (!found) return { gain: null, trace };

  let ultimate = measure(high);
  for (let index = 0; index < refineSteps; index++) {
    const kp = Math.sqrt(low * high);
    const response = measure(kp);
    if (response.oscillating && response.amplitudeTrend >= 1) {
      high = kp;
      ultimate = response;
    } else {
      low = kp;
    }
  }
  return {
    gain: {
      kp: high,
      tu: ultimate.period,
      amplitude: ultimate.amplitude,
      trend: ultimate.amplitudeTrend,
      halfPeriods: ultimate.halfPeriods,
    },
    trace,
  };
}

/** Tangent-at-steepest-slope identification of apparent lag for a step. */
export function reactionCurve(
  response: readonly number[],
  dt: number,
  inputStep: number,
  startIndex = 0,
): ReactionCurve {
  if (inputStep === 0 || response.length < startIndex + 3) {
    throw new RangeError("reaction curve needs a non-zero step and samples");
  }
  const baseline = response[startIndex] ?? 0;
  let bestIndex = startIndex + 1;
  let bestSlope = 0;
  for (let index = startIndex + 1; index < response.length - 1; index++) {
    const slope = (response[index + 1]! - response[index - 1]!) / (2 * dt);
    if (Math.abs(slope) > Math.abs(bestSlope)) {
      bestSlope = slope;
      bestIndex = index;
    }
  }
  if (bestSlope === 0) throw new RangeError("reaction curve has no measurable slope");
  const rise = (response[bestIndex]! - baseline) / inputStep;
  const time = bestIndex * dt;
  return {
    slope: bestSlope / inputStep,
    deadTime: Math.max(0, time - rise / (bestSlope / inputStep)),
  };
}

/** Classic ZN closed-loop rules, used verbatim by the baseline script. */
export function znPid(ku: number, tu: number): ZngPid {
  return { kp: 0.6 * ku, ki: (0.6 * ku) / (0.5 * tu), kd: 0.6 * ku * (0.125 * tu) };
}

export function znPi(ku: number, tu: number): { kp: number; ki: number } {
  return { kp: 0.45 * ku, ki: (0.45 * ku) / (0.83 * tu) };
}

/** Open-loop reaction-curve rule for the rate loop (integrator + lag). */
export function znReactionPid(slope: number, deadTime: number): ZngPid {
  const kp = 1.2 / (slope * deadTime);
  return { kp, ki: kp / (2 * deadTime), kd: kp * (0.5 * deadTime) };
}

/** Classic reaction-curve PI rule (no derivative) for the rate loop. */
export function znReactionPi(slope: number, deadTime: number): ZngPid {
  const kp = 0.9 / (slope * deadTime);
  return { kp, ki: kp / (3.33 * deadTime), kd: 0 };
}
