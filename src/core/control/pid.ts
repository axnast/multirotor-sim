/**
 * Single-axis PID with a filtered derivative and conditional-integration
 * anti-windup.
 *
 * - The derivative is taken on the **measurement**, not on the error, so a
 *   setpoint step cannot produce a derivative kick.
 * - The raw derivative `-d(measurement)/dt` is low-pass filtered with
 *   `alpha = exp(-cutoff·dt)`; the filter memory lives in `PidState`.
 * - Conditional integration: when the unclamped output would exceed a limit
 *   and the error would push it further past that limit, the integrator is
 *   held. Otherwise it integrates normally, which allows recovery without a
 *   large overshoot once the error reverses.
 *
 * Pure data-in/data-out: no closures, no hidden state, no mutation.
 */

import type { PidConfig, PidResult, PidState } from "./types";

function assertFinite(name: string, value: number): void {
  if (!Number.isFinite(value)) {
    throw new RangeError(`${name} must be a finite number`);
  }
}

function assertValidState(state: PidState): void {
  assertFinite("state.integral", state.integral);
  assertFinite("state.derivative", state.derivative);
  if (state.lastMeasurement !== null) {
    assertFinite("state.lastMeasurement", state.lastMeasurement);
  }
}

function assertValidConfig(config: PidConfig): void {
  assertFinite("gains.kp", config.gains.kp);
  assertFinite("gains.ki", config.gains.ki);
  assertFinite("gains.kd", config.gains.kd);
  assertFinite("derivativeCutoff", config.derivativeCutoff);
  assertFinite("minOutput", config.minOutput);
  assertFinite("maxOutput", config.maxOutput);
  if (config.derivativeCutoff <= 0) {
    throw new RangeError("derivativeCutoff must be positive");
  }
  if (config.minOutput >= config.maxOutput) {
    throw new RangeError("minOutput must be less than maxOutput");
  }
}

/** A fresh integrator/filter state; the first derivative is defined as zero. */
export function emptyPidState(): PidState {
  return { integral: 0, derivative: 0, lastMeasurement: null };
}

/**
 * Advance one PID step. `measurement` is the raw process value (not the error);
 * its derivative is filtered. Returns the new state and the clamped output.
 */
export function pidStep(
  state: PidState,
  error: number,
  measurement: number,
  dt: number,
  config: PidConfig,
): PidResult {
  assertValidState(state);
  assertValidConfig(config);
  assertFinite("error", error);
  assertFinite("measurement", measurement);
  assertFinite("dt", dt);
  if (dt <= 0) {
    throw new RangeError("dt must be positive");
  }

  const alpha = Math.exp(-config.derivativeCutoff * dt);
  const rawDerivative =
    state.lastMeasurement === null
      ? 0
      : -(measurement - state.lastMeasurement) / dt;
  const derivative = alpha * state.derivative + (1 - alpha) * rawDerivative;

  const candidateIntegral = state.integral + error * dt;
  const p = config.gains.kp * error;
  const d = config.gains.kd * derivative;
  const candidate =
    p + config.gains.ki * candidateIntegral + d;
  const saturating =
    candidate > config.maxOutput || candidate < config.minOutput;
  const pushFurther =
    (candidate > config.maxOutput && error > 0) ||
    (candidate < config.minOutput && error < 0);
  const integral = pushFurther ? state.integral : candidateIntegral;

  const unclamped = p + config.gains.ki * integral + d;
  const output = Math.min(
    config.maxOutput,
    Math.max(config.minOutput, unclamped),
  );
  const saturated = saturating || unclamped !== output;

  return {
    state: { integral, derivative, lastMeasurement: measurement },
    output,
    saturated,
  };
}
