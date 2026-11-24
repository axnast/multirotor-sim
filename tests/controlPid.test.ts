import { describe, expect, test } from "vitest";

import { emptyPidState, pidStep } from "../src/core/control/pid";
import type { PidConfig, PidResult, PidState } from "../src/core/control/types";

function config(
  gains: { kp: number; ki: number; kd: number },
  overrides: Partial<PidConfig> = {},
): PidConfig {
  return {
    gains,
    derivativeCutoff: 100,
    minOutput: -10,
    maxOutput: 10,
    ...overrides,
  };
}

function close(actual: number, expected: number, tol = 1e-12): void {
  expect(Math.abs(actual - expected)).toBeLessThanOrEqual(tol);
}

describe("PID", () => {
  test("P term alone is exactly kp·error", () => {
    const result = pidStep(emptyPidState(), 1.5, 0, 1e-3, config({ kp: 2, ki: 0, kd: 0 }));
    close(result.output, 3);
    expect(result.saturated).toBe(false);
  });

  test("I term integrates error over time", () => {
    let state = emptyPidState();
    const cfg = config({ kp: 0, ki: 2, kd: 0 });
    for (let i = 0; i < 100; i++) {
      state = pidStep(state, 1, 0, 0.01, cfg).state;
    }
    close(state.integral, 1, 1e-12);
    close(pidStep(state, 0, 0, 0.01, cfg).output, 2, 1e-12);
  });

  test("filtered derivative on measurement matches the analytic step response", () => {
    const cfg = config(
      { kp: 0, ki: 0, kd: 1 },
      { derivativeCutoff: 100, minOutput: -1e6, maxOutput: 1e6 },
    );
    const dt = 1e-3;
    let state = emptyPidState();
    state = pidStep(state, 0, 0, dt, cfg).state; // first sample: derivative = 0
    close(state.derivative, 0);

    const first = pidStep(state, 0, 1, dt, cfg); // step from 0 to 1
    const alpha = Math.exp(-100 * dt);
    const expectedFirst = -(1 - alpha) / dt;
    close(first.state.derivative, expectedFirst, 1e-9);
    close(first.output, expectedFirst, 1e-9);

    state = first.state;
    for (let i = 2; i <= 20; i++) {
      const result = pidStep(state, 0, 1, dt, cfg);
      close(result.state.derivative, expectedFirst * alpha ** (i - 1), 1e-9);
      state = result.state;
    }
    expect(state.derivative).not.toBe(0);
  });

  test("setpoint changes do not kick the derivative", () => {
    const cfg = config({ kp: 0, ki: 0, kd: 1 });
    let state = pidStep(emptyPidState(), 0, 0, 1e-3, cfg).state;
    state = pidStep(state, 0, 0, 1e-3, cfg).state;
    const before = state.derivative;
    const result = pidStep(state, 100, 0, 1e-3, cfg);
    close(result.state.derivative, before);
    close(result.output, before);
  });

  test("anti-windup holds the integrator under saturation and recovers", () => {
    const cfg = config(
      { kp: 1, ki: 100, kd: 0 },
      { minOutput: 0, maxOutput: 1 },
    );
    let state: PidState = emptyPidState();
    for (let i = 0; i < 500; i++) {
      const result = pidStep(state, 1, 0, 1e-3, cfg);
      expect(Number.isFinite(result.output)).toBe(true);
      expect(result.output).toBeLessThanOrEqual(1);
      state = result.state;
    }
    close(state.integral, 0, 1e-12);

    let recovery = state;
    for (let i = 0; i < 500; i++) {
      const result = pidStep(recovery, 0, 0, 1e-3, cfg);
      expect(Number.isFinite(result.output)).toBe(true);
      expect(result.output).toBeLessThanOrEqual(1);
      recovery = result.state;
    }
    close(pidStep(recovery, 0, 0, 1e-3, cfg).output, 0, 1e-12);
  });

  test("output is always clamped to the configured limits", () => {
    const cfg = config({ kp: 100, ki: 100, kd: 0 }, { minOutput: -2, maxOutput: 2 });
    for (const error of [-100, -1, 0, 1, 100]) {
      const result = pidStep(emptyPidState(), error, 0, 1e-3, cfg);
      expect(result.output).toBeGreaterThanOrEqual(-2);
      expect(result.output).toBeLessThanOrEqual(2);
    }
  });

  test("is deterministic", () => {
    const cfg = config({ kp: 3, ki: 0.5, kd: 0.02 });
    const run = (): readonly PidResult[] => {
      let state = emptyPidState();
      const results: PidResult[] = [];
      for (let i = 0; i < 200; i++) {
        const result = pidStep(state, 1, Math.sin(i * 0.01), 1e-3, cfg);
        state = result.state;
        results.push(result);
      }
      return results;
    };
    expect(run()).toEqual(run());
  });
});
