import { describe, expect, it } from "vitest";

import {
  detectOscillation,
  findUltimateGain,
  reactionCurve,
  znPi,
  znPid,
  znReactionPi,
  znReactionPid,
} from "../src/core/tuning/zieglerNichols";

const DT = 0.001;
const TAU = 0.1;
const DELAY_STEPS = 50;
const L = DELAY_STEPS * DT;

/** Synthetic second-order-plus-lag plant: e^{-Ls}/(tau s + 1)^2. */
function plantResponse(kp: number): number[] {
  const buffer = Array(DELAY_STEPS).fill(0);
  let index = 0;
  let x1 = 0;
  let x2 = 0;
  const output: number[] = [];
  for (let step = 0; step < 4_000; step++) {
    const delayed = buffer[index]!;
    buffer[index] = kp * (1 - x2);
    index = (index + 1) % DELAY_STEPS;
    x1 += (DT / TAU) * (delayed - x1);
    x2 += (DT / TAU) * (x1 - x2);
    output.push(x2);
  }
  return output.slice(1_000);
}

function analyticUltimate(): { ku: number; tu: number } {
  let low = 0;
  let high = 50;
  for (let index = 0; index < 80; index++) {
    const omega = (low + high) / 2;
    if (2 * Math.atan(omega * TAU) + omega * L < Math.PI) low = omega;
    else high = omega;
  }
  const omega = (low + high) / 2;
  return { ku: 1 + (omega * TAU) ** 2, tu: (2 * Math.PI) / omega };
}

describe("Ziegler-Nichols helpers", () => {
  it("finds the analytic ultimate gain and period of a synthetic SOPDT plant", () => {
    const analytic = analyticUltimate();
    const found = findUltimateGain(plantResponse, {
      minKp: 0.1,
      maxKp: 50,
      dt: DT,
      stepScale: 1,
    });
    expect(found.gain).not.toBeNull();
    expect(found.gain!.kp / analytic.ku).toBeCloseTo(1, 1);
    expect(found.gain!.tu / analytic.tu).toBeCloseTo(1, 1);
  }, 30_000);

  it("rejects a non-oscillating response", () => {
    const flat = Array(2_000).fill(0.5);
    expect(detectOscillation(flat, DT, 1).oscillating).toBe(false);
  });

  it("identifies slope and dead time from an open-loop ramp", () => {
    const slope = 2;
    const deadTime = 0.05;
    const response = Array.from({ length: 1_000 }, (_, index) => {
      const time = index * DT;
      return time < deadTime ? 0 : slope * (time - deadTime);
    });
    const curve = reactionCurve(response, DT, 1);
    expect(curve.slope / slope).toBeCloseTo(1, 2);
    expect(Math.abs(curve.deadTime - deadTime)).toBeLessThan(5 * DT);
  });

  it("applies the classic ZN rules verbatim", () => {
    expect(znPid(4, 0.5)).toEqual({ kp: 2.4, ki: 9.6, kd: 0.15 });
    expect(znPi(4, 0.5).kp).toBeCloseTo(1.8, 12);
    expect(znReactionPid(4, 0.05).kp).toBeCloseTo(6, 12);
    expect(znReactionPi(4, 0.05).kp).toBeCloseTo(4.5, 12);
    expect(znReactionPi(4, 0.05).kd).toBe(0);
  });
});
