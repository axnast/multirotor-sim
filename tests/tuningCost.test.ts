import { describe, expect, it } from "vitest";

import { defaultControllerParams } from "../src/core/control/controller";
import type { MultirotorState } from "../src/core/multirotor/dynamics";
import { QUAT_IDENTITY, quatFromAxisAngle } from "../src/core/math/quat";
import { makeHex } from "../src/core/multirotor/config";
import {
  COST_KEYS,
  NONFINITE_COST,
  TERM_WEIGHTS,
  createCostSuite,
  gradedPenalty,
  rollout,
  tiltOf,
} from "../src/core/tuning/cost";
import {
  DIMENSION,
  assertFixedDefaults,
  decode,
  defaultsToVector,
  encode,
  withGains,
} from "../src/core/tuning/params";

const config = makeHex({ groundPlane: true });
const base = defaultControllerParams(config);
// Structural test only: production always uses the full 120 s suite.
const suite = createCostSuite(config, base, 4_000);
const SEEDS = [1, 2] as const;

describe("M6 gain parameterization", () => {
  it("has nine log-scaled dimensions and round-trips the defaults", () => {
    expect(DIMENSION).toBe(9);
    const vector = defaultsToVector(base);
    const gains = decode(vector);
    expect(encode(gains)).toEqual(vector);
    expect(gains[0]).toBeCloseTo(0.25, 9);
    expect(gains[8]).toBeCloseTo(0.1, 9);
  });

  it("clips out-of-range coordinates and leaves fixed fields untouched", () => {
    const gains = decode(Array(DIMENSION).fill(100));
    expect(gains[0]).toBeCloseTo(1, 9);
    expect(gains[8]).toBeCloseTo(1, 9);
    const tuned = withGains(base, Array(DIMENSION).fill(1));
    assertFixedDefaults(tuned);
    expect(tuned.position.integralDwell).toBe(base.position.integralDwell);
    expect(tuned.position.integralTrimGain).toBe(base.position.integralTrimGain);
    expect(tuned.position.integralReleaseGuard).toBe(base.position.integralReleaseGuard);
    expect(tuned.rate.derivativeCutoff).toBe(base.rate.derivativeCutoff);
    expect(tuned.position.maxTilt).toBe(base.position.maxTilt);
    expect(tuned.rate.maxAngularAcceleration).toEqual(base.rate.maxAngularAcceleration);
  });

  it("matches the fixed defaults verified in the code", () => {
    expect(base.position.integralDwell).toBe(5);
    expect(base.position.integralTrimGain).toBe(3);
    expect(base.position.integralReleaseGuard).toBe(2);
    expect(base.rate.derivativeCutoff).toBe(50);
  });
});

describe("M6 tuning cost", () => {
  it("scores the default gains at exactly 1.000 with weights summing to one", () => {
    expect(COST_KEYS.reduce((sum, key) => sum + TERM_WEIGHTS[key], 0)).toBeCloseTo(1, 12);
    const result = suite.evaluate(base, SEEDS);
    expect(result.total).toBeCloseTo(1, 12);
    expect(result.terms.every((term) => Math.abs(term.normalized - 1) < 1e-12)).toBe(true);
  }, 30_000);

  it("scores a deliberately bad gain set worse", () => {
    const bad = withGains(base, [
      Math.log(1),
      Math.log(0.2),
      Math.log(0.1),
      Math.log(3),
      Math.log(3),
      Math.log(0.5),
      Math.log(3),
      Math.log(0.002),
      Math.log(1),
    ]);
    expect(suite.evaluate(bad, SEEDS).total).toBeGreaterThan(suite.evaluate(base, SEEDS).total);
  }, 30_000);

  it("returns a finite, severity-ordered penalty for hard failures", () => {
    expect(gradedPenalty(0)).toBe(100);
    expect(gradedPenalty(0.1)).toBeGreaterThan(gradedPenalty(0));
    expect(gradedPenalty(0.5)).toBeGreaterThan(gradedPenalty(0.1));
    expect(gradedPenalty(0, true)).toBe(NONFINITE_COST);
    expect(gradedPenalty(0.5)).toBeLessThan(NONFINITE_COST);

    // Very weak position/rate gains let the drone fall: ground contact in
    // flight is a hard failure in every scenario except +20% mass.
    const vector = [
      Math.log(0.05), Math.log(0.2), Math.log(4), Math.log(3), Math.log(3),
      Math.log(0.5), Math.log(3), Math.log(0.002), Math.log(1),
    ];
    const result = suite.evaluate(withGains(base, vector), SEEDS);
    expect(Number.isFinite(result.total)).toBe(true);
    expect(result.total).toBeGreaterThan(1);
    expect(result.terms.some((term) => term.reason !== null)).toBe(true);
  }, 30_000);

  it("aggregates over every supplied seed, not just the first", () => {
    const bad = withGains(base, [
      Math.log(1), Math.log(0.2), Math.log(0.1), Math.log(3), Math.log(3),
      Math.log(0.5), Math.log(3), Math.log(0.002), Math.log(1),
    ]);
    const onlySeed1 = suite.evaluate(bad, [1]).total;
    const bothSeeds = suite.evaluate(bad, [1, 2]).total;
    expect(bothSeeds).not.toBe(onlySeed1);
    expect(suite.evaluate(base, [1, 2, 3, 4, 5]).total).toBeCloseTo(1, 12);
    // Five full-horizon cost suites; under full-suite worker contention this
    // takes ~30-40 s. This is a harness timeout, not a physics tolerance.
  }, 120_000);

  it("is deterministic for a fixed seed subset", () => {
    const first = suite.evaluate(base, SEEDS);
    const second = suite.evaluate(base, SEEDS);
    expect(second).toEqual(first);
  }, 30_000);

  it("keeps latency opt-in and byte-identical at zero", () => {
    const plain = rollout(config, base, 1, "step", 2_000);
    const zero = rollout(config, base, 1, "step", 2_000, null, {
      measurementLatencySteps: 0,
      sensorScale: 1,
      windDirectionDeg: 0,
    });
    expect(zero).toEqual(plain);
    const delayed = rollout(config, base, 1, "step", 2_000, null, {
      measurementLatencySteps: 5,
    });
    expect(delayed).not.toEqual(plain);
  }, 30_000);

  it("computes max tilt from [w,x,y,z] quaternions", () => {
    const at = (orientation: MultirotorState["body"]["orientation"]) =>
      tiltOf({
        body: {
          position: [0, 0, 5],
          velocity: [0, 0, 0],
          orientation,
          angularVelocity: [0, 0, 0],
        },
        motorSpeeds: [0, 0, 0, 0, 0, 0],
      });
    expect(at(QUAT_IDENTITY)).toBe(0);
    expect(at(quatFromAxisAngle([1, 0, 0], 0.3))).toBeCloseTo(0.3, 12);
    expect(at(quatFromAxisAngle([0, 1, 0], 0.8))).toBeCloseTo(0.8, 12);
  });
});
