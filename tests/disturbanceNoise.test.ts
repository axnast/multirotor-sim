import { describe, expect, it } from "vitest";

import type { Measurement } from "../src/core/control/types";
import { QUAT_IDENTITY } from "../src/core/math/quat";
import { makeHex } from "../src/core/multirotor/config";
import { stepMultirotorDetailed } from "../src/core/multirotor/dynamics";
import {
  DEFAULT_SENSOR_PARAMS,
  gyroBiasStep,
  initialSensorState,
  noiseStep,
} from "../src/core/disturbance/noise";
import { measurementOf } from "../src/viewer/sim/runner";
import { MISSIONS, missionInitialMultirotor } from "../src/viewer/sim/mission";
import { SIM_DT } from "../src/viewer/sim/constants";

function noisyParams(overrides: Partial<typeof DEFAULT_SENSOR_PARAMS> = {}) {
  return { ...DEFAULT_SENSOR_PARAMS, enabled: true, ...overrides };
}

const TRUTH: Measurement = Object.freeze({
  position: Object.freeze([1, 2, 3]),
  velocity: Object.freeze([0.1, -0.2, 0.3]),
  orientation: Object.freeze(QUAT_IDENTITY),
  angularVelocity: Object.freeze([0.01, 0, -0.02]),
}) as Measurement;

describe("sensor noise", () => {
  it("matches the configured sigmas on every measurement channel", () => {
    // N=100k, SE(std)=sigma/sqrt(2N)=0.22% -> 4 sigma = 0.9%.
    const params = noisyParams({ gyroBiasWalkSigma: 0 });
    const count = 100_000;
    let state = initialSensorState(77);
    const sums = [0, 0, 0, 0];
    const squares = [0, 0, 0, 0];
    for (let i = 0; i < count; i++) {
      const result = noiseStep(state, params, TRUTH, SIM_DT);
      state = result.state;
      const measured = result.measurement;
      const values = [
        measured.position[0] - TRUTH.position[0],
        measured.velocity[0] - TRUTH.velocity[0],
        // For small angles q_x = r_x/2 to better than 1e-7 relative.
        2 * measured.orientation[1],
        measured.angularVelocity[0] - TRUTH.angularVelocity[0],
      ];
      for (let axis = 0; axis < values.length; axis++) {
        sums[axis]! += values[axis]!;
        squares[axis]! += values[axis]! * values[axis]!;
      }
    }
    const sigmas = [
      params.positionSigma,
      params.velocitySigma,
      params.attitudeSigmaRad,
      params.gyroWhiteSigma,
    ];
    for (let axis = 0; axis < sigmas.length; axis++) {
      const mean = sums[axis]! / count;
      const std = Math.sqrt(squares[axis]! / count - mean * mean);
      expect(Math.abs(std - sigmas[axis]!)).toBeLessThan(0.01 * sigmas[axis]!);
    }
  }, 30_000);

  it("grows the gyro bias walk variance linearly across 10,000 paths", () => {
    // Independent paths remove the OU/AR(1) effective-sample penalty.
    // For M paths, Var(bias_t) = sigma^2*t and SE(var) = var*sqrt(2/M)
    // (relative 1.41% at M=10000), so the bounds below are 4 sigma.
    const params = noisyParams();
    const paths = 10_000;
    const sigma = params.gyroBiasWalkSigma;
    const dt = 0.01;
    const shortSteps = 200; // 2 s
    const longSteps = 800; // 8 s
    let sumShort = 0;
    let sumShortSq = 0;
    let sumLong = 0;
    let sumLongSq = 0;
    for (let path = 0; path < paths; path++) {
      let state = initialSensorState(path + 1);
      for (let step = 0; step < shortSteps; step++) {
        state = gyroBiasStep(state, params, dt);
      }
      const short = state.gyroBias[0]!;
      sumShort += short;
      sumShortSq += short * short;
      for (let step = shortSteps; step < longSteps; step++) {
        state = gyroBiasStep(state, params, dt);
      }
      const long = state.gyroBias[0]!;
      sumLong += long;
      sumLongSq += long * long;
    }
    const shortMean = sumShort / paths;
    const longMean = sumLong / paths;
    const shortVariance = sumShortSq / paths - shortMean * shortMean;
    const longVariance = sumLongSq / paths - longMean * longMean;
    const shortExpected = sigma * sigma * shortSteps * dt;
    const longExpected = sigma * sigma * longSteps * dt;
    expect(
      Math.abs(shortVariance - shortExpected),
    ).toBeLessThan(4 * shortExpected * Math.sqrt(2 / paths));
    expect(
      Math.abs(longVariance - longExpected),
    ).toBeLessThan(4 * longExpected * Math.sqrt(2 / paths));
    // Reported for the record; the two absolute checks above are the test.
    expect(longVariance / shortVariance).toBeGreaterThan(2.7);
    expect(longVariance / shortVariance).toBeLessThan(5.3);
  }, 60_000);

  it("never alters the truth when the controller output is held fixed", () => {
    const config = makeHex({ groundPlane: true });
    const initial = missionInitialMultirotor(config, MISSIONS.hover);
    const commands = initial.motorSpeeds.map((omega, index) =>
      omega + 3 * Math.sin(index + 1),
    );
    let withNoise = initial;
    let withoutNoise = initial;
    let sensors = initialSensorState(42);
    const params = noisyParams();
    const frozenTruth = measurementOf(initial);
    expect(() => noiseStep(sensors, params, frozenTruth, SIM_DT)).not.toThrow();
    expect(measurementOf(initial)).toEqual(frozenTruth);

    for (let step = 0; step < 1_000; step++) {
      withNoise = stepMultirotorDetailed(
        withNoise, config, commands, SIM_DT,
      ).state;
      withoutNoise = stepMultirotorDetailed(
        withoutNoise, config, commands, SIM_DT,
      ).state;
      const result = noiseStep(
        sensors, params, measurementOf(withNoise), SIM_DT,
      );
      sensors = result.state;
    }
    expect(withNoise).toEqual(withoutNoise);
    expect(withNoise).not.toBe(withoutNoise);
  });

  it("returns the same objects without consuming draws when disabled", () => {
    const sensors = initialSensorState(1);
    const params = { ...DEFAULT_SENSOR_PARAMS, enabled: false };
    const result = noiseStep(sensors, params, TRUTH, SIM_DT);
    expect(result.state).toBe(sensors);
    expect(result.measurement).toBe(TRUTH);
  });
});
