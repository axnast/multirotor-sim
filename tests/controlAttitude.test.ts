import { describe, expect, test } from "vitest";

import { attitudeError, attitudeRateCommand, desiredAttitude } from "../src/core/control/attitude";
import {
  defaultControllerParams,
  initialControllerState,
  rateStep,
} from "../src/core/control/controller";
import type { AxisPidState, Measurement } from "../src/core/control/types";
import {
  QUAT_IDENTITY,
  quatFromAxisAngle,
  quatMultiply,
  quatRotateVector,
  quatToEulerZYX,
  type Quat,
} from "../src/core/math/quat";
import { vec3Length, type Vec3 } from "../src/core/math/vec3";
import {
  hoverMotorSpeeds,
  makeHex,
  makeOcto,
  makeQuad,
} from "../src/core/multirotor/config";
import { stepMultirotor, type MultirotorState } from "../src/core/multirotor/dynamics";
import { mixDetailed, type Wrench } from "../src/core/multirotor/mixer";

const DT = 1e-3;
const DEG = Math.PI / 180;
const config = makeHex();
const params = defaultControllerParams(config);
const weight = config.mass * 9.81;

function measurementOf(state: MultirotorState): Measurement {
  return {
    position: state.body.position,
    velocity: state.body.velocity,
    orientation: state.body.orientation,
    angularVelocity: state.body.angularVelocity,
  };
}

function hoverState(): MultirotorState {
  return {
    body: {
      position: [0, 0, 5],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    },
    motorSpeeds: hoverMotorSpeeds(config),
  };
}

function advance(
  state: MultirotorState,
  rateState: AxisPidState,
  desiredRate: Vec3,
): { state: MultirotorState; rateState: AxisPidState; saturated: boolean } {
  const command = rateStep(rateState, desiredRate, measurementOf(state), params, DT);
  const wrench: Wrench = [
    weight,
    command.torque[0],
    command.torque[1],
    command.torque[2],
  ];
  const mixed = mixDetailed(config, wrench);
  return {
    state: stepMultirotor(state, config, mixed.motorSpeeds, DT),
    rateState: command.state,
    saturated: mixed.saturated,
  };
}

function errorAngle(current: Quat, target: Quat): number {
  const w = Math.min(1, Math.abs(attitudeError(current, target)[0]));
  return 2 * Math.acos(w);
}

function runRateStep(steps: number): {
  response: number[];
  riseTime: number;
  overshoot: number;
  settlingTime: number;
} {
  let state = hoverState();
  let rateState = initialControllerState().rate;
  const response: number[] = [];
  const target: Vec3 = [1, 0, 0];
  for (let i = 0; i < steps; i++) {
    const next = advance(state, rateState, target);
    state = next.state;
    rateState = next.rateState;
    response.push(state.body.angularVelocity[0]);
  }
  const first10 = response.findIndex((v) => v >= 0.1);
  const first90 = response.findIndex((v) => v >= 0.9);
  let lastOutside = 0;
  for (let i = 0; i < response.length; i++) {
    if (Math.abs(response[i]! - 1) > 0.05) lastOutside = i;
  }
  return {
    response,
    riseTime: (first90 - first10) * DT,
    overshoot: Math.max(0, Math.max(...response) - 1),
    settlingTime: lastOutside * DT,
  };
}

function runAttitudeStep(target: Quat, steps: number): {
  finalError: number;
  path: number;
  saturatedCount: number;
  rollHistory: number[];
} {
  let state = hoverState();
  let rateState = initialControllerState().rate;
  let path = 0;
  let saturatedCount = 0;
  const rollHistory: number[] = [];
  for (let i = 0; i < steps; i++) {
    const desired = attitudeRateCommand(
      state.body.orientation,
      target,
      params.attitude,
    );
    const next = advance(state, rateState, desired);
    state = next.state;
    rateState = next.rateState;
    if (next.saturated) saturatedCount++;
    path += vec3Length(state.body.angularVelocity) * DT;
    rollHistory.push(quatToEulerZYX(state.body.orientation)[0]);
  }
  return {
    finalError: errorAngle(state.body.orientation, target),
    path,
    saturatedCount,
    rollHistory,
  };
}

describe("default controller params", () => {
  test("derives per-axis rate limits from real yaw authority", () => {
    expect(params.rate.maxAngularAcceleration[0]).toBe(20);
    expect(params.rate.maxAngularAcceleration[1]).toBe(20);
    const yawAcceleration = params.rate.maxAngularAcceleration[2];
    expect(yawAcceleration).toBeGreaterThan(1.5);
    expect(yawAcceleration).toBeLessThan(2.5);
    expect(params.position.maxThrust).toBeCloseTo(48, 12);
    expect(params.position.minThrust).toBeCloseTo(0.1 * weight, 12);
    expect(params.position.gains.ki).toBe(0.01);
    expect(params.rate.gains.ki).toBe(0.5);
  });

  test("scales thrust and yaw authority for a quad and an octo", () => {
    for (const [airframe, expectedThrust] of [
      [makeQuad(), 32],
      [makeOcto(), 64],
    ] as const) {
      const p = defaultControllerParams(airframe);
      expect(p.position.maxThrust).toBeCloseTo(expectedThrust, 12);
      expect(p.rate.maxAngularAcceleration[2]).toBeGreaterThan(0);
    }
  });
});

describe("attitude loop", () => {
  test("small roll, pitch and yaw steps settle within 0.5 degrees", () => {
    const cases = [
      { axis: 0 as const, target: quatFromAxisAngle([1, 0, 0], 10 * DEG), goal: 10 },
      { axis: 1 as const, target: quatFromAxisAngle([0, 1, 0], 10 * DEG), goal: 10 },
      { axis: 2 as const, target: quatFromAxisAngle([0, 0, 1], 20 * DEG), goal: 20 },
    ];
    for (const { axis, target, goal } of cases) {
      let state = hoverState();
      let rateState = initialControllerState().rate;
      let lastOutside = 0;
      let peak = 0;
      for (let i = 0; i < 8000; i++) {
        const desired = attitudeRateCommand(state.body.orientation, target, params.attitude);
        const next = advance(state, rateState, desired);
        state = next.state;
        rateState = next.rateState;
        const angle = quatToEulerZYX(state.body.orientation)[axis];
        peak = Math.max(peak, angle);
        if (Math.abs(angle - goal * DEG) > 0.5 * DEG) lastOutside = i;
      }
      const final = quatToEulerZYX(state.body.orientation)[axis];
      // eslint-disable-next-line no-console
      console.log(
        `[attitude ${axis}] final ${(final / DEG).toFixed(4)} deg, ` +
          `overshoot ${((peak - goal * DEG) / DEG).toFixed(4)} deg, ` +
          `0.5-deg settling ${(lastOutside * DT).toFixed(3)} s`,
      );
      expect(Math.abs(final - goal * DEG)).toBeLessThan(0.5 * DEG);
      expect(Math.max(0, peak - goal * DEG)).toBeLessThan(goal * 0.1 * DEG);
      expect(lastOutside * DT).toBeLessThan(4);
    }
  });

  test("takes the shortest path when the error quaternion has w < 0", () => {
    const target = quatFromAxisAngle([0, 0, 1], 200 * DEG);
    const error = attitudeError(QUAT_IDENTITY, target);
    expect(error[0]).toBeGreaterThan(0); // flipped from cos(100°) < 0
    expect(error[3]).toBeLessThan(0); // rotate the short way, about -z
    const result = runAttitudeStep(target, 18000);
    const shortest = 160 * DEG;
    // eslint-disable-next-line no-console
    console.log(`[attitude 200deg] final ${(result.finalError / DEG).toFixed(4)} deg, path ${result.path.toFixed(4)} rad`);
    expect(result.finalError).toBeLessThan(0.5 * DEG);
    expect(result.path).toBeLessThanOrEqual(1.2 * shortest);
  });

  test("handles a 120-degree yaw + tilt combination", () => {
    const target = quatMultiply(
      quatFromAxisAngle([0, 0, 1], 120 * DEG),
      quatFromAxisAngle([1, 0, 0], 120 * DEG),
    );
    const shortest = errorAngle(QUAT_IDENTITY, target);
    const result = runAttitudeStep(target, 18000);
    expect(result.finalError).toBeLessThan(0.5 * DEG);
    expect(result.path).toBeLessThanOrEqual(1.2 * shortest);
  });

  test("a 20-degree yaw step stays unsaturated at the mixer", () => {
    const target = quatFromAxisAngle([0, 0, 1], 20 * DEG);
    let state = hoverState();
    let rateState = initialControllerState().rate;
    let saturatedCount = 0;
    let lastYaw = 0;
    for (let i = 0; i < 6000; i++) {
      const desired = attitudeRateCommand(state.body.orientation, target, params.attitude);
      const next = advance(state, rateState, desired);
      if (next.saturated) saturatedCount++;
      state = next.state;
      rateState = next.rateState;
      lastYaw = quatToEulerZYX(state.body.orientation)[2];
    }
    expect(saturatedCount).toBe(0);
    expect(Math.abs(lastYaw - 20 * DEG)).toBeLessThan(0.5 * DEG);
  });

  test("desiredAttitude aligns body +z with thrust and +x with yaw", () => {
    const q = desiredAttitude([1, 0, 3], 0);
    const rotated = quatRotateVector(q, [0, 0, 1]);
    expect(rotated[0] / rotated[2]).toBeCloseTo(1 / 3, 12);
  });
});

describe("rate loop", () => {
  test("a 1 rad/s step converges with the measured bandwidth", () => {
    const result = runRateStep(6000);
    // eslint-disable-next-line no-console
    console.log(
      `[rate step] rise(10-90%) ${result.riseTime.toFixed(4)} s, ` +
        `overshoot ${(result.overshoot * 100).toFixed(2)} %, ` +
        `5% settling ${result.settlingTime.toFixed(4)} s`,
    );
    expect(result.riseTime).toBeGreaterThan(0);
    expect(result.riseTime).toBeLessThan(0.8);
    expect(result.overshoot).toBeLessThan(0.1);
    expect(result.settlingTime).toBeLessThan(3);
  });
});
