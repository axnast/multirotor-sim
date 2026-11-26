import { describe, expect, test } from "vitest";

import {
  controllerStep,
  defaultControllerParams,
  initialControllerState,
} from "../src/core/control/controller";
import type { ControllerState, Setpoint } from "../src/core/control/types";
import { QUAT_IDENTITY, quatToEulerZYX } from "../src/core/math/quat";
import { vec3Length, type Vec3 } from "../src/core/math/vec3";
import {
  hoverMotorSpeeds,
  makeHex,
  makeOcto,
  makeQuad,
  type MultirotorConfig,
} from "../src/core/multirotor/config";
import {
  stepMultirotorDetailed,
  type MultirotorState,
} from "../src/core/multirotor/dynamics";
import { mixDetailed } from "../src/core/multirotor/mixer";

const DT = 1e-3;

interface FlightStats {
  readonly final: MultirotorState;
  readonly maxPositionError: number;
  readonly maxTilt: number;
  readonly maxMotorSpeed: number;
  readonly mixerSaturations: number;
  readonly nonFinite: number;
  readonly lastContactTime: number | null;
  readonly positionIntegral: number;
  readonly xHistory: number[];
  readonly yHistory: number[];
  readonly zHistory: number[];
}

function measurementOf(state: MultirotorState) {
  return {
    position: state.body.position,
    velocity: state.body.velocity,
    orientation: state.body.orientation,
    angularVelocity: state.body.angularVelocity,
  };
}

function runFlight(
  config: MultirotorConfig,
  initial: MultirotorState,
  setpointAt: (time: number) => Setpoint,
  steps: number,
): FlightStats {
  const params = defaultControllerParams(config);
  let state = initial;
  let controllerState: ControllerState = initialControllerState();
  let maxPositionError = 0;
  let maxTilt = 0;
  let maxMotorSpeed = 0;
  let mixerSaturations = 0;
  let nonFinite = 0;
  let lastContactTime: number | null = null;
  const xHistory: number[] = [];
  const yHistory: number[] = [];
  const zHistory: number[] = [];

  for (let step = 0; step < steps; step++) {
    const time = step * DT;
    const setpoint = setpointAt(time);
    const command = controllerStep(
      controllerState,
      measurementOf(state),
      setpoint,
      params,
      DT,
    );
    controllerState = command.state;
    const mixed = mixDetailed(config, command.wrench);
    if (mixed.saturated) mixerSaturations++;
    const result = stepMultirotorDetailed(state, config, mixed.motorSpeeds, DT);
    state = result.state;
    if (result.ground?.contacted) lastContactTime = (step + 1) * DT;

    const error = vec3Length([
      state.body.position[0] - setpoint.position[0],
      state.body.position[1] - setpoint.position[1],
      state.body.position[2] - setpoint.position[2],
    ]);
    maxPositionError = Math.max(maxPositionError, error);
    maxTilt = Math.max(
      maxTilt,
      Math.abs(quatToEulerZYX(state.body.orientation)[0]),
      Math.abs(quatToEulerZYX(state.body.orientation)[1]),
    );
    for (const speed of state.motorSpeeds) {
      maxMotorSpeed = Math.max(maxMotorSpeed, speed);
    }
    for (const value of [...state.body.position, ...state.body.velocity]) {
      if (!Number.isFinite(value)) nonFinite++;
    }
    xHistory.push(state.body.position[0]);
    yHistory.push(state.body.position[1]);
    zHistory.push(state.body.position[2]);
  }

  const integrals = controllerState.position.map((axis) => axis.integral);
  return {
    final: state,
    maxPositionError,
    maxTilt,
    maxMotorSpeed,
    mixerSaturations,
    nonFinite,
    lastContactTime,
    positionIntegral: Math.hypot(...integrals),
    xHistory,
    yHistory,
    zHistory,
  };
}

function hoverInitial(config: MultirotorConfig, z = 5): MultirotorState {
  return {
    body: {
      position: [0, 0, z],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    },
    motorSpeeds: hoverMotorSpeeds(config),
  };
}

function fixedSetpoint(position: Vec3, yaw = 0): Setpoint {
  return { position, yaw };
}

function settlingIndex(values: number[], target: number, band: number): number {
  let last = 0;
  for (let i = 0; i < values.length; i++) {
    if (Math.abs(values[i]! - target) > band) last = i;
  }
  return last;
}

const config = makeHex();

describe("cascade", () => {
  test("holds hover within 1 mm for 10 s", () => {
    const stats = runFlight(config, hoverInitial(config), () => fixedSetpoint([0, 0, 5]), 10000);
    // eslint-disable-next-line no-console
    console.log(`[hover hold] max error ${stats.maxPositionError.toExponential(3)} m, mixer saturations ${stats.mixerSaturations}`);
    expect(stats.maxPositionError).toBeLessThan(1e-3);
    expect(stats.maxTilt).toBeLessThan(0.5 * Math.PI / 180);
    expect(stats.mixerSaturations).toBe(0);
    expect(stats.maxMotorSpeed).toBeLessThanOrEqual(1000);
  });

  test("steps 5 m in x with bounded overshoot and settling", () => {
    const stats = runFlight(config, hoverInitial(config), () => fixedSetpoint([5, 0, 5]), 25000);
    const target = 5;
    const settle = settlingIndex(stats.xHistory, target, 0.1);
    const overshoot = Math.max(0, Math.max(...stats.xHistory) - target);
    const steadyState = Math.abs(stats.xHistory[stats.xHistory.length - 1]! - target);
    // eslint-disable-next-line no-console
    console.log(
      `[position step] overshoot ${overshoot.toFixed(4)} m (${(overshoot / target * 100).toFixed(2)} %), ` +
        `2% settling ${(settle * DT).toFixed(3)} s, steady-state error ${steadyState.toExponential(3)} m, ` +
        `max tilt ${(stats.maxTilt * 180 / Math.PI).toFixed(3)} deg`,
    );
    expect(overshoot / target).toBeLessThan(0.1);
    expect(settle * DT).toBeLessThan(20);
    expect(steadyState).toBeLessThan(0.01);
    expect(stats.maxTilt).toBeLessThanOrEqual((30.1 * Math.PI) / 180);
    expect(stats.maxMotorSpeed).toBeLessThanOrEqual(1000);
    expect(stats.mixerSaturations).toBe(0);
  }, 6_000);

  test("flies a 5 m square without exceeding limits", () => {
    const corners: readonly Vec3[] = [
      [5, 0, 5],
      [5, 5, 5],
      [0, 5, 5],
      [0, 0, 5],
    ];
    const initial = hoverInitial(config);
    const started: MultirotorState = {
      ...initial,
      body: { ...initial.body, position: corners[0]! },
    };
    const stats = runFlight(
      config,
      started,
      (time) => fixedSetpoint(corners[Math.min(3, Math.floor(time / 20))]!),
      80000,
    );
    let finalWindowMaxError = 0;
    for (let leg = 0; leg < 4; leg++) {
      const corner = corners[leg]!;
      for (let step = (leg * 20 + 17) * 1000; step < (leg + 1) * 20000; step++) {
        finalWindowMaxError = Math.max(
          finalWindowMaxError,
          Math.hypot(
            stats.xHistory[step]! - corner[0],
            stats.yHistory[step]! - corner[1],
            stats.zHistory[step]! - corner[2],
          ),
        );
      }
    }
    // eslint-disable-next-line no-console
    console.log(
      `[square] final-3s max error ${finalWindowMaxError.toFixed(4)} m, max tilt ${(stats.maxTilt * 180 / Math.PI).toFixed(3)} deg, saturations ${stats.mixerSaturations}`,
    );
    expect(finalWindowMaxError).toBeLessThan(0.1);
    expect(stats.maxTilt).toBeLessThanOrEqual((30.1 * Math.PI) / 180);
    expect(stats.maxMotorSpeed).toBeLessThanOrEqual(1000);
    expect(stats.mixerSaturations).toBe(0);
  }, 20_000);

  test("takes off from the ground to 5 m without oscillation", () => {
    const groundConfig = makeHex({ groundPlane: true });
    const initial: MultirotorState = {
      body: {
        position: [0, 0, 0],
        velocity: [0, 0, 0],
        orientation: QUAT_IDENTITY,
        angularVelocity: [0, 0, 0],
      },
      motorSpeeds: groundConfig.motors.map(() => 0),
    };
    const stats = runFlight(
      groundConfig,
      initial,
      () => fixedSetpoint([0, 0, 5]),
      25000,
    );
    const z = stats.zHistory[stats.zHistory.length - 1]!;
    // eslint-disable-next-line no-console
    console.log(
      `[takeoff] final z ${z.toFixed(4)} m, last contact ${stats.lastContactTime} s, saturations ${stats.mixerSaturations}`,
    );
    expect(Math.abs(z - 5)).toBeLessThan(0.05);
    expect(stats.lastContactTime).not.toBeNull();
    expect(stats.lastContactTime!).toBeLessThan(1);
    expect(stats.maxTilt).toBeLessThan(2 * Math.PI / 180);
    expect(stats.mixerSaturations).toBe(0);
  }, 8_000);

  test("survives an unreachable setpoint and recovers", () => {
    const stats = runFlight(
      config,
      hoverInitial(config),
      (time) => fixedSetpoint(time < 5 ? [500, 0, 5] : [0, 0, 5]),
      65000,
    );
    const final = stats.final.body.position;
    // eslint-disable-next-line no-console
    console.log(
      `[saturation] |integral| ${stats.positionIntegral.toFixed(3)} m·s, max tilt ${(stats.maxTilt * 180 / Math.PI).toFixed(3)} deg, ` +
        `final [${final.map((v) => v.toFixed(3)).join(", ")}], mixer saturations ${stats.mixerSaturations}`,
    );
    expect(stats.nonFinite).toBe(0);
    expect(stats.positionIntegral).toBeLessThan(100);
    expect(stats.maxTilt).toBeLessThanOrEqual((32 * Math.PI) / 180);
    expect(vec3Length([final[0], final[1], final[2] - 5])).toBeLessThan(0.1);
  }, 18_000);

  test("the same gains stabilise quad, hex and octo with scaled inertia", () => {
    for (const airframe of [
      makeQuad({ armLength: 0.2 }),
      makeHex({ armLength: 0.25 }),
      makeOcto({ armLength: 0.3 }),
    ]) {
      const stats = runFlight(
        airframe,
        hoverInitial(airframe),
        () => fixedSetpoint([2, 0, 5]),
        25000,
      );
      const settle = settlingIndex(stats.xHistory, 2, 0.04);
      // eslint-disable-next-line no-console
      console.log(
        `[airframe ${airframe.motors.length}] settling ${(settle * DT).toFixed(3)} s, final x ${stats.xHistory[stats.xHistory.length - 1]!.toFixed(4)} m`,
      );
      expect(settle * DT).toBeLessThan(20);
      expect(Math.abs(stats.xHistory[stats.xHistory.length - 1]! - 2)).toBeLessThan(0.05);
      expect(stats.maxMotorSpeed).toBeLessThanOrEqual(1000);
      expect(stats.mixerSaturations).toBe(0);
    }
  }, 18_000);

  test("is deterministic across two identical runs", () => {
    const run = (): MultirotorState =>
      runFlight(config, hoverInitial(config), () => fixedSetpoint([3, 0, 5]), 2000).final;
    expect(run()).toEqual(run());
  });
});
