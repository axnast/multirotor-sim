/**
 * Milestone 3 demo: a hex takes off from the ground, climbs to 5 m, flies a
 * square waypoint pattern, then holds at the start corner. Runs at 1 kHz with
 * the pure cascade controller and writes setpoint and actual values together.
 *
 * Run with: npm run demo:flight
 * Output:   out/flight.csv
 */

import { createHash } from "node:crypto";
import { mkdirSync, writeFileSync } from "node:fs";

import {
  controllerStep,
  defaultControllerParams,
  initialControllerState,
} from "../src/core/control/controller";
import type { ControllerParams, ControllerState, Setpoint } from "../src/core/control/types";
import {
  QUAT_IDENTITY,
  quatNormalize,
  quatToEulerZYX,
} from "../src/core/math/quat";
import { vec3Dot, vec3Length, vec3Scale, vec3Sub, type Vec3 } from "../src/core/math/vec3";
import { makeHex, type MultirotorConfig } from "../src/core/multirotor/config";
import {
  stepMultirotorDetailed,
  type MultirotorState,
} from "../src/core/multirotor/dynamics";
import { mixDetailed } from "../src/core/multirotor/mixer";

const DT = 1e-3;
const LEG_STEPS = 20_000; // a 5 m step settles in about 15.2 s
const HOLD_STEPS = 5_000;
const ALTITUDE = 5;
const LEG = 5;
const OUTPUT_PATH = "out/flight.csv";

const CSV_HEADER = [
  "t_s", "phase",
  "x_sp_m", "y_sp_m", "z_sp_m", "yaw_sp_rad",
  "x_m", "y_m", "z_m",
  "vx_ms", "vy_ms", "vz_ms",
  "qw", "qx", "qy", "qz",
  "roll_rad", "pitch_rad", "yaw_rad",
  "wx_rads", "wy_rads", "wz_rads",
  "thrust_N", "tau_x_Nm", "tau_y_Nm", "tau_z_Nm",
  "saturated", "contact", "normal_impulse_Ns",
  "w0_rads", "w1_rads", "w2_rads", "w3_rads", "w4_rads", "w5_rads",
] as const;

interface Phase {
  readonly name: "takeoff" | "leg-1" | "leg-2" | "leg-3" | "leg-4" | "hold";
  readonly steps: number;
  readonly setpoint: Setpoint;
}

interface Summary {
  readonly phase: string;
  readonly overshoot: number;
  readonly settlingTime: number;
  readonly settledError: number;
}

function csvRow(values: readonly (number | boolean | string)[]): string {
  return values
    .map((value) => {
      if (typeof value === "string") return value;
      if (typeof value === "boolean") return value ? "1" : "0";
      return value.toFixed(9);
    })
    .join(",");
}

function fixed(position: Vec3): Setpoint {
  return { position, yaw: 0 };
}

function phases(): readonly Phase[] {
  return [
    { name: "takeoff", steps: LEG_STEPS, setpoint: fixed([0, 0, ALTITUDE]) },
    { name: "leg-1", steps: LEG_STEPS, setpoint: fixed([LEG, 0, ALTITUDE]) },
    { name: "leg-2", steps: LEG_STEPS, setpoint: fixed([LEG, LEG, ALTITUDE]) },
    { name: "leg-3", steps: LEG_STEPS, setpoint: fixed([0, LEG, ALTITUDE]) },
    { name: "leg-4", steps: LEG_STEPS, setpoint: fixed([0, 0, ALTITUDE]) },
    { name: "hold", steps: HOLD_STEPS, setpoint: fixed([0, 0, ALTITUDE]) },
  ];
}

function runPhase(
  phase: Phase,
  state: MultirotorState,
  controllerState: ControllerState,
  params: ControllerParams,
  config: MultirotorConfig,
  globalStep: number,
  rows: string[],
): { state: MultirotorState; controllerState: ControllerState; summary: Summary } {
  let current = state;
  let control = controllerState;
  const startPosition = current.body.position;
  const delta = vec3Sub(phase.setpoint.position, startPosition);
  const deltaLength = vec3Length(delta);
  const direction: Vec3 = deltaLength > 0 ? vec3Scale(delta, 1 / deltaLength) : [0, 0, 0];

  let reached = false;
  let overshoot = 0;
  let lastOutside = 0;
  let settledError = 0;

  for (let step = 0; step < phase.steps; step++) {
    const command = controllerStep(
      control,
      {
        position: current.body.position,
        velocity: current.body.velocity,
        orientation: current.body.orientation,
        angularVelocity: current.body.angularVelocity,
      },
      phase.setpoint,
      params,
      DT,
    );
    control = command.state;
    const mixed = mixDetailed(config, command.wrench);
    const result = stepMultirotorDetailed(current, config, mixed.motorSpeeds, DT);
    current = result.state;

    const position: Vec3 = current.body.position;
    const deviation = vec3Length(vec3Sub(position, phase.setpoint.position));
    if (deviation < 0.1) reached = true;
    if (reached && deltaLength > 0) {
      overshoot = Math.max(overshoot, vec3Dot(vec3Sub(position, phase.setpoint.position), direction));
    }
    if (deviation > 0.1) lastOutside = step;
    if (step >= phase.steps - 3000) settledError = Math.max(settledError, deviation);

    rows.push(csvRow([
      (globalStep + step) * DT,
      phase.name,
      ...phase.setpoint.position,
      phase.setpoint.yaw,
      ...position,
      ...current.body.velocity,
      ...quatNormalize(current.body.orientation),
      ...quatToEulerZYX(current.body.orientation),
      ...current.body.angularVelocity,
      ...command.wrench,
      mixed.saturated,
      result.ground?.contacted ?? false,
      result.ground?.normalImpulse ?? 0,
      ...current.motorSpeeds,
    ]));
  }

  return {
    state: current,
    controllerState: control,
    summary: {
      phase: phase.name,
      overshoot,
      settlingTime: (lastOutside + 1) * DT,
      settledError,
    },
  };
}

function main(): void {
  const config = makeHex({ groundPlane: true });
  const params = defaultControllerParams(config);
  let state: MultirotorState = {
    body: {
      position: [0, 0, 0],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    },
    motorSpeeds: config.motors.map(() => 0),
  };
  let controllerState = initialControllerState();
  const rows: string[] = [];
  const summaries: Summary[] = [];
  let globalStep = 0;
  for (const phase of phases()) {
    const result = runPhase(
      phase, state, controllerState, params, config, globalStep, rows,
    );
    state = result.state;
    controllerState = result.controllerState;
    summaries.push(result.summary);
    globalStep += phase.steps;
  }

  const content = `${[CSV_HEADER.join(","), ...rows].join("\n")}\n`;
  const hash = createHash("sha256").update(content, "utf8").digest("hex");
  mkdirSync("out", { recursive: true });
  writeFileSync(OUTPUT_PATH, content, "utf8");

  console.log("multirotor flight demo — controller cascade at 1000 Hz\n");
  for (const summary of summaries) {
    console.log(
      `phase ${summary.phase.padEnd(8)} | overshoot ${summary.overshoot.toFixed(4)} m | ` +
        `2% settling ${summary.settlingTime.toFixed(3)} s | tracking error ${summary.settledError.toFixed(4)} m`,
    );
  }
  console.log(
    `wrote ${rows.length} rows × ${CSV_HEADER.length} columns to ${OUTPUT_PATH} ` +
      `(${(Buffer.byteLength(content, "utf8") / 1e6).toFixed(2)} MB)`,
  );
  console.log(`sha256 : ${hash}`);
}

main();
