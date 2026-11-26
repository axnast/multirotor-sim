/**
 * Pure telemetry extraction. Samples are taken inside the runner every
 * `SAMPLE_INTERVAL_STEPS` core steps, never at render-frame boundaries.
 */

import type { ControllerStepDetailedResult } from "../../core/control/controller";
import type { Measurement, Setpoint } from "../../core/control/types";
import { quatToEulerZYX } from "../../core/math/quat";
import type { Vec3 } from "../../core/math/vec3";
import type { MultirotorState } from "../../core/multirotor/dynamics";

export interface TelemetrySample {
  readonly t: number;
  readonly position: Vec3;
  readonly positionSp: Vec3;
  /** Actual Euler angles in radians (display only). */
  readonly euler: Vec3;
  /** Desired Euler angles in radians, from the cascade diagnostics. */
  readonly eulerSp: Vec3;
  readonly motorSpeeds: readonly number[];
  /** What the controller actually saw, or null when sensors are disabled. */
  readonly measuredPosition: Vec3 | null;
}

export interface TelemetryInput {
  readonly t: number;
  readonly multirotor: MultirotorState;
  readonly setpoint: Setpoint;
  readonly last: ControllerStepDetailedResult | null;
  readonly measurement?: Measurement | null;
}

/** Build a single sample without touching the simulation state. */
export function makeTelemetrySample(input: TelemetryInput): TelemetrySample {
  const { t, multirotor, setpoint, last, measurement } = input;
  const euler = quatToEulerZYX(multirotor.body.orientation);
  const eulerSp =
    last === null
      ? ([0, 0, setpoint.yaw] as Vec3)
      : quatToEulerZYX(last.desiredOrientation);
  return {
    t,
    position: [
      multirotor.body.position[0],
      multirotor.body.position[1],
      multirotor.body.position[2],
    ],
    positionSp: [
      setpoint.position[0],
      setpoint.position[1],
      setpoint.position[2],
    ],
    euler,
    eulerSp,
    motorSpeeds: [...multirotor.motorSpeeds],
    measuredPosition:
      measurement === undefined || measurement === null
        ? null
        : [...measurement.position],
  };
}
