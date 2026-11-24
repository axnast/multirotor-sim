/**
 * Shared types for the Milestone 3 cascaded controller.
 *
 * Everything here is plain data: the controller has no hidden state, and the
 * integrator/filter memory lives entirely in `ControllerState` / `PidState`.
 * These types are independent of the simulator config; `defaultControllerParams`
 * in controller.ts builds a `ControllerParams` from a `MultirotorConfig`.
 */

import type { Quat } from "../math/quat";
import type { Vec3 } from "../math/vec3";
import type { Wrench } from "../multirotor/mixer";

/**
 * The measured flight state. M3 uses the true simulator state; M5 may replace
 * it with noisy estimates without changing the controller.
 */
export interface Measurement {
  readonly position: Vec3;
  readonly velocity: Vec3;
  readonly orientation: Quat;
  readonly angularVelocity: Vec3;
}

/** Position + world yaw, with optional velocity feedforward. */
export interface Setpoint {
  readonly position: Vec3;
  readonly yaw: number;
  readonly velocity?: Vec3;
}

export interface PidGains {
  readonly kp: number;
  readonly ki: number;
  readonly kd: number;
}

export interface PidConfig {
  readonly gains: PidGains;
  /** First-order low-pass cutoff on the derivative of the measurement (rad/s). */
  readonly derivativeCutoff: number;
  readonly minOutput: number;
  readonly maxOutput: number;
}

export interface PidState {
  readonly integral: number;
  readonly derivative: number;
  /** null only for the very first step, where the derivative is defined as 0. */
  readonly lastMeasurement: number | null;
}

export interface PidResult {
  readonly state: PidState;
  readonly output: number;
  readonly saturated: boolean;
}

export interface PositionParams {
  readonly gains: PidGains;
  readonly maxAccelXY: number;
  readonly maxAccelZ: number;
  readonly maxTilt: number;
  readonly minThrust: number;
  readonly maxThrust: number;
}

export interface AttitudeParams {
  readonly tiltGain: number;
  readonly yawGain: number;
  readonly maxTiltRate: number;
  readonly maxYawRate: number;
  readonly maxBodyRate: number;
}

export interface RateParams {
  readonly gains: PidGains;
  readonly derivativeCutoff: number;
  /** Per-axis angular-acceleration limits (rad/s²), body x/y/z. */
  readonly maxAngularAcceleration: Vec3;
}

export interface ControllerParams {
  readonly mass: number;
  readonly inertiaDiagonal: Vec3;
  readonly gravity: Vec3;
  readonly position: PositionParams;
  readonly attitude: AttitudeParams;
  readonly rate: RateParams;
}

export type AxisPidState = readonly [PidState, PidState, PidState];

export interface ControllerState {
  readonly position: AxisPidState;
  readonly rate: AxisPidState;
}

export interface ControllerStepResult {
  readonly state: ControllerState;
  readonly wrench: Wrench;
}
