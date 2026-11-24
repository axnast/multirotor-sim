/**
 * Cascaded controller state, defaults and the inner rate loop.
 *
 * `defaultControllerParams` derives all gains/limits from a MultirotorConfig.
 * The yaw angular-acceleration limit is computed from the *actual* yaw
 * authority at hover, not a fixed number: at hover thrust `mg`, the mixer can
 * produce at most `maxYawTorque`; we keep 80 % of `maxYawTorque / Izz` so the
 * mixer does not shed yaw and wind up the yaw integrator. Roll/pitch keep a
 * fixed 20 rad/s² ceiling.
 *
 * `controllerStep` composes position → attitude → rate into the body wrench
 * consumed directly by `mix()`.
 */

import { vec3Cross, vec3Length, type Vec3 } from "../math/vec3";
import {
  maxThrust as configMaxThrust,
  validateConfig,
  type MotorConfig,
  type MultirotorConfig,
} from "../multirotor/config";
import { attitudeRateCommand } from "./attitude";
import { emptyPidState, pidStep } from "./pid";
import { positionStep } from "./position";
import type {
  AxisPidState,
  ControllerParams,
  ControllerState,
  ControllerStepResult,
  Measurement,
  PidConfig,
  PidResult,
  Setpoint,
} from "./types";

function assertFinite(name: string, value: number): void {
  if (!Number.isFinite(value)) {
    throw new RangeError(`${name} must be a finite number`);
  }
}

function assertVec3(name: string, v: Vec3): void {
  for (let i = 0; i < 3; i++) {
    assertFinite(`${name}[${i}]`, v[i]!);
  }
}

function assertPositive(name: string, value: number): void {
  assertFinite(name, value);
  if (value <= 0) {
    throw new RangeError(`${name} must be positive`);
  }
}

/**
 * Torque per unit thrust about body z for one rotor:
 * `-spin · kQ/kT`, in metres.
 */
function yawPerThrust(motor: MotorConfig): number {
  return -motor.spin * (motor.kQ / motor.kT);
}

/**
 * Maximum |yaw torque| available at a fixed total thrust. The thrust is
 * allocated greedily to the motors with the most favourable yaw coefficient,
 * respecting each motor's speed limit. This is exact for the presets (equal
 * `kQ/kT`) and a conservative bound for heterogeneous motors.
 */
function maxYawTorqueAtThrust(config: MultirotorConfig, thrust: number): number {
  const motors = config.motors.map((motor) => ({
    value: yawPerThrust(motor),
    max: motor.kT * motor.maxOmega * motor.maxOmega,
  }));
  const allocate = (ordered: typeof motors): number => {
    let remaining = thrust;
    let torque = 0;
    for (const motor of ordered) {
      const f = Math.max(0, Math.min(motor.max, remaining));
      torque += motor.value * f;
      remaining -= f;
      if (remaining <= 0) break;
    }
    return torque;
  };
  const positive = [...motors].sort((a, b) => b.value - a.value);
  const negative = [...motors].sort((a, b) => a.value - b.value);
  return Math.max(allocate(positive), -allocate(negative));
}

/** Build the default gains and limits for a given airframe. */
export function defaultControllerParams(config: MultirotorConfig): ControllerParams {
  validateConfig(config);
  const weight = config.mass * vec3Length(config.gravity);
  const inertiaDiagonal: Vec3 = [
    config.inertia[0],
    config.inertia[4],
    config.inertia[8],
  ];
  const yawAcceleration =
    (0.8 * maxYawTorqueAtThrust(config, weight)) / config.inertia[8];

  const params: ControllerParams = {
    mass: config.mass,
    inertiaDiagonal,
    gravity: config.gravity,
    position: {
      gains: { kp: 0.25, ki: 0.01, kd: 1.0 },
      integralSeparation: 0.1,
      maxAccelXY: 6,
      maxAccelZ: 3,
      maxTilt: Math.PI / 6,
      minThrust: 0.1 * weight,
      maxThrust: 0.8 * configMaxThrust(config),
    },
    attitude: {
      tiltGain: 1.2,
      yawGain: 0.8,
      maxTiltRate: 2.5,
      maxYawRate: 1.2,
      maxBodyRate: 3.0,
    },
    rate: {
      gains: { kp: 3.0, ki: 0.5, kd: 0.02 },
      derivativeCutoff: 50,
      maxAngularAcceleration: [20, 20, yawAcceleration],
    },
  };
  validateControllerParams(params);
  return params;
}

export function validateControllerParams(params: ControllerParams): void {
  assertPositive("mass", params.mass);
  assertVec3("gravity", params.gravity);
  assertVec3("inertiaDiagonal", params.inertiaDiagonal);
  assertPositive("position.kp", params.position.gains.kp);
  assertFinite("position.ki", params.position.gains.ki);
  assertFinite("position.kd", params.position.gains.kd);
  assertPositive("integralSeparation", params.position.integralSeparation);
  assertPositive("maxAccelXY", params.position.maxAccelXY);
  assertPositive("maxAccelZ", params.position.maxAccelZ);
  assertPositive("maxTilt", params.position.maxTilt);
  if (params.position.maxTilt >= Math.PI / 2) {
    throw new RangeError("maxTilt must be less than 90 degrees");
  }
  if (params.position.minThrust < 0 || params.position.maxThrust <= params.position.minThrust) {
    throw new RangeError("invalid thrust limits");
  }
  assertPositive("attitude.tiltGain", params.attitude.tiltGain);
  assertPositive("attitude.yawGain", params.attitude.yawGain);
  assertPositive("attitude.maxTiltRate", params.attitude.maxTiltRate);
  assertPositive("attitude.maxYawRate", params.attitude.maxYawRate);
  assertPositive("attitude.maxBodyRate", params.attitude.maxBodyRate);
  assertPositive("rate.kp", params.rate.gains.kp);
  assertFinite("rate.ki", params.rate.gains.ki);
  assertFinite("rate.kd", params.rate.gains.kd);
  assertPositive("rate.derivativeCutoff", params.rate.derivativeCutoff);
  assertVec3("maxAngularAcceleration", params.rate.maxAngularAcceleration);
  for (const value of params.rate.maxAngularAcceleration) {
    assertPositive("maxAngularAcceleration axis", value);
  }
}

/** A fresh, fully zeroed controller state. */
export function initialControllerState(): ControllerState {
  return {
    position: [emptyPidState(), emptyPidState(), emptyPidState()],
    rate: [emptyPidState(), emptyPidState(), emptyPidState()],
  };
}

export interface RateStepResult {
  readonly state: AxisPidState;
  /** Body-frame torque including the gyroscopic feedforward (N·m). */
  readonly torque: Vec3;
  readonly saturated: readonly [boolean, boolean, boolean];
}

/**
 * One inner rate-loop step. Each axis maps a rate error to a desired angular
 * acceleration via PID, then to torque via `I·α + ω × (Iω)`.
 */
export function rateStep(
  state: AxisPidState,
  desiredRate: Vec3,
  measurement: Measurement,
  params: ControllerParams,
  dt: number,
): RateStepResult {
  validateControllerParams(params);
  assertVec3("desiredRate", desiredRate);
  assertVec3("measurement.angularVelocity", measurement.angularVelocity);
  assertFinite("dt", dt);
  if (dt <= 0) throw new RangeError("dt must be positive");

  const omega = measurement.angularVelocity;
  const iOmega: Vec3 = [
    params.inertiaDiagonal[0] * omega[0],
    params.inertiaDiagonal[1] * omega[1],
    params.inertiaDiagonal[2] * omega[2],
  ];
  const gyroscopic = vec3Cross(omega, iOmega);

  const stepAxis = (axis: 0 | 1 | 2): PidResult => {
    const pidConfig: PidConfig = {
      gains: params.rate.gains,
      derivativeCutoff: params.rate.derivativeCutoff,
      minOutput: -params.rate.maxAngularAcceleration[axis],
      maxOutput: params.rate.maxAngularAcceleration[axis],
    };
    return pidStep(
      state[axis],
      desiredRate[axis] - omega[axis],
      omega[axis],
      dt,
      pidConfig,
    );
  };

  const x = stepAxis(0);
  const y = stepAxis(1);
  const z = stepAxis(2);
  const alpha: Vec3 = [x.output, y.output, z.output];
  return {
    state: [x.state, y.state, z.state],
    torque: [
      params.inertiaDiagonal[0] * alpha[0] + gyroscopic[0],
      params.inertiaDiagonal[1] * alpha[1] + gyroscopic[1],
      params.inertiaDiagonal[2] * alpha[2] + gyroscopic[2],
    ],
    saturated: [x.saturated, y.saturated, z.saturated],
  };
}

/**
 * One full cascade step: position -> attitude -> rate. The returned wrench is
 * exactly the `Wrench` accepted by `mix()`. All state (both PID memories) is
 * returned explicitly; nothing is hidden in the params or the functions.
 */
export function controllerStep(
  controllerState: ControllerState,
  measurement: Measurement,
  setpoint: Setpoint,
  params: ControllerParams,
  dt: number,
): ControllerStepResult {
  validateControllerParams(params);
  const outer = positionStep(
    controllerState.position,
    measurement,
    setpoint,
    params,
    dt,
  );
  const desiredRate = attitudeRateCommand(
    measurement.orientation,
    outer.desiredOrientation,
    params.attitude,
  );
  const inner = rateStep(
    controllerState.rate,
    desiredRate,
    measurement,
    params,
    dt,
  );
  return {
    state: { position: outer.state, rate: inner.state },
    wrench: [
      outer.thrust,
      inner.torque[0],
      inner.torque[1],
      inner.torque[2],
    ],
  };
}
