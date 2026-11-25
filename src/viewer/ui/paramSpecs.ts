/**
 * Pure slider specifications for live gain tuning. Every setter returns a new,
 * validated ControllerParams and never mutates its input.
 */

import {
  validateControllerParams,
} from "../../core/control/controller";
import type { ControllerParams } from "../../core/control/types";
import type { Vec3 } from "../../core/math/vec3";

export type ParamGroup = "position" | "attitude" | "rate";

export type ParamKey =
  | "position.kp"
  | "position.ki"
  | "position.kd"
  | "position.integralSeparation"
  | "position.maxAccelXY"
  | "position.maxAccelZ"
  | "position.maxTilt"
  | "attitude.tiltGain"
  | "attitude.yawGain"
  | "attitude.maxTiltRate"
  | "attitude.maxYawRate"
  | "attitude.maxBodyRate"
  | "rate.kp"
  | "rate.ki"
  | "rate.kd"
  | "rate.derivativeCutoff"
  | "rate.maxAngularAccelerationXY"
  | "rate.maxAngularAccelerationZ";

export interface ParamSlider {
  readonly id: ParamKey;
  readonly group: ParamGroup;
  readonly label: string;
  readonly unit: string;
  readonly min: number;
  readonly max: number;
  readonly step: number;
  readonly get: (params: ControllerParams) => number;
  readonly set: (params: ControllerParams, value: number) => ControllerParams;
}

export const PARAM_GROUPS: readonly { readonly id: ParamGroup; readonly label: string }[] = [
  { id: "position", label: "Position loop" },
  { id: "attitude", label: "Attitude loop" },
  { id: "rate", label: "Rate loop" },
];

export const PARAM_SLIDERS: readonly ParamSlider[] = [
  {
    id: "position.kp", group: "position", label: "kp", unit: "s⁻²",
    min: 0.05, max: 1, step: 0.01,
    get: (p) => p.position.gains.kp,
    set: (p, v) => ({ ...p, position: { ...p.position, gains: { ...p.position.gains, kp: v } } }),
  },
  {
    id: "position.ki", group: "position", label: "ki", unit: "s⁻³",
    min: 0, max: 0.1, step: 0.001,
    get: (p) => p.position.gains.ki,
    set: (p, v) => ({ ...p, position: { ...p.position, gains: { ...p.position.gains, ki: v } } }),
  },
  {
    id: "position.kd", group: "position", label: "kd", unit: "s⁻¹",
    min: 0.2, max: 3, step: 0.05,
    get: (p) => p.position.gains.kd,
    set: (p, v) => ({ ...p, position: { ...p.position, gains: { ...p.position.gains, kd: v } } }),
  },
  {
    id: "position.integralSeparation", group: "position", label: "I separation", unit: "m",
    min: 0.02, max: 1, step: 0.01,
    get: (p) => p.position.integralSeparation,
    set: (p, v) => ({ ...p, position: { ...p.position, integralSeparation: v } }),
  },
  {
    id: "position.maxAccelXY", group: "position", label: "max accel XY", unit: "m/s²",
    min: 1, max: 10, step: 0.5,
    get: (p) => p.position.maxAccelXY,
    set: (p, v) => ({ ...p, position: { ...p.position, maxAccelXY: v } }),
  },
  {
    id: "position.maxAccelZ", group: "position", label: "max accel Z", unit: "m/s²",
    min: 1, max: 6, step: 0.5,
    get: (p) => p.position.maxAccelZ,
    set: (p, v) => ({ ...p, position: { ...p.position, maxAccelZ: v } }),
  },
  {
    id: "position.maxTilt", group: "position", label: "max tilt", unit: "°",
    min: 5, max: 45, step: 1,
    get: (p) => (p.position.maxTilt * 180) / Math.PI,
    set: (p, v) => ({ ...p, position: { ...p.position, maxTilt: (v * Math.PI) / 180 } }),
  },
  {
    id: "attitude.tiltGain", group: "attitude", label: "tilt gain", unit: "s⁻¹",
    min: 0.2, max: 4, step: 0.05,
    get: (p) => p.attitude.tiltGain,
    set: (p, v) => ({ ...p, attitude: { ...p.attitude, tiltGain: v } }),
  },
  {
    id: "attitude.yawGain", group: "attitude", label: "yaw gain", unit: "s⁻¹",
    min: 0.2, max: 3, step: 0.05,
    get: (p) => p.attitude.yawGain,
    set: (p, v) => ({ ...p, attitude: { ...p.attitude, yawGain: v } }),
  },
  {
    id: "attitude.maxTiltRate", group: "attitude", label: "max tilt rate", unit: "rad/s",
    min: 0.5, max: 5, step: 0.1,
    get: (p) => p.attitude.maxTiltRate,
    set: (p, v) => ({ ...p, attitude: { ...p.attitude, maxTiltRate: v } }),
  },
  {
    id: "attitude.maxYawRate", group: "attitude", label: "max yaw rate", unit: "rad/s",
    min: 0.2, max: 3, step: 0.1,
    get: (p) => p.attitude.maxYawRate,
    set: (p, v) => ({ ...p, attitude: { ...p.attitude, maxYawRate: v } }),
  },
  {
    id: "attitude.maxBodyRate", group: "attitude", label: "max body rate", unit: "rad/s",
    min: 1, max: 6, step: 0.1,
    get: (p) => p.attitude.maxBodyRate,
    set: (p, v) => ({ ...p, attitude: { ...p.attitude, maxBodyRate: v } }),
  },
  {
    id: "rate.kp", group: "rate", label: "kp", unit: "s⁻¹",
    min: 0.5, max: 10, step: 0.1,
    get: (p) => p.rate.gains.kp,
    set: (p, v) => ({ ...p, rate: { ...p.rate, gains: { ...p.rate.gains, kp: v } } }),
  },
  {
    id: "rate.ki", group: "rate", label: "ki", unit: "s⁻²",
    min: 0, max: 3, step: 0.05,
    get: (p) => p.rate.gains.ki,
    set: (p, v) => ({ ...p, rate: { ...p.rate, gains: { ...p.rate.gains, ki: v } } }),
  },
  {
    id: "rate.kd", group: "rate", label: "kd", unit: "s",
    min: 0, max: 0.2, step: 0.005,
    get: (p) => p.rate.gains.kd,
    set: (p, v) => ({ ...p, rate: { ...p.rate, gains: { ...p.rate.gains, kd: v } } }),
  },
  {
    id: "rate.derivativeCutoff", group: "rate", label: "D cutoff", unit: "rad/s",
    min: 10, max: 100, step: 5,
    get: (p) => p.rate.derivativeCutoff,
    set: (p, v) => ({ ...p, rate: { ...p.rate, derivativeCutoff: v } }),
  },
  {
    id: "rate.maxAngularAccelerationXY", group: "rate", label: "max α roll/pitch", unit: "rad/s²",
    min: 5, max: 40, step: 1,
    get: (p) => p.rate.maxAngularAcceleration[0],
    set: (p, v) => {
      const limits: Vec3 = [v, v, p.rate.maxAngularAcceleration[2]];
      return { ...p, rate: { ...p.rate, maxAngularAcceleration: limits } };
    },
  },
  {
    id: "rate.maxAngularAccelerationZ", group: "rate", label: "max α yaw", unit: "rad/s²",
    min: 0.5, max: 5, step: 0.01,
    get: (p) => p.rate.maxAngularAcceleration[2],
    set: (p, v) => {
      const limits: Vec3 = [
        p.rate.maxAngularAcceleration[0],
        p.rate.maxAngularAcceleration[1],
        v,
      ];
      return { ...p, rate: { ...p.rate, maxAngularAcceleration: limits } };
    },
  },
];

/** True for the two integrator sliders; callers must rescale stored integrals. */
export function isKiSlider(id: ParamKey): boolean {
  return id === "position.ki" || id === "rate.ki";
}

export function clampSliderValue(spec: ParamSlider, value: number): number {
  return Math.min(spec.max, Math.max(spec.min, value));
}

/** Apply one slider value, returning a fresh validated params object. */
export function applyParamValue(
  params: ControllerParams,
  id: ParamKey,
  value: number,
): ControllerParams {
  const spec = PARAM_SLIDERS.find((candidate) => candidate.id === id);
  if (spec === undefined) {
    throw new Error(`unknown slider ${id}`);
  }
  if (!Number.isFinite(value)) {
    throw new RangeError(`${id} must be finite`);
  }
  const next = spec.set(params, clampSliderValue(spec, value));
  validateControllerParams(next);
  return next;
}
