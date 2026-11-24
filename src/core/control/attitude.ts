/**
 * Attitude loop: quaternion-error P controller producing a desired body rate.
 *
 * - The error is `q_err = q* ⊗ q_target`, expressed in the body frame.
 * - The sign of `q_err` is flipped when its scalar part is negative, so the
 *   controller always takes the shortest rotation (M3 test: large yaw step).
 * - Tilt (body x/y) and yaw (body z) use separate gains and rate limits. The
 *   total body-rate magnitude is capped with tilt priority: yaw is reduced
 *   first, then tilt only if it alone exceeds the cap.
 * - No Euler angles appear in the control path.
 *
 * `desiredAttitude` builds a target quaternion from a desired thrust direction
 * (body +z) and a world yaw (body +x heading). All functions are pure.
 */

import { mat3, type Mat3 } from "../math/mat3";
import {
  quatConjugate,
  quatFromRotationMatrix,
  quatMultiply,
  quatNormalize,
  type Quat,
} from "../math/quat";
import { vec3Cross, vec3Dot, vec3Normalize, vec3Scale, vec3Sub, type Vec3 } from "../math/vec3";
import type { AttitudeParams } from "./types";

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

function assertValidParams(params: AttitudeParams): void {
  assertFinite("tiltGain", params.tiltGain);
  assertFinite("yawGain", params.yawGain);
  assertFinite("maxTiltRate", params.maxTiltRate);
  assertFinite("maxYawRate", params.maxYawRate);
  assertFinite("maxBodyRate", params.maxBodyRate);
  if (
    params.tiltGain <= 0 ||
    params.yawGain <= 0 ||
    params.maxTiltRate <= 0 ||
    params.maxYawRate <= 0 ||
    params.maxBodyRate <= 0
  ) {
    throw new RangeError("attitude gains and limits must be positive");
  }
}

/** Shortest-path quaternion error from current to target, in the body frame. */
export function attitudeError(orientation: Quat, target: Quat): Quat {
  const q = quatNormalize(orientation);
  const desired = quatNormalize(target);
  const error = quatMultiply(quatConjugate(q), desired);
  return error[0] < 0
    ? [-error[0], -error[1], -error[2], -error[3]]
    : error;
}

/**
 * Desired body rate (rad/s) that drives `orientation` to `target`.
 * Tilt is limited first; yaw is then reduced to respect `maxBodyRate`.
 */
export function attitudeRateCommand(
  orientation: Quat,
  target: Quat,
  params: AttitudeParams,
): Vec3 {
  assertValidParams(params);
  const error = attitudeError(orientation, target);

  let tiltX = 2 * params.tiltGain * error[1];
  let tiltY = 2 * params.tiltGain * error[2];
  let yaw = 2 * params.yawGain * error[3];
  assertFinite("tiltX", tiltX);
  assertFinite("tiltY", tiltY);
  assertFinite("yaw", yaw);

  let tiltMag = Math.hypot(tiltX, tiltY);
  if (tiltMag > params.maxTiltRate) {
    const scale = params.maxTiltRate / tiltMag;
    tiltX *= scale;
    tiltY *= scale;
    tiltMag = params.maxTiltRate;
  }
  yaw = Math.min(params.maxYawRate, Math.max(-params.maxYawRate, yaw));

  if (tiltMag + Math.abs(yaw) > params.maxBodyRate) {
    const yawBudget = Math.sqrt(
      Math.max(0, params.maxBodyRate * params.maxBodyRate - tiltMag * tiltMag),
    );
    yaw = Math.sign(yaw) * Math.min(Math.abs(yaw), yawBudget);
  }
  const magnitude = Math.hypot(tiltX, tiltY, yaw);
  if (magnitude > params.maxBodyRate && magnitude > 0) {
    const scale = params.maxBodyRate / magnitude;
    tiltX *= scale;
    tiltY *= scale;
    yaw *= scale;
  }
  return [tiltX, tiltY, yaw];
}

/**
 * Target orientation whose body +z follows the thrust direction and whose
 * body +x points along the requested world yaw, projected onto that plane.
 */
export function desiredAttitude(thrustDirection: Vec3, yaw: number): Quat {
  assertVec3("thrustDirection", thrustDirection);
  assertFinite("yaw", yaw);

  const zBody = vec3Normalize(thrustDirection);
  let heading: Vec3 = [Math.cos(yaw), Math.sin(yaw), 0];
  let projected = vec3Sub(heading, vec3Scale(zBody, vec3Dot(heading, zBody)));
  if (Math.hypot(...projected) < 1e-9) {
    // Near-horizontal thrust: use the perpendicular horizontal heading.
    heading = [-Math.sin(yaw), Math.cos(yaw), 0];
    projected = vec3Sub(heading, vec3Scale(zBody, vec3Dot(heading, zBody)));
  }
  const xBody = vec3Normalize(projected);
  const yBody = vec3Cross(zBody, xBody);
  const rotation: Mat3 = mat3(
    xBody[0], yBody[0], zBody[0],
    xBody[1], yBody[1], zBody[1],
    xBody[2], yBody[2], zBody[2],
  );
  return quatFromRotationMatrix(rotation);
}
