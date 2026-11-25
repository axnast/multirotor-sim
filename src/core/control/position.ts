/**
 * Outer position loop: world-frame position error -> desired acceleration,
 * tilt-compensated thrust, and a target attitude for the inner loops.
 *
 * - P and I act on position error; D acts on the **true** velocity error
 *   `kd·(v_sp - v)`. There is no derivative filter here: the velocity comes
 *   directly from the measurement (and will come from an estimator in M5), so
 *   differentiating the position would only add phase lag.
 * - Conditional-integration anti-windup mirrors the rate PID, with an added
 *   integral-separation band: the position integral is held while the vehicle
 *   is far from the target *and* moving toward it. Without this, a long
 *   approach winds the integral up and produces a large overshoot.
 * - A persistent-error release unwinds that hold: after a setpoint has been
 *   unchanged for `integralReleaseGuard` seconds and a *filtered signed*
 *   velocity says the vehicle is at rest with a steady error for
 *   `integralDwell` seconds, the integral is released (latched) until the
 *   error re-enters the separation band. This removes the frozen-integral
 *   equilibrium under constant wind or a mass change without changing the
 *   M3 transient. `integralDwell = 0` reproduces the M3 gate exactly.
 * - The rigid-body equation is `v_dot = F/m + g`, so the thrust needed for a
 *   desired acceleration is `f_des = m·(a_des - g)` (with `g = [0,0,-9.81]`, a
 *   level hover needs `+mg` along world z). Thrust is `|f_des|`, clamped to the
 *   configured interval, and body +z follows `f_des` with the tilt clamped.
 * - Yaw comes from the setpoint; roll/pitch come only from the thrust vector.
 */

import { quatNormalize, type Quat } from "../math/quat";
import { vec3Length, vec3Scale, type Vec3 } from "../math/vec3";
import { desiredAttitude } from "./attitude";
import type {
  ControllerParams,
  Measurement,
  PositionAxisState,
  PositionPidState,
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

/** Rest-detection thresholds (documented constants, not viewer-tunable). */
const RELEASE_SPEED = 0.06; // m/s, filtered speed below which the error is "stalled"
const VELOCITY_FILTER_TAU = 0.3; // s, enough to suppress 0.05 m/s sensor noise

export interface PositionStepResult {
  readonly state: PositionAxisState;
  /** Clamped world-frame desired acceleration (m/s²). */
  readonly acceleration: Vec3;
  /** Clamped total thrust (N). */
  readonly thrust: number;
  /** Attitude target that points body +z along the desired thrust vector. */
  readonly desiredOrientation: Quat;
  readonly saturated: readonly [boolean, boolean, boolean];
}

/**
 * Advance the outer loop one step. `state` holds one PID memory per world axis;
 * its `derivative` field mirrors the velocity error for diagnostics.
 */
export function positionStep(
  state: PositionAxisState,
  measurement: Measurement,
  setpoint: Setpoint,
  params: ControllerParams,
  dt: number,
): PositionStepResult {
  assertVec3("measurement.position", measurement.position);
  assertVec3("measurement.velocity", measurement.velocity);
  assertVec3("setpoint.position", setpoint.position);
  assertFinite("setpoint.yaw", setpoint.yaw);
  if (setpoint.velocity !== undefined) assertVec3("setpoint.velocity", setpoint.velocity);
  assertFinite("dt", dt);
  if (dt <= 0) throw new RangeError("dt must be positive");

  const saturated: [boolean, boolean, boolean] = [false, false, false];
  const acceleration: number[] = [];
  const nextState: PositionPidState[] = [];
  const filterAlpha = dt / (VELOCITY_FILTER_TAU + dt);
  const dwell = params.position.integralDwell;

  for (let axis = 0; axis < 3; axis++) {
    const error = setpoint.position[axis]! - measurement.position[axis]!;
    const velocity = measurement.velocity[axis]!;
    const velocityError = (setpoint.velocity?.[axis] ?? 0) - velocity;
    const limit =
      axis === 2 ? params.position.maxAccelZ : params.position.maxAccelXY;
    const previous = state[axis]!;

    // Rest detector: filter the SIGNED velocity so zero-mean sensor noise
    // cannot create a false bias (E|N(0,σ)| = σ√(2/π) would).
    const vf = previous.vf + filterAlpha * (velocity - previous.vf);
    const setpointChanged = setpoint.position[axis]! !== previous.lastSetpoint;
    const setpointAge = setpointChanged ? 0 : previous.setpointAge + dt;
    const near = Math.abs(error) < params.position.integralSeparation;
    const movingAway = error * velocity <= 0;
    const slow = Math.abs(vf) < RELEASE_SPEED;
    const stallTime = slow && !near ? previous.stallTime + dt : 0;

    const armed =
      dwell > 0 && setpointAge >= params.position.integralReleaseGuard;
    let released = armed && (previous.released || stallTime >= dwell);

    const p = params.position.gains.kp * error;
    const d = params.position.gains.kd * velocityError;
    const candidateIntegral =
      previous.integral +
      error * dt * (released ? params.position.integralTrimGain : 1);
    const candidate = p + params.position.gains.ki * candidateIntegral + d;
    const pushFurther =
      (candidate > limit && error > 0) || (candidate < -limit && error < 0);
    if (near || movingAway || pushFurther || setpointChanged) released = false;

    const integral =
      pushFurther || (!near && !movingAway && !released)
        ? previous.integral
        : candidateIntegral;
    const unclamped = p + params.position.gains.ki * integral + d;
    const output = Math.min(limit, Math.max(-limit, unclamped));
    saturated[axis] = pushFurther || unclamped !== output;
    acceleration.push(output);
    nextState.push({
      integral,
      derivative: velocityError,
      lastMeasurement: null,
      vf,
      stallTime,
      released,
      lastSetpoint: setpoint.position[axis]!,
      setpointAge,
    });
  }

  const accel: Vec3 = [acceleration[0]!, acceleration[1]!, acceleration[2]!];
  const desiredForce = vec3Scale(
    [accel[0] - params.gravity[0], accel[1] - params.gravity[1], accel[2] - params.gravity[2]],
    params.mass,
  );
  const magnitude = vec3Length(desiredForce);
  const thrust = Math.min(
    params.position.maxThrust,
    Math.max(params.position.minThrust, magnitude),
  );

  let bodyZ: Vec3 = magnitude > 0 ? vec3Scale(desiredForce, 1 / magnitude) : [0, 0, 1];
  const maxTilt = params.position.maxTilt;
  const horizontal = Math.hypot(bodyZ[0], bodyZ[1]);
  if (bodyZ[2] < Math.cos(maxTilt) && horizontal > 0) {
    const scale = Math.sin(maxTilt) / horizontal;
    bodyZ = [bodyZ[0] * scale, bodyZ[1] * scale, Math.cos(maxTilt)];
  }
  const desiredOrientation = desiredAttitude(bodyZ, setpoint.yaw);

  return {
    state: [nextState[0]!, nextState[1]!, nextState[2]!],
    acceleration: accel,
    thrust,
    desiredOrientation: quatNormalize(desiredOrientation),
    saturated,
  };
}
