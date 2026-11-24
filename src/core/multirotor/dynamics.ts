/**
 * Multirotor dynamics: the Milestone 1 rigid-body state plus one scalar per
 * rotor, integrated together with classical RK4.
 *
 * Conventions and equations (see docs/multirotor.md, added in the docs commit):
 *   v_air_w = v_world - wind_world
 *   v_air_b = R(q)ᵀ v_air_w
 *   F_drag_b = -(c1 ⊙ v_air_b + c2 ⊙ |v_air_b| · v_air_b)
 *   f_i      = kT_i ω_i²
 *   F_mot_b  = Σ f_i ẑ
 *   τ_mot_b  = Σ (r_i × f_i ẑ) + Σ (0, 0, -spin_i kQ_i ω_i|ω_i|)
 *   F_w      = R(q)(F_mot_b + F_drag_b)
 *   τ_b      = τ_mot_b
 *
 * The rigid-body equations themselves are unchanged and are delegated to
 * `rigidBody.derivatives`. Drag acts at the centre of mass, so it adds no
 * torque. There is no rotor gyroscopic torque, spin-up reaction torque, blade
 * flapping, ground effect or inflow model; see docs/multirotor.md.
 *
 * Numerical notes:
 * - Motor commands are zero-order held and clamped over a whole step. Every
 *   RK4 stage sees the same values.
 * - Forces are evaluated with a *unit* quaternion so that they depend only on
 *   orientation, not on the magnitude of the quaternion. The integrated state
 *   is renormalized once after the full step, as in Milestone 1; stage
 *   quaternions are never renormalized.
 * - Intermediate motor speeds are not clamped (that would make the vector
 *   field non-smooth). The final speeds are projected onto their limits.
 * - `dt` is caller-chosen: `dt ≤ τ/10` gives sub-0.01 % lag error and RK4's
 *   linear stability limit is `dt < 2.79 τ`. No substepping is performed.
 */

import {
  quatConjugate,
  quatNormalize,
  quatRotateVector,
  type Quat,
} from "../math/quat";
import { vec3Add, vec3Length, vec3Sub, type Vec3 } from "../math/vec3";
import {
  derivatives,
  type RigidBodyState,
  type StateDerivative,
} from "../rigidBody";
import {
  rigidBodyParamsOf,
  validateConfig,
  type DragConfig,
  type MotorConfig,
  type MultirotorConfig,
} from "./config";
import {
  clampMotorCommand,
  motorReactionTorque,
  motorSpeedDerivative,
  motorThrust,
} from "./motors";

/** Rigid-body state plus rotor speeds, in the same order as `config.motors`. */
export interface MultirotorState {
  readonly body: RigidBodyState;
  readonly motorSpeeds: readonly number[];
}

/** Air velocity in the world frame (m/s). */
export interface Environment {
  readonly windVelocity: Vec3;
}

/** Still air. Shared immutable default; never mutate it. */
export const STILL_AIR: Environment = { windVelocity: [0, 0, 0] };

/** Motor-only force and torque in the body frame (no drag, no gravity). */
export interface BodyWrench {
  readonly forceBody: Vec3;
  readonly torqueBody: Vec3;
}

/** Time derivative of `MultirotorState`; `body.orientation` is q_dot. */
export interface MultirotorDerivative {
  readonly body: StateDerivative;
  readonly motorSpeeds: readonly number[];
}

function assertFiniteNumber(name: string, value: number): void {
  if (!Number.isFinite(value)) {
    throw new RangeError(`${name} must be a finite number`);
  }
}

function assertFiniteVec3(name: string, v: Vec3): void {
  if (!Number.isFinite(v[0]) || !Number.isFinite(v[1]) || !Number.isFinite(v[2])) {
    throw new RangeError(`${name} must contain only finite numbers`);
  }
}

function assertFiniteQuat(name: string, q: Quat): void {
  if (
    !Number.isFinite(q[0]) ||
    !Number.isFinite(q[1]) ||
    !Number.isFinite(q[2]) ||
    !Number.isFinite(q[3])
  ) {
    throw new RangeError(`${name} must contain only finite numbers`);
  }
}

function assertValidBody(body: RigidBodyState): void {
  assertFiniteVec3("body.position", body.position);
  assertFiniteVec3("body.velocity", body.velocity);
  assertFiniteQuat("body.orientation", body.orientation);
  assertFiniteVec3("body.angularVelocity", body.angularVelocity);
  if (
    body.orientation[0] === 0 &&
    body.orientation[1] === 0 &&
    body.orientation[2] === 0 &&
    body.orientation[3] === 0
  ) {
    throw new RangeError("body.orientation must not be the zero quaternion");
  }
}

function assertValidEnvironment(environment: Environment): void {
  assertFiniteVec3("environment.windVelocity", environment.windVelocity);
}

function assertValidDt(dt: number): void {
  if (!Number.isFinite(dt) || dt <= 0) {
    throw new RangeError("dt must be a finite positive number");
  }
}

/** Indexed access with an explicit error, avoiding `noUncheckedIndexedAccess` defaults. */
function at(values: readonly number[], index: number): number {
  const value = values[index];
  if (value === undefined) {
    throw new RangeError(`missing value at index ${index}`);
  }
  return value;
}

function motorAt(motors: readonly MotorConfig[], index: number): MotorConfig {
  const motor = motors[index];
  if (motor === undefined) {
    throw new RangeError(`missing motor at index ${index}`);
  }
  return motor;
}

function assertValidState(state: MultirotorState, config: MultirotorConfig): void {
  assertValidBody(state.body);
  if (state.motorSpeeds.length !== config.motors.length) {
    throw new RangeError(
      `motorSpeeds has ${state.motorSpeeds.length} entries, expected ${config.motors.length}`,
    );
  }
  for (const [index, speed] of state.motorSpeeds.entries()) {
    assertFiniteNumber(`motorSpeeds[${index}]`, speed);
    const maxOmega = motorAt(config.motors, index).maxOmega;
    if (speed < 0 || speed > maxOmega) {
      throw new RangeError(
        `motorSpeeds[${index}] must be within [0, maxOmega]`,
      );
    }
  }
}

function assertValidCommands(
  commands: readonly number[],
  config: MultirotorConfig,
): void {
  if (commands.length !== config.motors.length) {
    throw new RangeError(
      `motorCommands has ${commands.length} entries, expected ${config.motors.length}`,
    );
  }
  for (const [index, command] of commands.entries()) {
    assertFiniteNumber(`motorCommands[${index}]`, command);
  }
}

function clampCommands(
  commands: readonly number[],
  config: MultirotorConfig,
): number[] {
  return commands.map((command, index) =>
    clampMotorCommand(command, motorAt(config.motors, index)),
  );
}

/**
 * Body-frame drag from air-relative velocity. `qUnit` must be a unit
 * quaternion; the quadratic term uses the isotropic speed |v|.
 */
function dragForceBodyFrom(
  qUnit: Quat,
  velocityWorld: Vec3,
  windWorld: Vec3,
  drag: DragConfig,
): Vec3 {
  const relativeWorld = vec3Sub(velocityWorld, windWorld);
  const relativeBody = quatRotateVector(quatConjugate(qUnit), relativeWorld);
  const speed = vec3Length(relativeBody);
  return [
    -(drag.linear[0] * relativeBody[0] + drag.quadratic[0] * speed * relativeBody[0]),
    -(drag.linear[1] * relativeBody[1] + drag.quadratic[1] * speed * relativeBody[1]),
    -(drag.linear[2] * relativeBody[2] + drag.quadratic[2] * speed * relativeBody[2]),
  ];
}

/** Motor-only force/torque in the body frame. Inputs must be pre-validated. */
function computeMotorWrench(
  speeds: readonly number[],
  config: MultirotorConfig,
): BodyWrench {
  let fx = 0;
  let fy = 0;
  let fz = 0;
  let tx = 0;
  let ty = 0;
  let tz = 0;

  for (const [index, motor] of config.motors.entries()) {
    const omega = at(speeds, index);
    const thrust = motorThrust(omega, motor);

    // r × (0, 0, f) = (r_y f, -r_x f, 0). r_z does not contribute.
    fz += thrust;
    tx += motor.position[1] * thrust;
    ty -= motor.position[0] * thrust;
    tz += motorReactionTorque(omega, motor)[2];
  }

  return { forceBody: [fx, fy, fz], torqueBody: [tx, ty, tz] };
}

/** Motor-only force/torque in the body frame. */
export function motorWrench(
  state: MultirotorState,
  config: MultirotorConfig,
): BodyWrench {
  validateConfig(config);
  assertValidState(state, config);
  return computeMotorWrench(state.motorSpeeds, config);
}

/** Sum of rotor thrusts (N), i.e. the body-z component of the motor force. */
export function totalThrust(
  state: MultirotorState,
  config: MultirotorConfig,
): number {
  validateConfig(config);
  assertValidState(state, config);
  return computeMotorWrench(state.motorSpeeds, config).forceBody[2];
}

/** Aerodynamic drag force in the body frame, acting at the centre of mass. */
export function dragForceBody(
  body: RigidBodyState,
  config: MultirotorConfig,
  environment: Environment = STILL_AIR,
): Vec3 {
  validateConfig(config);
  assertValidBody(body);
  assertValidEnvironment(environment);
  return dragForceBodyFrom(
    quatNormalize(body.orientation),
    body.velocity,
    environment.windVelocity,
    config.drag,
  );
}

/** Aerodynamic drag force in the world frame. */
export function dragForceWorld(
  body: RigidBodyState,
  config: MultirotorConfig,
  environment: Environment = STILL_AIR,
): Vec3 {
  validateConfig(config);
  assertValidBody(body);
  assertValidEnvironment(environment);
  const qUnit = quatNormalize(body.orientation);
  return quatRotateVector(
    qUnit,
    dragForceBodyFrom(
      qUnit,
      body.velocity,
      environment.windVelocity,
      config.drag,
    ),
  );
}

/**
 * Evaluate the extended equations of motion. Commands are clamped here but
 * are not applied to the motor state; they are only used for dω/dt.
 */
export function multirotorDerivatives(
  state: MultirotorState,
  config: MultirotorConfig,
  motorCommands: readonly number[],
  environment: Environment = STILL_AIR,
): MultirotorDerivative {
  validateConfig(config);
  assertValidState(state, config);
  assertValidCommands(motorCommands, config);
  assertValidEnvironment(environment);
  return evaluate(state, config, clampCommands(motorCommands, config), environment);
}

/** Pre-validated derivative, used for every RK4 stage. */
function evaluate(
  state: MultirotorState,
  config: MultirotorConfig,
  clampedCommands: readonly number[],
  environment: Environment,
): MultirotorDerivative {
  const qUnit = quatNormalize(state.body.orientation);
  const motor = computeMotorWrench(state.motorSpeeds, config);
  const dragBody = dragForceBodyFrom(
    qUnit,
    state.body.velocity,
    environment.windVelocity,
    config.drag,
  );
  const forceWorld = quatRotateVector(qUnit, vec3Add(motor.forceBody, dragBody));

  const body = derivatives(state.body, rigidBodyParamsOf(config), {
    forceWorld,
    torqueBody: motor.torqueBody,
  });

  const motorSpeeds = clampedCommands.map((command, index) =>
    motorSpeedDerivative(
      at(state.motorSpeeds, index),
      command,
      motorAt(config.motors, index),
    ),
  );

  return { body, motorSpeeds };
}

/** state + h·derivative, component-wise. The quaternion is not normalized. */
function addScaled(
  state: MultirotorState,
  derivative: MultirotorDerivative,
  h: number,
): MultirotorState {
  const body = state.body;
  const d = derivative.body;
  return {
    body: {
      position: [
        body.position[0] + h * d.position[0],
        body.position[1] + h * d.position[1],
        body.position[2] + h * d.position[2],
      ],
      velocity: [
        body.velocity[0] + h * d.velocity[0],
        body.velocity[1] + h * d.velocity[1],
        body.velocity[2] + h * d.velocity[2],
      ],
      orientation: [
        body.orientation[0] + h * d.orientation[0],
        body.orientation[1] + h * d.orientation[1],
        body.orientation[2] + h * d.orientation[2],
        body.orientation[3] + h * d.orientation[3],
      ] as Quat,
      angularVelocity: [
        body.angularVelocity[0] + h * d.angularVelocity[0],
        body.angularVelocity[1] + h * d.angularVelocity[1],
        body.angularVelocity[2] + h * d.angularVelocity[2],
      ],
    },
    motorSpeeds: state.motorSpeeds.map(
      (speed, index) => speed + h * at(derivative.motorSpeeds, index),
    ),
  };
}

function weighted4(a: number, b: number, c: number, d: number): number {
  return (a + 2 * b + 2 * c + d) / 6;
}

function weightedVec3(k1: Vec3, k2: Vec3, k3: Vec3, k4: Vec3): Vec3 {
  return [
    weighted4(k1[0], k2[0], k3[0], k4[0]),
    weighted4(k1[1], k2[1], k3[1], k4[1]),
    weighted4(k1[2], k2[2], k3[2], k4[2]),
  ];
}

function weightedQuat(k1: Quat, k2: Quat, k3: Quat, k4: Quat): Quat {
  return [
    weighted4(k1[0], k2[0], k3[0], k4[0]),
    weighted4(k1[1], k2[1], k3[1], k4[1]),
    weighted4(k1[2], k2[2], k3[2], k4[2]),
    weighted4(k1[3], k2[3], k3[3], k4[3]),
  ];
}

function weightedDerivative(
  k1: MultirotorDerivative,
  k2: MultirotorDerivative,
  k3: MultirotorDerivative,
  k4: MultirotorDerivative,
): MultirotorDerivative {
  return {
    body: {
      position: weightedVec3(
        k1.body.position, k2.body.position, k3.body.position, k4.body.position,
      ),
      velocity: weightedVec3(
        k1.body.velocity, k2.body.velocity, k3.body.velocity, k4.body.velocity,
      ),
      orientation: weightedQuat(
        k1.body.orientation, k2.body.orientation,
        k3.body.orientation, k4.body.orientation,
      ),
      angularVelocity: weightedVec3(
        k1.body.angularVelocity, k2.body.angularVelocity,
        k3.body.angularVelocity, k4.body.angularVelocity,
      ),
    },
    motorSpeeds: k1.motorSpeeds.map((_, index) =>
      weighted4(
        at(k1.motorSpeeds, index),
        at(k2.motorSpeeds, index),
        at(k3.motorSpeeds, index),
        at(k4.motorSpeeds, index),
      ),
    ),
  };
}

/**
 * One classical RK4 step over the extended state. Commands are held constant
 * across the step. The quaternion is normalized once after the step and the
 * resulting motor speeds are projected onto [minOmega, maxOmega].
 */
export function stepMultirotor(
  state: MultirotorState,
  config: MultirotorConfig,
  motorCommands: readonly number[],
  dt: number,
  environment: Environment = STILL_AIR,
): MultirotorState {
  assertValidDt(dt);
  validateConfig(config);
  assertValidState(state, config);
  assertValidCommands(motorCommands, config);
  assertValidEnvironment(environment);

  const commands = clampCommands(motorCommands, config);
  const k1 = evaluate(state, config, commands, environment);
  const k2 = evaluate(addScaled(state, k1, dt / 2), config, commands, environment);
  const k3 = evaluate(addScaled(state, k2, dt / 2), config, commands, environment);
  const k4 = evaluate(addScaled(state, k3, dt), config, commands, environment);

  const next = addScaled(state, weightedDerivative(k1, k2, k3, k4), dt);
  return {
    body: {
      ...next.body,
      orientation: quatNormalize(next.body.orientation),
    },
    motorSpeeds: next.motorSpeeds.map((speed, index) => {
      const motor = motorAt(config.motors, index);
      return Math.min(motor.maxOmega, Math.max(motor.minOmega, speed));
    }),
  };
}
