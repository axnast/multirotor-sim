/**
 * Multirotor configuration: airframe geometry, motor constants, drag and
 * ground options.
 *
 * Conventions (see docs/conventions.md and docs/multirotor.md):
 * - The body frame is FLU: +x forward, +y left, +z up. Thrust acts along body
 *   +z and drag is expressed in the body frame, acting at the centre of mass.
 * - Motor positions are body-frame offsets from the centre of mass in metres;
 *   all presets mount the rotors in the plane z = 0.
 * - `spin = +1` means the rotor turns counter-clockwise seen from above (about
 *   body +z). Its reaction torque on the airframe is clockwise, about -z.
 * - SI units throughout: kg, m, N, N·m, rad/s, s.
 *
 * Everything here is pure: factories return new, fully resolved configs and
 * never mutate their options. Coefficients in the presets are placeholders
 * chosen to be physically plausible, not measured values.
 */

import { mat3FromDiagonal, mat3Inverse, type Mat3 } from "../math/mat3";
import { vec3Length, type Vec3 } from "../math/vec3";
import { DEFAULT_GRAVITY, type RigidBodyParams } from "../rigidBody";

/** Rotor spin direction about the body +z axis. */
export type SpinDirection = 1 | -1;

export interface MotorConfig {
  /** Rotor hub position in the body frame, relative to the centre of mass (m). */
  readonly position: Vec3;
  /** +1 = counter-clockwise seen from above, -1 = clockwise. */
  readonly spin: SpinDirection;
  /** Thrust coefficient kT in T = kT * ω² (N/(rad/s)²). */
  readonly kT: number;
  /** Reaction-torque coefficient kQ in Q = kQ * ω² (N·m/(rad/s)²). */
  readonly kQ: number;
  /** Upper speed limit (rad/s). */
  readonly maxOmega: number;
  /** Lower speed limit (rad/s); typically 0 for an idling stopped rotor. */
  readonly minOmega: number;
  /** First-order motor lag τ in dω/dt = (ω_cmd - ω) / τ (s). */
  readonly motorTimeConstant: number;
}

/**
 * Linear and quadratic drag coefficients, per body axis.
 *
 * `linear` is in N/(m/s) and `quadratic` in N/(m/s)²; `quadratic` is required
 * in a resolved config, so set it to `[0, 0, 0]` to disable quadratic drag.
 */
export interface DragConfig {
  readonly linear: Vec3;
  readonly quadratic: Vec3;
}

export interface MultirotorConfig {
  /** Total mass (kg), strictly positive. */
  readonly mass: number;
  /** Inertia tensor about the centre of mass, in the body frame (kg·m²). */
  readonly inertia: Mat3;
  /** Gravity acceleration in the world frame (m/s²). */
  readonly gravity: Vec3;
  /** Motors in a fixed order that the mixer and state must preserve. */
  readonly motors: readonly MotorConfig[];
  /** Aerodynamic drag, body frame. */
  readonly drag: DragConfig;
  /**
   * Enables the opt-in ground plane at world z = 0. Contact is a single-point,
   * inelastic-by-default impulse at the centre of mass with Coulomb friction;
   * see ground.ts. No landing gear, tipping, support polygon or ground effect
   * is modelled, and contact applies no torque.
   */
  readonly groundPlane: boolean;
}

/** Optional overrides understood by the airframe factories. */
export interface MultirotorOptions {
  readonly mass?: number;
  readonly armLength?: number;
  readonly inertia?: Mat3;
  readonly gravity?: Vec3;
  readonly kT?: number;
  readonly kQ?: number;
  readonly maxOmega?: number;
  readonly minOmega?: number;
  readonly motorTimeConstant?: number;
  readonly linearDrag?: Vec3;
  readonly quadraticDrag?: Vec3;
  readonly groundPlane?: boolean;
}

/** Placeholder preset mass, roughly a 450 mm-class hexacopter (kg). */
export const DEFAULT_MASS = 1.5;
/** Default rotor-hub radius from the centre of mass (m). */
export const DEFAULT_ARM_LENGTH = 0.25;
/** Placeholder thrust coefficient (N/(rad/s)²). */
export const DEFAULT_KT = 1e-5;
/** Placeholder reaction-torque coefficient (N·m/(rad/s)²). */
export const DEFAULT_KQ = 1.5e-7;
/** Default maximum rotor speed (rad/s). */
export const DEFAULT_MAX_OMEGA = 1000;
/** Default minimum rotor speed (rad/s). */
export const DEFAULT_MIN_OMEGA = 0;
/** Default motor time constant (s). */
export const DEFAULT_MOTOR_TIME_CONSTANT = 0.05;
/** Default linear drag per body axis (N/(m/s)). */
export const DEFAULT_LINEAR_DRAG: Vec3 = [0.25, 0.25, 0.4];
/** Default quadratic drag per body axis (N/(m/s)²). */
export const DEFAULT_QUADRATIC_DRAG: Vec3 = [0.02, 0.02, 0.03];
/** Ground contact is opt-in. */
export const DEFAULT_GROUND_PLANE = false;

/** Quadcopter X layout: first rotor at 45°, then every 90° (rad). */
export const QUAD_START_ANGLE = Math.PI / 4;
/** Hexacopter layout with one rotor on body +x, then every 60° (rad). */
export const HEX_START_ANGLE = 0;
/** Octocopter layout with one rotor on body +x, then every 45° (rad). */
export const OCTO_START_ANGLE = 0;

function assertFiniteVec3(name: string, v: Vec3): void {
  if (!Number.isFinite(v[0]) || !Number.isFinite(v[1]) || !Number.isFinite(v[2])) {
    throw new RangeError(`${name} must contain only finite numbers`);
  }
}

function assertNonNegativeVec3(name: string, v: Vec3): void {
  assertFiniteVec3(name, v);
  if (v[0] < 0 || v[1] < 0 || v[2] < 0) {
    throw new RangeError(`${name} components must be non-negative`);
  }
}

/** Throws a RangeError if the config is structurally invalid. */
export function validateConfig(config: MultirotorConfig): void {
  if (!Number.isFinite(config.mass) || config.mass <= 0) {
    throw new RangeError("mass must be a finite positive number");
  }
  for (const value of config.inertia) {
    if (!Number.isFinite(value)) {
      throw new RangeError("inertia must contain only finite numbers");
    }
  }
  // Throws for a singular (or non-positive-definite) inertia tensor.
  mat3Inverse(config.inertia);

  assertFiniteVec3("gravity", config.gravity);
  assertNonNegativeVec3("drag.linear", config.drag.linear);
  assertNonNegativeVec3("drag.quadratic", config.drag.quadratic);

  if (typeof config.groundPlane !== "boolean") {
    throw new RangeError("groundPlane must be a boolean");
  }

  if (config.motors.length < 4) {
    throw new RangeError("a multirotor needs at least four motors");
  }
  for (const [index, motor] of config.motors.entries()) {
    const name = `motors[${index}]`;
    assertFiniteVec3(`${name}.position`, motor.position);

    if (motor.spin !== 1 && motor.spin !== -1) {
      throw new RangeError(`${name}.spin must be +1 or -1`);
    }
    if (!Number.isFinite(motor.kT) || motor.kT <= 0) {
      throw new RangeError(`${name}.kT must be a finite positive number`);
    }
    if (!Number.isFinite(motor.kQ) || motor.kQ <= 0) {
      throw new RangeError(`${name}.kQ must be a finite positive number`);
    }
    if (!Number.isFinite(motor.maxOmega) || motor.maxOmega <= 0) {
      throw new RangeError(`${name}.maxOmega must be a finite positive number`);
    }
    if (!Number.isFinite(motor.minOmega) || motor.minOmega < 0) {
      throw new RangeError(`${name}.minOmega must be a finite non-negative number`);
    }
    if (motor.minOmega >= motor.maxOmega) {
      throw new RangeError(`${name}.minOmega must be less than maxOmega`);
    }
    if (
      !Number.isFinite(motor.motorTimeConstant) ||
      motor.motorTimeConstant <= 0
    ) {
      throw new RangeError(
        `${name}.motorTimeConstant must be a finite positive number`,
      );
    }
  }
}

/**
 * Thin-ring inertia about the centre of mass:
 * Ixx = Iyy = ½ m R², Izz = m R².
 *
 * Used as the placeholder airframe inertia when no tensor is supplied.
 */
export function ringInertia(mass: number, radius: number): Mat3 {
  if (!Number.isFinite(mass) || mass <= 0) {
    throw new RangeError("mass must be a finite positive number");
  }
  if (!Number.isFinite(radius) || radius <= 0) {
    throw new RangeError("radius must be a finite positive number");
  }
  const lateral = 0.5 * mass * radius * radius;
  return mat3FromDiagonal([lateral, lateral, mass * radius * radius]);
}

/** View a config as the rigid-body parameters consumed by Milestone 1. */
export function rigidBodyParamsOf(config: MultirotorConfig): RigidBodyParams {
  return {
    mass: config.mass,
    inertia: config.inertia,
    gravity: config.gravity,
  };
}

/** Weight force magnitude m·|g| (N). */
function weightOf(config: MultirotorConfig): number {
  return config.mass * vec3Length(config.gravity);
}

/**
 * Motor speeds (rad/s) that support a level hover, split evenly across the
 * rotors. Assumes still air (drag vanishes at zero airspeed) and that thrust
 * exactly opposes gravity. Values may exceed a rotor's limits; the mixer is
 * responsible for saturation.
 */
export function hoverMotorSpeeds(config: MultirotorConfig): number[] {
  const thrustPerMotor = weightOf(config) / config.motors.length;
  return config.motors.map((motor) => Math.sqrt(thrustPerMotor / motor.kT));
}

/** Total thrust with every rotor at maxOmega (N). */
export function maxThrust(config: MultirotorConfig): number {
  return config.motors.reduce(
    (total, motor) => total + motor.kT * motor.maxOmega * motor.maxOmega,
    0,
  );
}

/** Total thrust with every rotor at minOmega (N). */
export function minThrust(config: MultirotorConfig): number {
  return config.motors.reduce(
    (total, motor) => total + motor.kT * motor.minOmega * motor.minOmega,
    0,
  );
}

/** Ratio of maximum thrust to weight; values above 1 allow hover and climb. */
export function thrustToWeight(config: MultirotorConfig): number {
  const weight = weightOf(config);
  if (weight === 0) {
    throw new RangeError("thrustToWeight requires non-zero gravity");
  }
  return maxThrust(config) / weight;
}

interface ResolvedOptions {
  readonly mass: number;
  readonly armLength: number;
  readonly inertia: Mat3;
  readonly gravity: Vec3;
  readonly kT: number;
  readonly kQ: number;
  readonly maxOmega: number;
  readonly minOmega: number;
  readonly motorTimeConstant: number;
  readonly linearDrag: Vec3;
  readonly quadraticDrag: Vec3;
  readonly groundPlane: boolean;
}

function resolveOptions(options: MultirotorOptions): ResolvedOptions {
  const mass = options.mass ?? DEFAULT_MASS;
  const armLength = options.armLength ?? DEFAULT_ARM_LENGTH;

  if (!Number.isFinite(mass) || mass <= 0) {
    throw new RangeError("mass must be a finite positive number");
  }
  if (!Number.isFinite(armLength) || armLength <= 0) {
    throw new RangeError("armLength must be a finite positive number");
  }

  return {
    mass,
    armLength,
    inertia: options.inertia ?? ringInertia(mass, armLength),
    gravity: options.gravity ?? DEFAULT_GRAVITY,
    kT: options.kT ?? DEFAULT_KT,
    kQ: options.kQ ?? DEFAULT_KQ,
    maxOmega: options.maxOmega ?? DEFAULT_MAX_OMEGA,
    minOmega: options.minOmega ?? DEFAULT_MIN_OMEGA,
    motorTimeConstant:
      options.motorTimeConstant ?? DEFAULT_MOTOR_TIME_CONSTANT,
    linearDrag: options.linearDrag ?? DEFAULT_LINEAR_DRAG,
    quadraticDrag: options.quadraticDrag ?? DEFAULT_QUADRATIC_DRAG,
    groundPlane: options.groundPlane ?? DEFAULT_GROUND_PLANE,
  };
}

type MotorConstants = Pick<
  MotorConfig,
  "kT" | "kQ" | "maxOmega" | "minOmega" | "motorTimeConstant"
>;

/**
 * Evenly spaced rotors on a circle of radius `armLength` in the z = 0 plane.
 * The first rotor sits at `startAngle` (measured counter-clockwise from body
 * +x) and spins alternate with the index, starting counter-clockwise.
 */
function radialMotors(
  count: number,
  armLength: number,
  startAngle: number,
  constants: MotorConstants,
): MotorConfig[] {
  const motors: MotorConfig[] = [];
  for (let index = 0; index < count; index++) {
    const angle = startAngle + (2 * Math.PI * index) / count;
    motors.push({
      position: [
        armLength * Math.cos(angle),
        armLength * Math.sin(angle),
        0,
      ],
      spin: index % 2 === 0 ? 1 : -1,
      ...constants,
    });
  }
  return motors;
}

function buildAirframe(
  count: number,
  startAngle: number,
  options: MultirotorOptions,
): MultirotorConfig {
  const resolved = resolveOptions(options);
  const config: MultirotorConfig = {
    mass: resolved.mass,
    inertia: resolved.inertia,
    gravity: resolved.gravity,
    motors: radialMotors(count, resolved.armLength, startAngle, {
      kT: resolved.kT,
      kQ: resolved.kQ,
      maxOmega: resolved.maxOmega,
      minOmega: resolved.minOmega,
      motorTimeConstant: resolved.motorTimeConstant,
    }),
    drag: {
      linear: resolved.linearDrag,
      quadratic: resolved.quadraticDrag,
    },
    groundPlane: resolved.groundPlane,
  };
  validateConfig(config);
  return config;
}

/** Quadcopter in an X layout (rotors at 45°, 135°, 225°, 315°). */
export function makeQuad(options: MultirotorOptions = {}): MultirotorConfig {
  return buildAirframe(4, QUAD_START_ANGLE, options);
}

/** Hexacopter with one rotor on body +x and the rest every 60°. */
export function makeHex(options: MultirotorOptions = {}): MultirotorConfig {
  return buildAirframe(6, HEX_START_ANGLE, options);
}

/** Octocopter with one rotor on body +x and the rest every 45°. */
export function makeOcto(options: MultirotorOptions = {}): MultirotorConfig {
  return buildAirframe(8, OCTO_START_ANGLE, options);
}
