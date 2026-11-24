/**
 * Test-only helpers shared by the Milestone 2 integration suite and, later,
 * the Milestone 3 controller tests.
 *
 * These live under `tests/` deliberately: they are not part of the pure core
 * API and are never imported from `src/`.
 */

import { expect } from "vitest";

import { QUAT_IDENTITY, type Quat } from "../../src/core/math/quat";
import type { Vec3 } from "../../src/core/math/vec3";
import type { RigidBodyState } from "../../src/core/rigidBody";
import {
  hoverMotorSpeeds,
  type MotorConfig,
  type MultirotorConfig,
} from "../../src/core/multirotor/config";
import {
  stepMultirotor,
  STILL_AIR,
  type Environment,
  type MultirotorState,
} from "../../src/core/multirotor/dynamics";
import {
  DEFAULT_GROUND_PARAMS,
  type GroundParams,
} from "../../src/core/multirotor/ground";
import type { Wrench } from "../../src/core/multirotor/mixer";

/** Default absolute tolerance for algebraic identities. */
export const TOL = 1e-12;

/** Indexed access with an explicit error, avoiding unchecked-index surprises. */
export function at<T>(values: readonly T[], index: number): T {
  const value = values[index];
  if (value === undefined) {
    throw new Error(`missing value at index ${index}`);
  }
  return value;
}

/** Assert two scalars agree within `tol`. */
export function close(actual: number, expected: number, tol = TOL): void {
  expect(Math.abs(actual - expected)).toBeLessThan(tol);
}

/** Assert two vectors agree component-wise within `tol`. */
export function vecClose(
  actual: readonly number[],
  expected: readonly number[],
  tol = TOL,
): void {
  expect(actual.length).toBe(expected.length);
  for (let index = 0; index < expected.length; index++) {
    close(at(actual, index), at(expected, index), tol);
  }
}

/** Assert two quaternions agree component-wise within `tol`. */
export function quatClose(actual: Quat, expected: Quat, tol = TOL): void {
  vecClose(actual, expected, tol);
}

/** Euclidean norm of a Hamilton quaternion `[w, x, y, z]`. */
export function quatNorm(q: Quat): number {
  return Math.sqrt(
    q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3],
  );
}

/** Dot product: `|a·b| = 1` means the quaternions represent the same attitude. */
export function quatDot(a: Quat, b: Quat): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

/** Optional per-field overrides for a fresh state. */
export interface StateOverrides {
  readonly body?: RigidBodyState;
  readonly position?: Vec3;
  readonly velocity?: Vec3;
  readonly orientation?: Quat;
  readonly angularVelocity?: Vec3;
  readonly motorSpeeds?: readonly number[];
}

/** A fresh state at rest, with motors at hover unless overridden. */
export function makeState(
  config: MultirotorConfig,
  overrides: StateOverrides = {},
): MultirotorState {
  const body: RigidBodyState = overrides.body ?? {
    position: overrides.position ?? [0, 0, 0],
    velocity: overrides.velocity ?? [0, 0, 0],
    orientation: overrides.orientation ?? QUAT_IDENTITY,
    angularVelocity: overrides.angularVelocity ?? [0, 0, 0],
  };
  return {
    body,
    motorSpeeds: overrides.motorSpeeds ?? hoverMotorSpeeds(config),
  };
}

/** Hold `commands` over `steps` fixed RK4 steps. */
export function stepN(
  state: MultirotorState,
  config: MultirotorConfig,
  commands: readonly number[],
  steps: number,
  dt: number,
  environment: Environment = STILL_AIR,
  ground: GroundParams = DEFAULT_GROUND_PARAMS,
): MultirotorState {
  let current = state;
  for (let index = 0; index < steps; index++) {
    current = stepMultirotor(current, config, commands, dt, environment, ground);
  }
  return current;
}

/** Level hover wrench in the body frame. */
export function hoverWrench(config: MultirotorConfig): Wrench {
  return [config.mass * 9.81, 0, 0, 0];
}

/** Hover thrust plus one pure body torque on axis 1 (x), 2 (y) or 3 (z). */
export function torqueWrench(
  config: MultirotorConfig,
  axis: 1 | 2 | 3,
  value: number,
): Wrench {
  const components: [number, number, number, number] = [config.mass * 9.81, 0, 0, 0];
  components[axis] = value;
  return components;
}

/** Mean speed of rotors with the given spin direction. */
export function meanSpeed(
  config: MultirotorConfig,
  speeds: readonly number[],
  spin: 1 | -1,
): number {
  const selected: number[] = [];
  for (const [index, motor] of config.motors.entries()) {
    if (motor.spin === spin) {
      selected.push(at(speeds, index));
    }
  }
  return selected.reduce((sum, value) => sum + value, 0) / selected.length;
}

/** Motor config by index, with an explicit error. */
export function motorAt(config: MultirotorConfig, index: number): MotorConfig {
  const motor = config.motors[index];
  if (motor === undefined) {
    throw new Error(`config has no motor at index ${index}`);
  }
  return motor;
}
