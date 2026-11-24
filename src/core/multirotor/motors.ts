/**
 * Motor model: thrust, reaction torque and first-order speed lag.
 *
 * Conventions (see docs/conventions.md and docs/multirotor.md):
 * - Rotor `spin = +1` turns counter-clockwise seen from above, i.e. about body
 *   +z; its reaction torque on the airframe is clockwise, about -z.
 * - `omega` is a non-negative speed magnitude in rad/s.
 *
 * Simplifications:
 * - Only the steady reaction torque kQ·ω² is modelled. There is no spin-up
 *   reaction torque from accelerating the rotor's own inertia, and no rotor
 *   gyroscopic torque from the airframe rotating a spinning rotor.
 * - Thrust and torque follow the instantaneous speed; the only motor dynamics
 *   is the first-order lag of the speed itself.
 * - Blade flapping, ground effect, inflow and other aerodynamic effects are
 *   ignored.
 */

import { type Vec3 } from "../math/vec3";
import type { MotorConfig } from "./config";

function assertFinite(name: string, value: number): void {
  if (!Number.isFinite(value)) {
    throw new RangeError(`${name} must be a finite number`);
  }
}

/** Thrust produced by a rotor at speed `omega`: T = kT·ω² (N). */
export function motorThrust(omega: number, motor: MotorConfig): number {
  assertFinite("omega", omega);
  return motor.kT * omega * omega;
}

/**
 * Reaction torque on the airframe from rotor drag, in the body frame.
 *
 * A rotor with `spin = +1` pushes the airframe clockwise: Q = -spin·kQ·ω²
 * about body z. The expression is written as ω·|ω| so that it stays odd in ω
 * (a rotor spinning backwards reverses its reaction) while agreeing with
 * -spin·kQ·ω² for the physical domain ω ≥ 0.
 */
export function motorReactionTorque(omega: number, motor: MotorConfig): Vec3 {
  assertFinite("omega", omega);
  return [0, 0, -motor.spin * motor.kQ * omega * Math.abs(omega)];
}

/**
 * First-order motor lag: dω/dt = (ω_cmd - ω) / τ.
 *
 * `command` is the requested speed; the caller decides how it is held (the
 * simulator uses zero-order hold over each integration step).
 */
export function motorSpeedDerivative(
  omega: number,
  command: number,
  motor: MotorConfig,
): number {
  assertFinite("omega", omega);
  assertFinite("command", command);
  return (command - omega) / motor.motorTimeConstant;
}

/** Clamp a requested rotor speed to [minOmega, maxOmega]. */
export function clampMotorCommand(
  command: number,
  motor: MotorConfig,
): number {
  assertFinite("command", command);
  return Math.min(motor.maxOmega, Math.max(motor.minOmega, command));
}
