/**
 * Discrete payload shift. A point mass is attached at a body-frame position
 * expressed in the *original* airframe frame; the whole config is rebuilt
 * about the new centre of mass so the existing rigid-body dynamics work
 * unchanged (thrust then produces torque about the shifted COM).
 *
 *   M  = m0 + mp
 *   d  = mp * r_p / M                 (new COM relative to the old COM)
 *   I' = I0 + m0 * PA(d) + mp * PA(r_p - d)
 *   r_i' = r_i - d
 *
 * where PA(v) = |v|^2 * E - v v^T is the parallel-axis tensor. Both mass
 * terms are included; the payload's own spin inertia is neglected because it
 * is modelled as a point mass. This is a placeholder, not a structural model.
 */

import { validateConfig, type MultirotorConfig } from "../multirotor/config";
import type { Vec3 } from "../math/vec3";
import type { PayloadSpec } from "./types";

/** Named case used by the tests, demo and viewer: 0.3 kg at x = 0.12 m. */
export const PAYLOAD_2CM: PayloadSpec = { mass: 0.3, position: [0.12, 0, 0] };

/** Centred +20% mass: isolates the mass mismatch from any COM shift. */
export const PAYLOAD_MASS_20PCT: PayloadSpec = { mass: 0.3, position: [0, 0, 0] };

/** Small off-centre mass: isolates trim from a large mass mismatch. */
export const PAYLOAD_TRIM: PayloadSpec = { mass: 0.01, position: [0.3, 0, 0] };

/**
 * Baseline survival-envelope labels (M5 measurements, not controller targets):
 * the largest case that survives the 120 s envelope (0.2% mass, 1 mm COM shift)
 * and the named 20% / 2 cm stress case that does not. See docs/disturbances.md.
 */
export const PAYLOAD_SMALL: PayloadSpec = { mass: 0.003, position: [0.501, 0, 0] };
export const PAYLOAD_LARGE_STRESS: PayloadSpec = PAYLOAD_2CM;

function assertFinite(name: string, value: number): void {
  if (!Number.isFinite(value)) {
    throw new RangeError(`${name} must be a finite number`);
  }
}

export function validatePayload(payload: PayloadSpec): void {
  assertFinite("payload.mass", payload.mass);
  if (payload.mass <= 0) {
    throw new RangeError("payload.mass must be positive");
  }
  for (let axis = 0; axis < 3; axis++) {
    assertFinite(`payload.position[${axis}]`, payload.position[axis]!);
  }
}

/** New centre-of-mass offset relative to the original COM. */
export function payloadCenterOfMass(
  config: MultirotorConfig,
  payload: PayloadSpec,
): Vec3 {
  validatePayload(payload);
  const total = config.mass + payload.mass;
  return [
    payload.mass * payload.position[0] / total,
    payload.mass * payload.position[1] / total,
    payload.mass * payload.position[2] / total,
  ];
}

/** Parallel-axis tensor in row-major order (9 numbers). */
export function parallelAxisTensor(offset: Vec3): readonly number[] {
  const x = offset[0];
  const y = offset[1];
  const z = offset[2];
  for (const value of [x, y, z]) assertFinite("parallel-axis offset", value);
  return [
    y * y + z * z, -x * y, -x * z,
    -y * x, x * x + z * z, -y * z,
    -z * x, -z * y, x * x + y * y,
  ];
}

/** Return a new config with the point payload included. Inputs are unchanged. */
export function applyPayload(
  config: MultirotorConfig,
  payload: PayloadSpec,
): MultirotorConfig {
  validatePayload(payload);
  const originalMass = config.mass;
  const totalMass = originalMass + payload.mass;
  const com = payloadCenterOfMass(config, payload);
  const payloadRelative: Vec3 = [
    payload.position[0] - com[0],
    payload.position[1] - com[1],
    payload.position[2] - com[2],
  ];
  const bodyShift = parallelAxisTensor(com);
  const payloadShift = parallelAxisTensor(payloadRelative);
  const inertia = config.inertia.map((value, index) =>
    value +
      originalMass * bodyShift[index]! +
      payload.mass * payloadShift[index]!,
  );
  const shiftedInertia: MultirotorConfig["inertia"] = [
    inertia[0]!, inertia[1]!, inertia[2]!,
    inertia[3]!, inertia[4]!, inertia[5]!,
    inertia[6]!, inertia[7]!, inertia[8]!,
  ];
  const result: MultirotorConfig = {
    ...config,
    mass: totalMass,
    inertia: shiftedInertia,
    motors: config.motors.map((motor) => ({
      ...motor,
      position: [
        motor.position[0] - com[0],
        motor.position[1] - com[1],
        motor.position[2] - com[2],
      ],
    })),
  };
  validateConfig(result);
  return result;
}
