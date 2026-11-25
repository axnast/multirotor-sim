/**
 * Sensor noise on the Measurement passed to the controller.
 *
 * The truth state is never touched: position and velocity get additive
 * Gaussian errors, attitude gets a small body-frame rotation, and gyro gets
 * white noise plus a bias random walk. The corrupted measurement is a fresh
 * object with fresh arrays, so the controller cannot accidentally alias the
 * truth. Sample-and-hold sensor rates are out of scope (M5 runs at 1 kHz).
 *
 * Draw order per step, fixed for reproducibility: gyro bias (3), gyro white
 * (3), attitude (3), position (3), velocity (3).
 */

import type { Measurement } from "../control/types";
import {
  quatFromAxisAngle,
  quatMultiply,
  type Quat,
} from "../math/quat";
import { vec3Length, type Vec3 } from "../math/vec3";
import { createPrng, nextGaussian3 } from "./prng";
import type { NoiseStepResult, SensorParams, SensorState } from "./types";

/** Illustrative placeholders, not measured sensor specifications. */
export const DEFAULT_SENSOR_PARAMS: SensorParams = {
  enabled: false,
  positionSigma: 0.02,
  velocitySigma: 0.05,
  attitudeSigmaRad: (0.2 * Math.PI) / 180,
  gyroWhiteSigma: 0.005,
  gyroBiasWalkSigma: 2e-5,
};

function assertFinite(name: string, value: number): void {
  if (!Number.isFinite(value)) {
    throw new RangeError(`${name} must be a finite number`);
  }
}

/** Fresh sensor stream with zero gyro bias. */
export function initialSensorState(seed: number): SensorState {
  return { prng: createPrng(seed), gyroBias: [0, 0, 0] };
}

export function validateSensorParams(params: SensorParams): void {
  if (typeof params.enabled !== "boolean") {
    throw new RangeError("sensor.enabled must be a boolean");
  }
  for (const name of [
    "positionSigma",
    "velocitySigma",
    "attitudeSigmaRad",
    "gyroWhiteSigma",
    "gyroBiasWalkSigma",
  ] as const) {
    const value = params[name];
    assertFinite(`sensor.${name}`, value);
    if (value < 0) {
      throw new RangeError(`sensor.${name} must be non-negative`);
    }
  }
}

/**
 * Advance only the gyro bias random walk:
 * `bias' = bias + sigma_walk * sqrt(dt) * N(0,1)^3`.
 * Exported so the variance linearity can be tested on an ensemble of
 * independent paths without paying for the other noise channels.
 */
export function gyroBiasStep(
  state: SensorState,
  params: SensorParams,
  dt: number,
): SensorState {
  validateSensorParams(params);
  assertFinite("dt", dt);
  if (dt <= 0) throw new RangeError("dt must be positive");
  if (!params.enabled || params.gyroBiasWalkSigma === 0) return state;
  const draw = nextGaussian3(state.prng);
  const coefficient = params.gyroBiasWalkSigma * Math.sqrt(dt);
  return {
    prng: draw.state,
    gyroBias: [
      state.gyroBias[0]! + coefficient * draw.values[0],
      state.gyroBias[1]! + coefficient * draw.values[1],
      state.gyroBias[2]! + coefficient * draw.values[2],
    ],
  };
}

/** Add a small body-frame rotation to the truth orientation. */
function perturbOrientation(
  truth: Quat,
  rotation: Vec3,
  sigma: number,
): Quat {
  if (sigma === 0) return truth;
  const scaled: Vec3 = [
    sigma * rotation[0],
    sigma * rotation[1],
    sigma * rotation[2],
  ];
  const angle = vec3Length(scaled);
  // quatFromAxisAngle normalises its axis; it returns identity when angle=0.
  const noise = quatFromAxisAngle(scaled, angle);
  return quatMultiply(truth, noise);
}

/**
 * Produce one corrupted measurement. Disabled noise returns the *same*
 * measurement object and state, with no PRNG consumption.
 */
export function noiseStep(
  state: SensorState,
  params: SensorParams,
  truth: Measurement,
  dt: number,
): NoiseStepResult {
  validateSensorParams(params);
  assertFinite("dt", dt);
  if (dt <= 0) throw new RangeError("dt must be positive");
  if (!params.enabled) return { state, measurement: truth };

  const withBias = gyroBiasStep(state, params, dt);
  const gyroWhite = nextGaussian3(withBias.prng);
  const attitude = nextGaussian3(gyroWhite.state);
  const position = nextGaussian3(attitude.state);
  const velocity = nextGaussian3(position.state);

  return {
    state: { prng: velocity.state, gyroBias: withBias.gyroBias },
    measurement: {
      position: [
        truth.position[0]! + params.positionSigma * position.values[0],
        truth.position[1]! + params.positionSigma * position.values[1],
        truth.position[2]! + params.positionSigma * position.values[2],
      ],
      velocity: [
        truth.velocity[0]! + params.velocitySigma * velocity.values[0],
        truth.velocity[1]! + params.velocitySigma * velocity.values[1],
        truth.velocity[2]! + params.velocitySigma * velocity.values[2],
      ],
      orientation: perturbOrientation(
        truth.orientation,
        attitude.values,
        params.attitudeSigmaRad,
      ),
      angularVelocity: [
        truth.angularVelocity[0]! + withBias.gyroBias[0]! +
          params.gyroWhiteSigma * gyroWhite.values[0],
        truth.angularVelocity[1]! + withBias.gyroBias[1]! +
          params.gyroWhiteSigma * gyroWhite.values[1],
        truth.angularVelocity[2]! + withBias.gyroBias[2]! +
          params.gyroWhiteSigma * gyroWhite.values[2],
      ],
    },
  };
}
