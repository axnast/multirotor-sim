/**
 * Shared plain-data types for deterministic disturbances (Milestone 5).
 *
 * Nothing here is DOM-aware and nothing is hidden in a class: every random
 * stream, filter memory and process state is explicit and serialisable. The
 * same seed and the same sequence of inputs always produce the same bytes.
 *
 * This file is added incrementally: commits 1 and 2 define the PRNG and
 * sensor types; battery, payload, scenarios and the combined disturbance
 * boundary are added in later commits.
 */

import type { Measurement } from "../control/types";
import type { Vec3 } from "../math/vec3";
import type { Environment } from "../multirotor/dynamics";
import type { MultirotorConfig } from "../multirotor/config";

/**
 * xoshiro128** state plus one cached Box–Muller spare.
 *
 * `s` is four unsigned 32-bit words, in the order used by the next-state
 * function. `hasSpare` is true exactly when `spare` holds an unused Gaussian.
 */
export interface PrngState {
  readonly s: readonly [number, number, number, number];
  readonly spare: number;
  readonly hasSpare: boolean;
}

/** Deterministic 1-cosine gust, zero at both ends of its interval. */
export interface CosineGust {
  /** Start time in simulator seconds. */
  readonly startTime: number;
  /** Duration in seconds; must be positive. */
  readonly duration: number;
  /** Peak amplitude in m/s (reached at the midpoint). */
  readonly amplitude: number;
  /** World-frame unit direction. */
  readonly direction: Vec3;
}

/**
 * World-frame wind: a constant mean, a per-axis Ornstein–Uhlenbeck gust and
 * an optional discrete 1-cosine event.
 */
export interface WindParams {
  readonly enabled: boolean;
  /** Steady mean velocity in the world frame (m/s). */
  readonly mean: Vec3;
  /** Stationary standard deviation of the OU process per axis (m/s). */
  readonly gustSigma: Vec3;
  /** OU correlation time in seconds; `a = exp(-dt/Tc)`. */
  readonly correlationTime: number;
  /** null when no discrete gust is scheduled. */
  readonly cosineGust: CosineGust | null;
}

/** Wind process memory: current OU state, PRNG state and process time. */
export interface WindState {
  readonly prng: PrngState;
  /** Current OU gust vector in the world frame (m/s). */
  readonly gust: Vec3;
  /** Start time of the next step, in seconds. */
  readonly time: number;
}

export interface WindStepResult {
  readonly state: WindState;
  /** Total wind velocity to pass to `Environment.windVelocity` (m/s). */
  readonly wind: Vec3;
}

/**
 * Placeholder sensor error model for the measurement given to the controller.
 * All sigmas are 1-standard-deviation values; sample-and-hold sensor rates are
 * out of scope. These are illustrative numbers, not measured hardware specs.
 */
export interface SensorParams {
  readonly enabled: boolean;
  /** GPS-RTK-like position sigma (m). */
  readonly positionSigma: number;
  /** Velocity sigma (m/s). */
  readonly velocitySigma: number;
  /** Per-axis small-rotation sigma (rad). */
  readonly attitudeSigmaRad: number;
  /** Gyro white noise sigma (rad/s). */
  readonly gyroWhiteSigma: number;
  /** Gyro bias random-walk coefficient (rad/s per sqrt(s)). */
  readonly gyroBiasWalkSigma: number;
}

/** Sensor noise memory: the PRNG stream and the integrated gyro bias. */
export interface SensorState {
  readonly prng: PrngState;
  /** Body-frame gyro bias (rad/s), a random walk. */
  readonly gyroBias: Vec3;
}

export interface NoiseStepResult {
  readonly state: SensorState;
  /** Corrupted copy of the truth; the truth itself is never changed. */
  readonly measurement: Measurement;
}

/**
 * Placeholder pack model. `minVoltageFraction` 0.85 is a plausible 6S pack;
 * the named stress pack uses 0.45 and is explicitly non-physical, used only
 * to make saturation observable in short runs.
 */
export interface BatteryParams {
  readonly enabled: boolean;
  readonly initialStateOfCharge: number;
  readonly nominalVoltage: number;
  readonly minVoltageFraction: number;
  readonly internalResistance: number;
  /** Electrical-to-mechanical efficiency used to infer current from power. */
  readonly efficiency: number;
  readonly capacityAh: number;
}

export interface BatteryState {
  readonly stateOfCharge: number;
  /** Loaded terminal voltage (V). */
  readonly voltage: number;
  /** Open-circuit voltage from state of charge (V). */
  readonly openCircuitVoltage: number;
  /** Draw from the pack (A), inferred from mechanical rotor power. */
  readonly current: number;
  /** Total mechanical rotor power (W). */
  readonly power: number;
}

export interface BatteryStepResult {
  readonly state: BatteryState;
  /** `maxOmega_eff / maxOmega_nominal` for the upcoming step. */
  readonly scale: number;
}

/** A point payload at a body-frame position in the original airframe frame. */
export interface PayloadSpec {
  readonly mass: number;
  readonly position: Vec3;
}

/** Discrete payload event: nominal until the step starting at `atTime`. */
export interface PayloadEvent {
  readonly atTime: number;
  readonly payload: PayloadSpec;
}

/** All disturbance parameters are plain data, suitable for M6 benchmarks. */
export interface DisturbanceParams {
  readonly seed: number;
  readonly wind: WindParams;
  readonly sensors: SensorParams;
  readonly battery: BatteryParams;
  readonly payload: PayloadEvent | null;
}

/** Disturbance diagnostics exposed to the HUD, CSV and M6 metrics. */
export interface DisturbanceDiagnostics {
  readonly wind: Vec3;
  readonly voltage: number;
  readonly stateOfCharge: number;
  readonly openCircuitVoltage: number;
  readonly current: number;
  readonly power: number;
  readonly maxOmegaScale: number;
  readonly payloadApplied: boolean;
  readonly trueMass: number;
}

/** All disturbance memory, including both independent PRNG streams. */
export interface DisturbanceState {
  readonly wind: WindState;
  readonly sensors: SensorState;
  readonly battery: BatteryState;
  readonly payloadApplied: boolean;
  /** Start time of the next core step (s). */
  readonly time: number;
}

export interface DisturbanceStepResult {
  readonly state: DisturbanceState;
  /** Corrupted controller measurement. */
  readonly measurement: Measurement;
  /** Disturbed config used by the plant only; never by controller or mixer. */
  readonly trueConfig: MultirotorConfig;
  /** Wind passed through the existing Environment.windVelocity. */
  readonly environment: Environment;
  readonly diagnostics: DisturbanceDiagnostics;
}
