/**
 * The single disturbance boundary used by the simulator.
 *
 * Everything here is pure data-in/data-out: the caller receives a corrupted
 * Measurement for the controller, a wind vector for the existing Environment,
 * and a *true* config for the plant. The nominal config and nominal controller
 * params are never returned, so the controller and mixer cannot see the
 * disturbed mass, COM, inertia or voltage ceiling.
 *
 * Step order (fixed and documented): wind OU, battery (from the previous
 * step's rotor speeds), discrete payload event, sensor noise. Wind and motor
 * commands are zero-order held across the RK4 stages of the core step.
 */

import type { Measurement } from "../control/types";
import type { MultirotorConfig } from "../multirotor/config";
import {
  STILL_AIR,
  type Environment,
} from "../multirotor/dynamics";
import {
  applyBatteryVoltage,
  batteryStep,
  initialBatteryState,
  validateBatteryParams,
} from "./battery";
import { noiseStep, initialSensorState, validateSensorParams } from "./noise";
import { applyPayload } from "./payload";
import { deriveDisturbanceSeeds } from "./seeds";
import type {
  DisturbanceDiagnostics,
  DisturbanceParams,
  DisturbanceState,
  DisturbanceStepResult,
} from "./types";
import {
  initialWindState,
  validateWindParams,
  windStep,
} from "./wind";

export function validateDisturbanceParams(params: DisturbanceParams): void {
  if (!Number.isInteger(params.seed) || params.seed < 0 || params.seed > 0xffffffff) {
    throw new RangeError("seed must be a 32-bit unsigned integer");
  }
  validateWindParams(params.wind);
  validateSensorParams(params.sensors);
  validateBatteryParams(params.battery);
  if (params.payload !== null) {
    if (!Number.isFinite(params.payload.atTime) || params.payload.atTime < 0) {
      throw new RangeError("payload.atTime must be a finite non-negative number");
    }
  }
}

/** True only when at least one model can affect the flight. */
export function isDisturbanceActive(params: DisturbanceParams): boolean {
  return (
    params.wind.enabled ||
    params.sensors.enabled ||
    params.battery.enabled ||
    params.payload !== null
  );
}

/** Fresh disturbance memory; wind and sensors use distinct seeded streams. */
export function createDisturbanceState(
  params: DisturbanceParams,
): DisturbanceState {
  validateDisturbanceParams(params);
  const seeds = deriveDisturbanceSeeds(params.seed);
  return {
    wind: initialWindState(seeds.wind),
    sensors: initialSensorState(seeds.sensors),
    battery: initialBatteryState(params.battery),
    payloadApplied: false,
    time: 0,
  };
}

/**
 * Advance all disturbances by one core step. `truth` is the uncorrupted
 * measurement; `motorSpeeds` are the speeds from the previous step, so the
 * battery model has an explicit one-step lag.
 */
export function disturbanceStep(
  state: DisturbanceState,
  params: DisturbanceParams,
  truth: Measurement,
  motorSpeeds: readonly number[],
  nominalConfig: MultirotorConfig,
  dt: number,
): DisturbanceStepResult {
  validateDisturbanceParams(params);

  const wind = windStep(state.wind, params.wind, dt);
  const battery = batteryStep(
    state.battery,
    params.battery,
    motorSpeeds,
    nominalConfig.motors.map((motor) => motor.kQ),
    dt,
  );

  let trueConfig = nominalConfig;
  const event = params.payload;
  const payloadApplied =
    state.payloadApplied ||
    (event !== null && state.time >= event.atTime);
  // The event fires once, but the resulting config must be rebuilt on every
  // subsequent step until the flight is reset.
  if (event !== null && payloadApplied) {
    trueConfig = applyPayload(trueConfig, event.payload);
  }
  if (params.battery.enabled) {
    trueConfig = applyBatteryVoltage(trueConfig, battery.scale);
  }

  const sensors = noiseStep(state.sensors, params.sensors, truth, dt);
  const diagnostics: DisturbanceDiagnostics = {
    wind: wind.wind,
    voltage: battery.state.voltage,
    stateOfCharge: battery.state.stateOfCharge,
    openCircuitVoltage: battery.state.openCircuitVoltage,
    current: battery.state.current,
    power: battery.state.power,
    maxOmegaScale: battery.scale,
    payloadApplied,
    trueMass: trueConfig.mass,
  };
  const environment: Environment = params.wind.enabled
    ? { windVelocity: wind.wind }
    : STILL_AIR;

  return {
    state: {
      wind: wind.state,
      sensors: sensors.state,
      battery: battery.state,
      payloadApplied,
      time: state.time + dt,
    },
    measurement: sensors.measurement,
    trueConfig,
    environment,
    diagnostics,
  };
}
