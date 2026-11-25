/**
 * Placeholder battery-sag model. This is not a chemistry or electrical model.
 *
 * Open-circuit voltage falls linearly with state of charge:
 *   V_oc(SoC) = V_nom * (f_min + (1 - f_min) * SoC)
 *
 * Rotor mechanical power is the sum of kQ*omega^3, using the config's own
 * motor constants. Current is inferred as I = P / (eta*V) and the terminal
 * voltage solves the loaded-pack equation exactly (no iteration):
 *   V = V_oc - I*R  =>  V^2 - V_oc*V + P*R/eta = 0
 *   V = (V_oc + sqrt(max(0, V_oc^2 - 4*P*R/eta))) / 2
 *
 * Only the *plant* config is affected: every motor's maxOmega is scaled by
 * V/V_nom. The controller keeps its nominal params and the mixer keeps the
 * nominal allocation, so saturation grows unnoticed as the pack depletes.
 * SoC drains as dSoC/dt = -I / (capacityAh * 3600).
 */

import { validateConfig, type MultirotorConfig } from "../multirotor/config";
import type { BatteryParams, BatteryState, BatteryStepResult } from "./types";

/** Plausible 6S-style placeholder values (not measured hardware data). */
export const REALISTIC_PACK = {
  nominalVoltage: 22.2,
  minVoltageFraction: 0.85,
  internalResistance: 0.15,
  efficiency: 0.75,
  capacityAh: 2.2,
} as const;

/**
 * Deliberately non-physical wide voltage curve. Used only for saturation
 * tests, the stress-pack demo row and the viewer's stress toggle.
 */
export const STRESS_PACK = {
  nominalVoltage: 22.2,
  minVoltageFraction: 0.45,
  internalResistance: 0.15,
  efficiency: 0.75,
  capacityAh: 2.2,
} as const;

export type BatteryPack = typeof REALISTIC_PACK | typeof STRESS_PACK;

export function batteryParams(
  pack: BatteryPack = REALISTIC_PACK,
  initialStateOfCharge = 1,
  enabled = false,
): BatteryParams {
  return { ...pack, initialStateOfCharge, enabled };
}

/** Disabled by default so a zero-disturbance flight is unchanged. */
export const DEFAULT_BATTERY_PARAMS: BatteryParams = batteryParams();

function assertFinite(name: string, value: number): void {
  if (!Number.isFinite(value)) {
    throw new RangeError(`${name} must be a finite number`);
  }
}

export function validateBatteryParams(params: BatteryParams): void {
  if (typeof params.enabled !== "boolean") {
    throw new RangeError("battery.enabled must be a boolean");
  }
  for (const name of [
    "initialStateOfCharge",
    "nominalVoltage",
    "minVoltageFraction",
    "internalResistance",
    "efficiency",
    "capacityAh",
  ] as const) {
    assertFinite(`battery.${name}`, params[name]);
  }
  const soc = params.initialStateOfCharge;
  if (soc < 0 || soc > 1) {
    throw new RangeError("battery.initialStateOfCharge must be within [0, 1]");
  }
  if (params.nominalVoltage <= 0) {
    throw new RangeError("battery.nominalVoltage must be positive");
  }
  if (params.minVoltageFraction <= 0 || params.minVoltageFraction > 1) {
    throw new RangeError("battery.minVoltageFraction must be within (0, 1]");
  }
  if (params.internalResistance < 0) {
    throw new RangeError("battery.internalResistance must be non-negative");
  }
  if (params.efficiency <= 0 || params.efficiency > 1) {
    throw new RangeError("battery.efficiency must be within (0, 1]");
  }
  if (params.capacityAh <= 0) {
    throw new RangeError("battery.capacityAh must be positive");
  }
}

/** Open-circuit voltage at a given state of charge. */
export function openCircuitVoltage(soc: number, params: BatteryParams): number {
  return params.nominalVoltage *
    (params.minVoltageFraction + (1 - params.minVoltageFraction) * soc);
}

/** Fresh pack at its initial SoC with no load current yet. */
export function initialBatteryState(params: BatteryParams): BatteryState {
  validateBatteryParams(params);
  const voltage = openCircuitVoltage(params.initialStateOfCharge, params);
  return {
    stateOfCharge: params.initialStateOfCharge,
    voltage,
    openCircuitVoltage: voltage,
    current: 0,
    power: 0,
  };
}

/**
 * Advance the pack by one step using the previous step's rotor speeds and the
 * config's kQ constants. Disabled battery returns the same state and scale 1.
 */
export function batteryStep(
  state: BatteryState,
  params: BatteryParams,
  motorSpeeds: readonly number[],
  motorKq: readonly number[],
  dt: number,
): BatteryStepResult {
  if (motorSpeeds.length !== motorKq.length) {
    throw new RangeError("motorSpeeds and motorKq must have equal length");
  }
  validateBatteryParams(params);
  assertFinite("dt", dt);
  if (dt <= 0) throw new RangeError("dt must be positive");
  if (!params.enabled) return { state, scale: 1 };

  const openCircuit = openCircuitVoltage(state.stateOfCharge, params);
  let power = 0;
  for (let index = 0; index < motorSpeeds.length; index++) {
    const omega = motorSpeeds[index]!;
    assertFinite(`motorSpeeds[${index}]`, omega);
    power += motorKq[index]! * omega * omega * omega;
  }
  const discriminant = Math.max(
    0,
    openCircuit * openCircuit - 4 * power * params.internalResistance / params.efficiency,
  );
  const voltage = (openCircuit + Math.sqrt(discriminant)) / 2;
  const current = power / (params.efficiency * voltage);
  const stateOfCharge = Math.max(
    0,
    state.stateOfCharge - current * dt / (params.capacityAh * 3600),
  );
  return {
    state: {
      stateOfCharge,
      voltage,
      openCircuitVoltage: openCircuit,
      current,
      power,
    },
    scale: Math.min(1, Math.max(0, voltage / params.nominalVoltage)),
  };
}

/** Scale each motor's speed ceiling by the loaded voltage ratio. */
export function applyBatteryVoltage(
  config: MultirotorConfig,
  scale: number,
): MultirotorConfig {
  assertFinite("scale", scale);
  if (scale === 1) return config;
  const result: MultirotorConfig = {
    ...config,
    motors: config.motors.map((motor) => ({
      ...motor,
      minOmega: motor.minOmega * scale,
      maxOmega: motor.maxOmega * scale,
    })),
  };
  validateConfig(result);
  return result;
}
