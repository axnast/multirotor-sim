import { describe, expect, it } from "vitest";

import {
  defaultControllerParams,
  initialControllerState,
} from "../src/core/control/controller";
import { controllerStepDetailed } from "../src/core/control/controller";
import {
  applyBatteryVoltage,
  batteryParams,
  batteryStep,
  initialBatteryState,
  REALISTIC_PACK,
  STRESS_PACK,
} from "../src/core/disturbance/battery";
import { disturbanceParamsFor } from "../src/core/disturbance/scenario";
import type { BatteryParams } from "../src/core/disturbance/types";
import {
  hoverMotorSpeeds,
  makeHex,
  maxThrust,
} from "../src/core/multirotor/config";
import { SIM_DT, TELEMETRY_CAPACITY } from "../src/viewer/sim/constants";
import { MISSIONS } from "../src/viewer/sim/mission";
import {
  createSimulation,
  stepOnce,
  type SimulationPlant,
} from "../src/viewer/sim/runner";

function plant(): SimulationPlant {
  const config = makeHex({ groundPlane: true });
  return { config, params: defaultControllerParams(config) };
}

function hoverMargin(pack: BatteryParams): {
  readonly scale: number;
  readonly margin: number;
  readonly voltage: number;
} {
  const p = plant();
  const speeds = hoverMotorSpeeds(p.config);
  const step = batteryStep(
    initialBatteryState(pack),
    pack,
    speeds,
    p.config.motors.map((motor) => motor.kQ),
    SIM_DT,
  );
  const trueConfig = applyBatteryVoltage(p.config, step.scale);
  const weight = p.config.mass * 9.81;
  return {
    scale: step.scale,
    margin: maxThrust(trueConfig) / weight,
    voltage: step.state.voltage,
  };
}

describe("battery sag", () => {
  it("keeps hover margins above 1 with the realistic pack at any SoC", () => {
    const full = hoverMargin(batteryParams(REALISTIC_PACK, 1, true));
    const empty = hoverMargin(batteryParams(REALISTIC_PACK, 0, true));
    expect(full.scale).toBeCloseTo(0.953, 3);
    expect(full.margin).toBeCloseTo(3.707, 3);
    expect(empty.scale).toBeCloseTo(0.794, 3);
    expect(empty.margin).toBeCloseTo(2.572, 3);
    expect(empty.margin).toBeGreaterThan(2.5);

    // Analytic check: max thrust with scaled ceilings divided by weight.
    const p = plant();
    const expected = (p.config.motors.length * p.config.motors[0]!.kT *
      Math.pow(p.config.motors[0]!.maxOmega * empty.scale, 2)) /
      (p.config.mass * 9.81);
    expect(empty.margin).toBeCloseTo(expected, 12);
  });

  it("drops hover thrust below weight with the stress pack at low SoC", () => {
    const low = hoverMargin(batteryParams(STRESS_PACK, 0.15, true));
    expect(low.scale).toBeCloseTo(0.429, 3);
    expect(low.margin).toBeCloseTo(0.751, 3);
    expect(low.margin).toBeLessThan(1);
    const p = plant();
    const expected = (p.config.motors.length * p.config.motors[0]!.kT *
      Math.pow(p.config.motors[0]!.maxOmega * low.scale, 2)) /
      (p.config.mass * 9.81);
    expect(low.margin).toBeCloseTo(expected, 12);
  });

  it("scales only the plant's speed ceilings, not kT or mass", () => {
    const config = plant().config;
    const scaled = applyBatteryVoltage(config, 0.5);
    expect(scaled).not.toBe(config);
    expect(scaled.mass).toBe(config.mass);
    expect(scaled.drag).toBe(config.drag);
    expect(scaled.motors[0]!.kT).toBe(config.motors[0]!.kT);
    expect(scaled.motors[0]!.maxOmega).toBeCloseTo(500, 12);
    expect(scaled.motors[0]!.minOmega).toBe(0);
    expect(applyBatteryVoltage(config, 1)).toBe(config);
  });

  it("drains SoC exactly as the integrated current predicts over 60 s", () => {
    const p = plant();
    const params = disturbanceParamsFor("calm", 3, {
      pack: REALISTIC_PACK,
      battery: true,
      stateOfCharge: 1,
    });
    let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
    let chargeUsed = 0;
    for (let step = 0; step < 60_000; step++) {
      state = stepOnce(state, p);
      chargeUsed += (state.diagnostics?.current ?? 0) * SIM_DT;
    }
    const expectedSoc = 1 - chargeUsed / (REALISTIC_PACK.capacityAh * 3600);
    expect(state.diagnostics!.stateOfCharge).toBeCloseTo(expectedSoc, 12);
    expect(state.diagnostics!.stateOfCharge).toBeLessThan(0.97);
    expect(state.diagnostics!.voltage).toBeLessThan(REALISTIC_PACK.nominalVoltage);
    // Reported by the test run: chargeUsed and SoC are the M6 baseline.
    expect(chargeUsed).toBeGreaterThan(0);
  }, 60_000);

  it("saturates the plant but not the nominal mixer at low SoC", () => {
    const p = plant();
    const params = disturbanceParamsFor("calm", 4, {
      pack: STRESS_PACK,
      battery: true,
      stateOfCharge: 0.15,
    });
    let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
    for (let step = 0; step < 2_000; step++) state = stepOnce(state, p);

    expect(state.diagnostics!.maxOmegaScale).toBeLessThan(0.5);
    expect(state.stats.plantSaturationSteps).toBeGreaterThan(0);
    expect(state.lastMix!.saturated).toBe(false);
    expect(state.multirotor.body.position.every(Number.isFinite)).toBe(true);
    expect(state.multirotor.body.position[2]).toBeLessThan(5);
    for (const [index, omega] of state.multirotor.motorSpeeds.entries()) {
      expect(omega).toBeLessThanOrEqual(
        state.trueConfig.motors[index]!.maxOmega + 1e-9,
      );
    }
  }, 30_000);

  it("leaves the controller wrench unchanged when the pack sags", () => {
    const p = plant();
    const params = disturbanceParamsFor("calm", 5, {
      pack: STRESS_PACK,
      battery: true,
      stateOfCharge: 0.15,
    });
    const initial = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
    const nominalCommand = controllerStepDetailed(
      initial.controller,
      {
        position: initial.multirotor.body.position,
        velocity: initial.multirotor.body.velocity,
        orientation: initial.multirotor.body.orientation,
        angularVelocity: initial.multirotor.body.angularVelocity,
      },
      initial.setpoint,
      p.params,
      SIM_DT,
    );
    const next = stepOnce(initial, p);
    expect(next.last!.wrench).toEqual(nominalCommand.wrench);
    expect(initialControllerState()).toEqual(initial.controller);
  });
});
