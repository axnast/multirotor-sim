import { describe, expect, it } from "vitest";

import {
  controllerStepDetailed,
  defaultControllerParams,
} from "../src/core/control/controller";
import { STRESS_PACK } from "../src/core/disturbance/battery";
import { PAYLOAD_2CM } from "../src/core/disturbance/payload";
import { disturbanceParamsFor } from "../src/core/disturbance/scenario";
import { makeHex } from "../src/core/multirotor/config";
import { mixDetailed } from "../src/core/multirotor/mixer";
import { SIM_DT, TELEMETRY_CAPACITY } from "../src/viewer/sim/constants";
import { MISSIONS } from "../src/viewer/sim/mission";
import {
  createSimulation,
  measurementOf,
  stepOnce,
} from "../src/viewer/sim/runner";

/**
 * The nominal/true split is the whole point of M5. This test fails if the
 * controller or mixer is ever wired to the disturbed config: the wrench and
 * mixed commands must match explicit calls with the nominal config, and must
 * differ from the same calls with the true config.
 */
describe("controller and mixer never see the true config", () => {
  it("uses nominal mass/config for control and allocation, true config for the plant", () => {
    const config = makeHex({ groundPlane: true });
    const plant = { config, params: defaultControllerParams(config) };
    const params = disturbanceParamsFor("calm", 9, {
      pack: STRESS_PACK,
      battery: true,
      stateOfCharge: 0.15,
      payload: { atTime: 0, payload: PAYLOAD_2CM },
    });
    const initial = createSimulation(
      plant, MISSIONS.hover, TELEMETRY_CAPACITY, params,
    );
    const truth = measurementOf(initial.multirotor);
    const nominalCommand = controllerStepDetailed(
      initial.controller, truth, initial.setpoint, plant.params, SIM_DT,
    );
    const next = stepOnce(initial, plant);

    expect(next.trueConfig).not.toBe(config);
    expect(next.trueConfig.mass).toBeCloseTo(1.8, 12);
    expect(next.trueConfig.motors[0]!.maxOmega).toBeLessThan(
      config.motors[0]!.maxOmega,
    );

    // Controller: exactly the nominal result, and not the true-mass result.
    expect(next.last!.wrench).toEqual(nominalCommand.wrench);
    const trueParams = defaultControllerParams(next.trueConfig);
    expect(trueParams.mass).toBeCloseTo(1.8, 12);
    expect(trueParams.position.maxThrust).not.toBe(
      plant.params.position.maxThrust,
    );
    const wrongCommand = controllerStepDetailed(
      initial.controller, truth, initial.setpoint, trueParams, SIM_DT,
    );
    expect(wrongCommand.wrench).not.toEqual(nominalCommand.wrench);

    // Mixer: exactly the nominal allocation, not the voltage-limited one.
    expect(next.lastMix).toEqual(mixDetailed(config, nominalCommand.wrench));
    expect(next.lastMix).not.toEqual(
      mixDetailed(next.trueConfig, nominalCommand.wrench),
    );

    // Yet the plant is genuinely limited by the true config.
    expect(next.stats.plantSaturationSteps).toBe(1);
    expect(next.multirotor.motorSpeeds[0]!).toBeLessThanOrEqual(
      next.trueConfig.motors[0]!.maxOmega + 1e-9,
    );
  });
});
