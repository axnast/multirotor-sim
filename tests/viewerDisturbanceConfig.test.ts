import { describe, expect, it } from "vitest";

import { defaultControllerParams } from "../src/core/control/controller";
import { PAYLOAD_LARGE_STRESS } from "../src/core/disturbance/payload";
import { isDisturbanceActive } from "../src/core/disturbance/step";
import { makeHex } from "../src/core/multirotor/config";
import {
  buildDisturbanceParams,
  DEFAULT_VIEWER_DISTURBANCE,
  viewerPack,
} from "../src/viewer/sim/disturbanceConfig";
import { MISSIONS } from "../src/viewer/sim/mission";
import { ringToArray } from "../src/viewer/sim/ringBuffer";
import {
  createSimulation,
  installPayload,
  stepOnce,
  type SimulationPlant,
} from "../src/viewer/sim/runner";

function plant(): SimulationPlant {
  const config = makeHex({ groundPlane: true });
  return { config, params: defaultControllerParams(config) };
}

describe("viewer disturbance settings", () => {
  it("maps the all-off defaults to an inactive disturbance", () => {
    const params = buildDisturbanceParams(DEFAULT_VIEWER_DISTURBANCE);
    expect(isDisturbanceActive(params)).toBe(false);
    expect(params.wind.enabled).toBe(false);
    expect(params.sensors.enabled).toBe(false);
    expect(params.battery.enabled).toBe(false);
    expect(params.payload).toBe(null);

    const p = plant();
    const state = createSimulation(
      p, MISSIONS.hover, undefined, buildDisturbanceParams(DEFAULT_VIEWER_DISTURBANCE),
    );
    expect(state.disturbance).toBe(null);
    expect(state.diagnostics).toBe(null);
    expect(ringToArray(state.telemetry)[0]!.measuredPosition).toBe(null);
  });

  it("maps scenario, seed, sensors, pack and SoC to plain params", () => {
    const params = buildDisturbanceParams({
      ...DEFAULT_VIEWER_DISTURBANCE,
      scenario: "gusty",
      seed: 7,
      sensors: true,
      battery: true,
      pack: "stress",
      stateOfCharge: 0.15,
    });
    expect(params.seed).toBe(7);
    expect(params.wind.enabled).toBe(true);
    expect(params.wind.mean[0]).toBe(5);
    expect(params.wind.gustSigma[0]).toBe(3);
    expect(params.sensors.enabled).toBe(true);
    expect(params.battery.enabled).toBe(true);
    expect(params.battery.initialStateOfCharge).toBe(0.15);
    expect(params.battery.minVoltageFraction).toBe(0.45);
    expect(viewerPack("realistic").minVoltageFraction).toBe(0.85);
    expect(isDisturbanceActive(params)).toBe(true);
  });

  it("rejects an invalid seed", () => {
    expect(() =>
      buildDisturbanceParams({ ...DEFAULT_VIEWER_DISTURBANCE, seed: -1 }),
    ).toThrow(RangeError);
  });

  it("applies a payload dropped mid-flight on the very next step", () => {
    const p = plant();
    let state = createSimulation(p, MISSIONS.hover);
    for (let step = 0; step < 5_000; step++) state = stepOnce(state, p);
    expect(state.simTime).toBeCloseTo(5, 6);

    state = installPayload(
      state,
      p,
      buildDisturbanceParams({
        ...DEFAULT_VIEWER_DISTURBANCE,
        payload: PAYLOAD_LARGE_STRESS,
      }),
    );
    expect(state.disturbanceParams?.payload?.atTime).toBe(0);
    expect(state.diagnostics).toBe(null);
    state = stepOnce(state, p);
    expect(state.diagnostics?.payloadApplied).toBe(true);
    expect(state.trueConfig.mass).toBeCloseTo(
      p.config.mass + PAYLOAD_LARGE_STRESS.mass, 9,
    );
  });

  it("removes a payload and restores the nominal plant", () => {
    const p = plant();
    let state = createSimulation(
      p, MISSIONS.hover, undefined,
      buildDisturbanceParams({
        ...DEFAULT_VIEWER_DISTURBANCE,
        payload: PAYLOAD_LARGE_STRESS,
      }),
    );
    state = stepOnce(state, p);
    expect(state.trueConfig.mass).toBeGreaterThan(p.config.mass);

    state = installPayload(
      state, p, buildDisturbanceParams(DEFAULT_VIEWER_DISTURBANCE),
    );
    expect(state.disturbance).toBe(null);
    state = stepOnce(state, p);
    expect(state.trueConfig.mass).toBeCloseTo(p.config.mass, 9);
  });

  it("records measured position in telemetry only with sensor noise on", () => {
    const p = plant();
    let noisy = createSimulation(
      p, MISSIONS.hover, undefined,
      buildDisturbanceParams({ ...DEFAULT_VIEWER_DISTURBANCE, sensors: true }),
    );
    for (let step = 0; step < 40; step++) noisy = stepOnce(noisy, p);
    const sample = ringToArray(noisy.telemetry).at(-1)!;
    expect(sample.measuredPosition).not.toBe(null);
    expect(sample.measuredPosition).not.toEqual(sample.position);
  });
});
