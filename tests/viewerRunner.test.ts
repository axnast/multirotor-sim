import { describe, expect, it } from "vitest";

import {
  controllerStepDetailed,
  defaultControllerParams,
  initialControllerState,
} from "../src/core/control/controller";
import type { Setpoint } from "../src/core/control/types";
import { makeHex } from "../src/core/multirotor/config";
import {
  stepMultirotorDetailed,
  type MultirotorState,
} from "../src/core/multirotor/dynamics";
import { mixDetailed } from "../src/core/multirotor/mixer";
import { SIM_DT } from "../src/viewer/sim/constants";
import {
  MISSIONS,
  missionInitialMultirotor,
  type Mission,
} from "../src/viewer/sim/mission";
import { ringToArray } from "../src/viewer/sim/ringBuffer";
import {
  advanceSimulation,
  createSimulation,
  measurementOf,
  resetSimulation,
  selectMission,
  setManualSetpoint,
  setPaused,
  setSpeed,
  stepOnce,
  type SimulationPlant,
} from "../src/viewer/sim/runner";

function makePlant(): SimulationPlant {
  const config = makeHex({ groundPlane: true });
  return { config, params: defaultControllerParams(config) };
}

describe("fixed-step simulation runner", () => {
  it("matches direct core calls bit-for-bit for 500 steps", () => {
    const plant = makePlant();
    const setpoint: Setpoint = { position: [0, 0, 5], yaw: 0 };
    let directState: MultirotorState = missionInitialMultirotor(
      plant.config,
      MISSIONS.hover,
    );
    let directController = initialControllerState();
    const wrenches: number[][] = [];
    for (let step = 0; step < 500; step++) {
      const command = controllerStepDetailed(
        directController,
        measurementOf(directState),
        setpoint,
        plant.params,
        SIM_DT,
      );
      directController = command.state;
      const mixed = mixDetailed(plant.config, command.wrench);
      directState = stepMultirotorDetailed(
        directState,
        plant.config,
        mixed.motorSpeeds,
        SIM_DT,
      ).state;
      wrenches.push([...command.wrench]);
    }

    let runner = createSimulation(plant, MISSIONS.hover);
    for (let step = 0; step < 500; step++) runner = stepOnce(runner, plant);

    expect(runner.multirotor).toEqual(directState);
    expect(runner.controller).toEqual(directController);
    expect([...runner.last!.wrench]).toEqual(wrenches[499]!);
    expect(runner.stepCount).toBe(500);
    expect(runner.simTime).toBeCloseTo(0.5, 12);
  });

  it("consumes whole 1 ms steps and keeps the fractional accumulator", () => {
    const plant = makePlant();
    let state = createSimulation(plant, MISSIONS.hover);

    let result = advanceSimulation(state, plant, 0.0004);
    expect(result.stepsRun).toBe(0);
    state = result.state;

    result = advanceSimulation(state, plant, 0.0004);
    expect(result.stepsRun).toBe(0);
    state = result.state;

    result = advanceSimulation(state, plant, 0.0004);
    expect(result.stepsRun).toBe(1);
    expect(result.state.accumulator).toBeCloseTo(0.0002, 12);
    expect(result.capped).toBe(false);
  });

  it("scales the accumulator by the speed multiplier", () => {
    const plant = makePlant();
    const state = setSpeed(createSimulation(plant, MISSIONS.hover), 2);
    expect(advanceSimulation(state, plant, 0.01).stepsRun).toBe(20);
    // A 1 s frame is clamped to 0.25 s, so at 2x only 0.5 s is consumed.
    expect(advanceSimulation(state, plant, 1, 1000).stepsRun).toBe(500);
    expect(setSpeed(state, 99).speed).toBe(4);
    expect(setSpeed(state, -99).speed).toBe(0.25);
  });

  it("caps a slow frame and drops the excess instead of carrying it", () => {
    const plant = makePlant();
    const result = advanceSimulation(
      createSimulation(plant, MISSIONS.hover),
      plant,
      5,
      5,
    );
    expect(result.stepsRun).toBe(5);
    expect(result.capped).toBe(true);
    expect(result.state.accumulator).toBeLessThan(SIM_DT);
    // 0.25 s is consumed (250 steps); 5 run and 245 are dropped.
    expect(result.state.droppedSteps).toBeGreaterThan(240);
    expect(result.state.cappedLastFrame).toBe(true);
  });

  it("freezes time while paused and steps exactly one millisecond", () => {
    const plant = makePlant();
    let state = setPaused(createSimulation(plant, MISSIONS.hover), true);

    const paused = advanceSimulation(state, plant, 10);
    expect(paused.stepsRun).toBe(0);
    expect(paused.state.simTime).toBe(0);
    expect(paused.state.accumulator).toBe(0);

    state = stepOnce(paused.state, plant);
    expect(state.stepCount).toBe(1);
    expect(state.simTime).toBeCloseTo(SIM_DT, 12);
    expect(state.paused).toBe(true);
  });

  it("resets to the mission start with fresh controller state", () => {
    const plant = makePlant();
    let state = createSimulation(plant, MISSIONS.hover);
    for (let step = 0; step < 100; step++) state = stepOnce(state, plant);

    const reset = resetSimulation(state, plant);
    expect(reset.stepCount).toBe(0);
    expect(reset.simTime).toBe(0);
    expect(reset.multirotor.body.position).toEqual(MISSIONS.hover.startPosition);
    expect(reset.controller).toEqual(initialControllerState());
    expect(reset.telemetry.length).toBe(1);
    expect(reset.trail.length).toBe(1);
    expect(reset.stats.saturationSteps).toBe(0);
  });

  it("advances square-style waypoints on integer step counts", () => {
    const shortMission: Mission = {
      ...MISSIONS.hover,
      waypoints: [
        { position: [0, 0, 5], yaw: 0, holdSteps: 5 },
        { position: [1, 0, 5], yaw: 0, holdSteps: null },
      ],
    };
    const plant = makePlant();
    let state = createSimulation(plant, shortMission);
    expect(state.setpoint.position[0]).toBe(0);
    for (let step = 0; step < 5; step++) state = stepOnce(state, plant);
    expect(state.mission!.waypointIndex).toBe(1);
    expect(state.setpoint.position[0]).toBe(1);
  });

  it("keeps manual setpoints and resets back to the home mission", () => {
    const plant = makePlant();
    const manual: Setpoint = { position: [2, 3, 4], yaw: 0.3 };
    let state = setManualSetpoint(createSimulation(plant, MISSIONS.hover), manual);
    expect(state.mission).toBeNull();
    state = advanceSimulation(state, plant, 0.01).state;
    expect(state.setpoint).toEqual(manual);

    const reset = resetSimulation(state, plant);
    expect(reset.mission).not.toBeNull();
    expect(reset.setpoint).toEqual({ position: [0, 0, 5], yaw: 0 });
    expect(selectMission(state, plant, MISSIONS.takeoff).paused).toBe(false);
  });

  it("is deterministic under different frame slicing", () => {
    const plant = makePlant();
    let stepped = createSimulation(plant, MISSIONS.hover);
    for (let step = 0; step < 500; step++) stepped = stepOnce(stepped, plant);

    let framed = createSimulation(plant, MISSIONS.hover);
    for (let frame = 0; frame < 50; frame++) {
      framed = advanceSimulation(framed, plant, 0.01).state;
    }

    expect(framed.stepCount).toBe(stepped.stepCount);
    expect(framed.multirotor).toEqual(stepped.multirotor);
    expect(framed.controller).toEqual(stepped.controller);
    expect(ringToArray(framed.telemetry)).toEqual(ringToArray(stepped.telemetry));
    expect(ringToArray(framed.trail)).toEqual(ringToArray(stepped.trail));
  });

  it("records telemetry and trail every 20 core steps", () => {
    const plant = makePlant();
    let state = createSimulation(plant, MISSIONS.hover);
    for (let step = 0; step < 100; step++) state = stepOnce(state, plant);

    const samples = ringToArray(state.telemetry);
    expect(samples.length).toBe(6);
    expect(samples[0]!.t).toBe(0);
    expect(samples[5]!.t).toBeCloseTo(0.1, 12);
    for (let index = 1; index < samples.length; index++) {
      expect(samples[index]!.t - samples[index - 1]!.t).toBeCloseTo(0.02, 12);
    }
    expect(state.trail.length).toBe(6);
    expect(ringToArray(state.trail)[5]).toEqual(state.multirotor.body.position);
  });

  it("tracks ground contact during takeoff", () => {
    const plant = makePlant();
    let state = createSimulation(plant, MISSIONS.takeoff);
    expect(state.multirotor.motorSpeeds.every((omega) => omega === 0)).toBe(true);
    state = stepOnce(state, plant);
    expect(state.contact).toBe(true);
    for (let step = 0; step < 2000; step++) state = stepOnce(state, plant);
    expect(state.stats.contactSteps).toBeGreaterThan(0);
    expect(state.stats.maxTiltRad).toBeGreaterThanOrEqual(0);
  });
});
