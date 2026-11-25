/**
 * Pre-M5 regression tripwire for the M3 flight scenario.
 *
 * `FROZEN_M3_DIGEST` was captured from HEAD 15cd29b (before any M5 code) with
 * the exact loop in `runLegacyTrace`. The helper is deliberately a copy of the
 * pre-M5 code path: controllerStepDetailed -> mixDetailed ->
 * stepMultirotorDetailed, with no disturbance boundary of any kind.
 *
 * The digest hashes raw Float64LE bytes, not a rounded CSV, so any change in
 * the physics/control arithmetic changes the hash. It is the bit-identity
 * definition for constraint 2 of Milestone 5.
 */

import { createHash } from "node:crypto";

import {
  controllerStepDetailed,
  defaultControllerParams,
  initialControllerState,
} from "../../src/core/control/controller";
import type {
  ControllerState,
  Setpoint,
} from "../../src/core/control/types";
import { makeHex } from "../../src/core/multirotor/config";
import {
  stepMultirotorDetailed,
  type MultirotorState,
} from "../../src/core/multirotor/dynamics";
import { mixDetailed } from "../../src/core/multirotor/mixer";
import { SIM_DT } from "../../src/viewer/sim/constants";
import { MISSIONS, missionInitialMultirotor } from "../../src/viewer/sim/mission";
import {
  createSimulation,
  setManualSetpoint,
  stepOnce,
  type SimulationState,
} from "../../src/viewer/sim/runner";

/** Hexagonal airframe with ground contact, as in scripts/flightDemo.ts. */
export function m3Plant(): {
  readonly config: ReturnType<typeof makeHex>;
  readonly params: ReturnType<typeof defaultControllerParams>;
} {
  const config = makeHex({ groundPlane: true });
  return { config, params: defaultControllerParams(config) };
}

/** Frozen digest of 2251 samples at 20 ms cadence over 45 s. */
export const FROZEN_M3_DIGEST =
  "8ed20e2ed39d155e248b688c3e70a1e4a5f440640c498bd5a5908ca791d9444b";

export const M3_STEPS = 45_000;
export const M3_SAMPLE_STEPS = 20;
/** pos3, vel3, quat4, omega3, motors6, intPos3, intRate3, wrench4. */
export const M3_FIELDS = 29;

export interface M3Sample {
  readonly position: readonly number[];
  readonly velocity: readonly number[];
  readonly orientation: readonly number[];
  readonly angularVelocity: readonly number[];
  readonly motorSpeeds: readonly number[];
  readonly positionIntegrals: readonly number[];
  readonly rateIntegrals: readonly number[];
  readonly wrench: readonly number[];
}

export function sampleOf(
  state: MultirotorState,
  controller: ControllerState,
  wrench: readonly number[],
): M3Sample {
  return {
    position: [...state.body.position],
    velocity: [...state.body.velocity],
    orientation: [...state.body.orientation],
    angularVelocity: [...state.body.angularVelocity],
    motorSpeeds: [...state.motorSpeeds],
    positionIntegrals: controller.position.map((axis) => axis.integral),
    rateIntegrals: controller.rate.map((axis) => axis.integral),
    wrench: [...wrench],
  };
}

/** Hash the exact Float64LE representation of every sample. */
export function digestM3Samples(samples: readonly M3Sample[]): string {
  const bytes = Buffer.alloc(samples.length * M3_FIELDS * 8);
  for (const [index, sample] of samples.entries()) {
    const values = new Float64Array(M3_FIELDS);
    let i = 0;
    for (const v of sample.position) values[i++] = v;
    for (const v of sample.velocity) values[i++] = v;
    for (const v of sample.orientation) values[i++] = v;
    for (const v of sample.angularVelocity) values[i++] = v;
    for (const v of sample.motorSpeeds) values[i++] = v;
    for (const v of sample.positionIntegrals) values[i++] = v;
    for (const v of sample.rateIntegrals) values[i++] = v;
    for (const v of sample.wrench) values[i++] = v;
    if (i !== M3_FIELDS) throw new Error(`field count ${i}`);
    Buffer.from(values.buffer, values.byteOffset, values.byteLength)
      .copy(bytes, index * M3_FIELDS * 8);
  }
  return createHash("sha256").update(bytes).digest("hex");
}

function setpointAt(step: number): Setpoint {
  return step < 20_000
    ? { position: [0, 0, 5], yaw: 0 }
    : { position: [5, 0, 5], yaw: 0 };
}

/** The pre-M5 loop: takeoff 20 s, 5 m step 20 s, hold 5 s. */
export function runLegacyTrace(): M3Sample[] {
  const { config, params } = m3Plant();
  let state = missionInitialMultirotor(config, MISSIONS.takeoff);
  let controller = initialControllerState();
  let wrench: readonly number[] = [0, 0, 0, 0];
  const samples: M3Sample[] = [sampleOf(state, controller, wrench)];
  for (let step = 1; step <= M3_STEPS; step++) {
    const command = controllerStepDetailed(
      controller,
      {
        position: state.body.position,
        velocity: state.body.velocity,
        orientation: state.body.orientation,
        angularVelocity: state.body.angularVelocity,
      },
      setpointAt(step - 1),
      params,
      SIM_DT,
    );
    controller = command.state;
    wrench = command.wrench;
    const mixed = mixDetailed(config, command.wrench);
    state = stepMultirotorDetailed(
      state, config, mixed.motorSpeeds, SIM_DT,
    ).state;
    if (step % M3_SAMPLE_STEPS === 0) samples.push(sampleOf(state, controller, wrench));
  }
  return samples;
}

/** The same scenario through the viewer runner with all disturbances absent. */
export function runRunnerTrace(): {
  readonly samples: M3Sample[];
  readonly state: SimulationState;
} {
  const plant = m3Plant();
  let state = createSimulation(plant, MISSIONS.takeoff);
  const samples: M3Sample[] = [sampleOf(
    state.multirotor,
    state.controller,
    [0, 0, 0, 0],
  )];
  let lastSetpoint = -1;
  for (let step = 1; step <= M3_STEPS; step++) {
    const wanted = setpointAt(step - 1);
    if (lastSetpoint !== wanted.position[0]) {
      state = setManualSetpoint(state, wanted);
      lastSetpoint = wanted.position[0];
    }
    state = stepOnce(state, plant);
    if (step % M3_SAMPLE_STEPS === 0) {
      samples.push(sampleOf(
        state.multirotor,
        state.controller,
        state.last?.wrench ?? [0, 0, 0, 0],
      ));
    }
  }
  return { samples, state };
}
