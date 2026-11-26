/**
 * The shared one-step simulation loop: disturbance boundary -> controller ->
 * mixer -> plant. Pure and DOM-free, with no viewer concepts (mission,
 * telemetry, accumulator). The runner, flight demo, disturbance demo and the
 * M6 tuning/benchmark tools all advance physics through `simulateStep`, so a
 * gain set is evaluated by exactly the same arithmetic the viewer uses.
 *
 * All state is passed explicitly. With no disturbance active this is a pure
 * pass-through of the same objects used by the pre-M5 step order.
 */

import {
  controllerStepDetailed,
  type ControllerStepDetailedResult,
} from "../control/controller";
import type {
  ControllerParams,
  ControllerState,
  Measurement,
  Setpoint,
} from "../control/types";
import {
  disturbanceStep,
} from "../disturbance/step";
import type {
  DisturbanceDiagnostics,
  DisturbanceParams,
  DisturbanceState,
} from "../disturbance/types";
import { STILL_AIR, stepMultirotorDetailed } from "../multirotor/dynamics";
import type {
  Environment,
  MultirotorState,
} from "../multirotor/dynamics";
import type { GroundContactResult } from "../multirotor/ground";
import type { MultirotorConfig } from "../multirotor/config";
import { mixDetailed, type MixResult } from "../multirotor/mixer";

/** Nominal config and controller gains: what the controller and mixer see. */
export interface SimulationPlant {
  readonly config: MultirotorConfig;
  readonly params: ControllerParams;
}

/** Everything the closed loop needs to carry from one step to the next. */
export interface CoreState {
  readonly multirotor: MultirotorState;
  readonly controller: ControllerState;
  /** Disturbance memory, or null when every model is disabled. */
  readonly disturbance: DisturbanceState | null;
  readonly disturbanceParams: DisturbanceParams | null;
  readonly diagnostics: DisturbanceDiagnostics | null;
  /** Config actually used by the plant; the controller never sees it. */
  readonly trueConfig: MultirotorConfig;
}

export interface StepResult {
  readonly state: MultirotorState;
  readonly controller: ControllerState;
  readonly last: ControllerStepDetailedResult;
  readonly lastMix: MixResult;
  readonly disturbance: DisturbanceState | null;
  /** Corrupted measurement when disturbances are active, else null. */
  readonly disturbedMeasurement: Measurement | null;
  readonly diagnostics: DisturbanceDiagnostics | null;
  readonly trueConfig: MultirotorConfig;
  readonly environment: Environment;
  readonly ground: GroundContactResult | null;
  readonly contact: boolean;
}

/** The uncorrupted simulator state is the controller measurement by default. */
export function measurementOf(state: MultirotorState): Measurement {
  return {
    position: state.body.position,
    velocity: state.body.velocity,
    orientation: state.body.orientation,
    angularVelocity: state.body.angularVelocity,
  };
}

/**
 * Advance exactly one fixed-size step. The disturbance boundary is unchanged
 * from the viewer: only the plant sees `trueConfig` and the wind, while the
 * controller and mixer keep the nominal config and gains.
 */
export function simulateStep(
  state: CoreState,
  plant: SimulationPlant,
  setpoint: Setpoint,
  dt: number,
): StepResult {
  const truth = measurementOf(state.multirotor);
  const disturbed =
    state.disturbance !== null && state.disturbanceParams !== null
      ? disturbanceStep(
          state.disturbance,
          state.disturbanceParams,
          truth,
          state.multirotor.motorSpeeds,
          plant.config,
          dt,
        )
      : null;
  const command = controllerStepDetailed(
    state.controller,
    disturbed?.measurement ?? truth,
    setpoint,
    plant.params,
    dt,
  );
  const mixed = mixDetailed(plant.config, command.wrench);
  const trueConfig = disturbed?.trueConfig ?? state.trueConfig;
  const environment = disturbed?.environment ?? STILL_AIR;
  // A voltage drop can lower the ceiling below the rotors' current speed.
  // The core state invariant forbids overspeed, so project onto the new
  // ceiling before the step (documented as an instantaneous ESC limit).
  const plantMultirotor =
    disturbed !== null && disturbed.diagnostics.maxOmegaScale < 1
      ? {
          ...state.multirotor,
          motorSpeeds: state.multirotor.motorSpeeds.map((omega, index) =>
            Math.min(omega, trueConfig.motors[index]?.maxOmega ?? omega),
          ),
        }
      : state.multirotor;
  const result = stepMultirotorDetailed(
    plantMultirotor,
    trueConfig,
    mixed.motorSpeeds,
    dt,
    environment,
  );

  return {
    state: result.state,
    controller: command.state,
    last: command,
    lastMix: mixed,
    disturbance: disturbed?.state ?? state.disturbance,
    disturbedMeasurement: disturbed?.measurement ?? null,
    diagnostics: disturbed?.diagnostics ?? state.diagnostics,
    trueConfig,
    environment,
    ground: result.ground,
    contact: result.ground?.contacted ?? false,
  };
}

/**
 * Open-loop plant advance, used by the phase-C hover demo and available for
 * reaction-curve identification in the M6 scripts. The controller/mixer step
 * is deliberately absent.
 */
export function stepPlant(
  state: MultirotorState,
  config: MultirotorConfig,
  motorCommands: readonly number[],
  dt: number,
  environment: Environment = STILL_AIR,
): ReturnType<typeof stepMultirotorDetailed> {
  return stepMultirotorDetailed(state, config, motorCommands, dt, environment);
}
