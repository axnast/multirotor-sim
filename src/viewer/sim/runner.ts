/**
 * Pure, DOM-free fixed-step simulation runner for the viewer.
 *
 * All physics and control are delegated to src/core. The accumulator, mission
 * timeline, telemetry and trail are deterministic: identical inputs and frame
 * deltas produce identical states, independent of rendering.
 */

import {
  controllerStepDetailed,
  initialControllerState,
  type ControllerStepDetailedResult,
} from "../../core/control/controller";
import type {
  ControllerParams,
  ControllerState,
  Measurement,
  Setpoint,
} from "../../core/control/types";
import { quatRotateVector, type Quat } from "../../core/math/quat";
import { vec3Length, vec3Sub, type Vec3 } from "../../core/math/vec3";
import type { MultirotorConfig } from "../../core/multirotor/config";
import {
  STILL_AIR,
  stepMultirotorDetailed,
  type MultirotorState,
} from "../../core/multirotor/dynamics";
import {
  createDisturbanceState,
  disturbanceStep,
  isDisturbanceActive,
} from "../../core/disturbance/step";
import type {
  DisturbanceDiagnostics,
  DisturbanceParams,
  DisturbanceState,
} from "../../core/disturbance/types";
import { mixDetailed, type MixResult } from "../../core/multirotor/mixer";
import {
  DEFAULT_MAX_STEPS_PER_FRAME,
  MAX_FRAME_SECONDS,
  SAMPLE_INTERVAL_STEPS,
  SIM_DT,
  SPEED_MAX,
  SPEED_MIN,
  TELEMETRY_CAPACITY,
  TRAIL_CAPACITY,
} from "./constants";
import {
  advanceMission,
  createMissionRuntime,
  missionInitialMultirotor,
  missionSetpoint,
  type Mission,
  type MissionRuntime,
} from "./mission";
import {
  createRingBuffer,
  ringClear,
  ringPush,
  type RingBufferState,
} from "./ringBuffer";
import { makeTelemetrySample, type TelemetrySample } from "./telemetry";

export interface SimulationPlant {
  readonly config: MultirotorConfig;
  readonly params: ControllerParams;
}

export interface SimulationStats {
  readonly maxTiltRad: number;
  readonly maxPositionErrorM: number;
  readonly maxSpeedMps: number;
  /** Steps where the nominal mixer commanded above the plant's true ceiling. */
  readonly plantSaturationSteps: number;
  readonly saturationSteps: number;
  readonly contactSteps: number;
}

export interface SimulationState {
  readonly multirotor: MultirotorState;
  readonly controller: ControllerState;
  /** Disturbance memory, or null when every model is disabled. */
  readonly disturbance: DisturbanceState | null;
  readonly disturbanceParams: DisturbanceParams | null;
  readonly diagnostics: DisturbanceDiagnostics | null;
  /** Config actually used by the plant; the mixer/controller never see it. */
  readonly trueConfig: MultirotorConfig;
  readonly mission: MissionRuntime | null;
  /** Mission used by reset, even while in manual setpoint mode. */
  readonly homeMission: Mission;
  readonly setpoint: Setpoint;
  readonly stepCount: number;
  readonly simTime: number;
  readonly accumulator: number;
  readonly speed: number;
  readonly paused: boolean;
  readonly last: ControllerStepDetailedResult | null;
  readonly lastMix: MixResult | null;
  readonly contact: boolean;
  readonly stats: SimulationStats;
  readonly telemetry: RingBufferState<TelemetrySample>;
  readonly trail: RingBufferState<Vec3>;
  readonly droppedSteps: number;
  readonly cappedLastFrame: boolean;
}

export interface AdvanceResult {
  readonly state: SimulationState;
  readonly stepsRun: number;
  readonly capped: boolean;
}

/** Tilt of body +z above the world horizontal plane, in radians. */
export function tiltAngleRad(orientation: Quat): number {
  const bodyZ = quatRotateVector(orientation, [0, 0, 1]);
  return Math.acos(Math.min(1, Math.max(-1, bodyZ[2])));
}

/** The true simulator state is the controller measurement for now. */
export function measurementOf(state: MultirotorState): Measurement {
  return {
    position: state.body.position,
    velocity: state.body.velocity,
    orientation: state.body.orientation,
    angularVelocity: state.body.angularVelocity,
  };
}

function zeroStats(): SimulationStats {
  return {
    maxTiltRad: 0,
    maxPositionErrorM: 0,
    maxSpeedMps: 0,
    plantSaturationSteps: 0,
    saturationSteps: 0,
    contactSteps: 0,
  };
}

/** Build a fresh simulation at the start of `mission`. */
export function createSimulation(
  plant: SimulationPlant,
  mission: Mission,
  trailCapacity: number = TRAIL_CAPACITY,
  disturbance: DisturbanceParams | null = null,
): SimulationState {
  const activeParams =
    disturbance !== null && isDisturbanceActive(disturbance) ? disturbance : null;
  const runtime = createMissionRuntime(mission);
  const multirotor = missionInitialMultirotor(plant.config, mission);
  const setpoint = missionSetpoint(runtime);
  const initialSample = makeTelemetrySample({
    t: 0,
    multirotor,
    setpoint,
    last: null,
  });
  return {
    multirotor,
    controller: initialControllerState(),
    disturbance: activeParams === null ? null : createDisturbanceState(activeParams),
    disturbanceParams: activeParams,
    diagnostics: null,
    trueConfig: plant.config,
    mission: runtime,
    homeMission: mission,
    setpoint,
    stepCount: 0,
    simTime: 0,
    accumulator: 0,
    speed: 1,
    paused: false,
    last: null,
    lastMix: null,
    contact: false,
    stats: zeroStats(),
    telemetry: ringPush(
      createRingBuffer<TelemetrySample>(TELEMETRY_CAPACITY),
      initialSample,
    ),
    trail: ringPush(
      createRingBuffer<Vec3>(trailCapacity),
      multirotor.body.position,
    ),
    droppedSteps: 0,
    cappedLastFrame: false,
  };
}

/** Advance exactly one 1 ms core step, ignoring the accumulator and pause. */
export function stepOnce(
  state: SimulationState,
  plant: SimulationPlant,
): SimulationState {
  // Disturbance boundary: the controller is given the corrupted measurement,
  // the mixer and controller use the NOMINAL config/params, and only the plant
  // sees `trueConfig` and the wind. When nothing is enabled this is a pure
  // pass-through of the same objects, so the pre-M5 arithmetic is unchanged.
  const disturbed =
    state.disturbance !== null && state.disturbanceParams !== null
      ? disturbanceStep(
          state.disturbance,
          state.disturbanceParams,
          measurementOf(state.multirotor),
          state.multirotor.motorSpeeds,
          plant.config,
          SIM_DT,
        )
      : null;
  const command = controllerStepDetailed(
    state.controller,
    disturbed?.measurement ?? measurementOf(state.multirotor),
    state.setpoint,
    plant.params,
    SIM_DT,
  );
  const mixed = mixDetailed(plant.config, command.wrench);
  const trueConfig = disturbed?.trueConfig ?? plant.config;
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
    SIM_DT,
    environment,
  );
  const nextMultirotor = result.state;
  const nextStepCount = state.stepCount + 1;
  const nextRuntime =
    state.mission === null ? null : advanceMission(state.mission);
  const nextSetpoint =
    nextRuntime === null ? state.setpoint : missionSetpoint(nextRuntime);

  const positionError = vec3Length(
    vec3Sub(nextMultirotor.body.position, state.setpoint.position),
  );
  const stats: SimulationStats = {
    maxTiltRad: Math.max(state.stats.maxTiltRad, tiltAngleRad(nextMultirotor.body.orientation)),
    maxPositionErrorM: Math.max(state.stats.maxPositionErrorM, positionError),
    maxSpeedMps: Math.max(state.stats.maxSpeedMps, vec3Length(nextMultirotor.body.velocity)),
    plantSaturationSteps:
      state.stats.plantSaturationSteps +
      (mixed.motorSpeeds.some((omega, index) =>
        omega > (trueConfig.motors[index]?.maxOmega ?? Infinity) + 1e-9,
      ) ? 1 : 0),
    saturationSteps: state.stats.saturationSteps + (mixed.saturated ? 1 : 0),
    contactSteps: state.stats.contactSteps + (result.ground?.contacted ? 1 : 0),
  };

  let telemetry = state.telemetry;
  let trail = state.trail;
  if (nextStepCount % SAMPLE_INTERVAL_STEPS === 0) {
    telemetry = ringPush(
      telemetry,
      makeTelemetrySample({
        t: nextStepCount * SIM_DT,
        multirotor: nextMultirotor,
        setpoint: nextSetpoint,
        last: command,
      }),
    );
    trail = ringPush(trail, nextMultirotor.body.position);
  }

  return {
    ...state,
    multirotor: nextMultirotor,
    controller: command.state,
    disturbance: disturbed?.state ?? state.disturbance,
    diagnostics: disturbed?.diagnostics ?? state.diagnostics,
    trueConfig,
    mission: nextRuntime,
    setpoint: nextSetpoint,
    stepCount: nextStepCount,
    simTime: nextStepCount * SIM_DT,
    last: command,
    lastMix: mixed,
    contact: result.ground?.contacted ?? false,
    stats,
    telemetry,
    trail,
    cappedLastFrame: false,
  };
}

/**
 * Consume real elapsed time into fixed simulator steps. Excess accumulated
 * time beyond `maxStepsPerFrame` is dropped (not carried over), so a slow or
 * resumed tab cannot spiral.
 */
export function advanceSimulation(
  state: SimulationState,
  plant: SimulationPlant,
  realDeltaSeconds: number,
  maxStepsPerFrame: number = DEFAULT_MAX_STEPS_PER_FRAME,
): AdvanceResult {
  if (!Number.isFinite(realDeltaSeconds) || realDeltaSeconds < 0) {
    throw new RangeError("realDeltaSeconds must be a finite non-negative number");
  }
  if (!Number.isInteger(maxStepsPerFrame) || maxStepsPerFrame < 1) {
    throw new RangeError("maxStepsPerFrame must be a positive integer");
  }
  if (state.paused || realDeltaSeconds === 0) {
    return { state: { ...state, cappedLastFrame: false }, stepsRun: 0, capped: false };
  }

  const elapsed = Math.min(realDeltaSeconds, MAX_FRAME_SECONDS);
  let accumulator = state.accumulator + elapsed * state.speed;
  let next = state;
  let stepsRun = 0;
  // The epsilon absorbs binary error so an exact 1 ms contribution is consumed.
  while (accumulator >= SIM_DT - 1e-12 && stepsRun < maxStepsPerFrame) {
    next = stepOnce(next, plant);
    accumulator -= SIM_DT;
    stepsRun++;
  }

  let dropped = 0;
  if (accumulator >= SIM_DT) {
    while (accumulator >= SIM_DT) {
      accumulator -= SIM_DT;
      dropped++;
    }
  }
  // Absorb the last sub-epsilon remainder so the next frame starts clean.
  if (accumulator >= SIM_DT - 1e-12) {
    accumulator = 0;
    dropped++;
  }
  if (accumulator < 0) accumulator = 0;

  return {
    state: {
      ...next,
      accumulator,
      droppedSteps: state.droppedSteps + dropped,
      cappedLastFrame: dropped > 0,
    },
    stepsRun,
    capped: dropped > 0,
  };
}

/** Replace the mission and restart at its initial state, unpaused. */
export function selectMission(
  state: SimulationState,
  plant: SimulationPlant,
  mission: Mission,
): SimulationState {
  const fresh = createSimulation(
    plant, mission, state.trail.capacity, state.disturbanceParams,
  );
  return { ...fresh, speed: state.speed, paused: false };
}

/** Restart the current (or last selected) mission, preserving pause and speed. */
export function resetSimulation(
  state: SimulationState,
  plant: SimulationPlant,
): SimulationState {
  const fresh = createSimulation(
    plant, state.homeMission, state.trail.capacity, state.disturbanceParams,
  );
  return { ...fresh, speed: state.speed, paused: state.paused };
}

/** Install (or clear) disturbance params without resetting the flight. */
export function withDisturbance(
  state: SimulationState,
  plant: SimulationPlant,
  params: DisturbanceParams | null,
): SimulationState {
  const active = params !== null && isDisturbanceActive(params) ? params : null;
  return {
    ...state,
    disturbanceParams: active,
    disturbance: active === null ? null : createDisturbanceState(active),
    diagnostics: null,
    trueConfig: plant.config,
  };
}

export function setPaused(state: SimulationState, paused: boolean): SimulationState {
  return { ...state, paused };
}

export function setSpeed(state: SimulationState, speed: number): SimulationState {
  if (!Number.isFinite(speed)) {
    throw new RangeError("speed must be a finite number");
  }
  return {
    ...state,
    speed: Math.min(SPEED_MAX, Math.max(SPEED_MIN, speed)),
  };
}

/** Switch to manual setpoint mode without resetting the flight. */
export function setManualSetpoint(
  state: SimulationState,
  setpoint: Setpoint,
): SimulationState {
  return { ...state, mission: null, setpoint };
}

/** Change trail capacity, clearing the old trail. */
export function withTrailCapacity(
  state: SimulationState,
  capacity: number,
): SimulationState {
  return { ...state, trail: ringClear(createRingBuffer<Vec3>(capacity)) };
}
