/** Shared runner for the persistent-error release tests (both fast and slow). */

import {
  defaultControllerParams,
  initialControllerState,
} from "../../src/core/control/controller";
import type {
  ControllerParams,
  ControllerState,
  Measurement,
} from "../../src/core/control/types";
import { disturbanceParamsFor } from "../../src/core/disturbance/scenario";
import type { DisturbanceParams } from "../../src/core/disturbance/types";
import { QUAT_IDENTITY } from "../../src/core/math/quat";
import { makeHex } from "../../src/core/multirotor/config";
import { TELEMETRY_CAPACITY } from "../../src/viewer/sim/constants";
import { MISSIONS } from "../../src/viewer/sim/mission";
import {
  createSimulation,
  setManualSetpoint,
  stepOnce,
  type SimulationPlant,
} from "../../src/viewer/sim/runner";

export const DESIGN: Pick<
  ControllerParams["position"],
  "integralDwell" | "integralTrimGain" | "integralReleaseGuard"
> = {
  integralDwell: 5,
  integralTrimGain: 3,
  integralReleaseGuard: 2,
};

export interface RunResult {
  /** Mean true position error over the final 10 s (never the noisy measurement). */
  readonly meanTrueError: number;
  readonly finalTrueError: number;
  /** Fraction of steps with any position axis in the released state. */
  readonly releaseFraction: number;
  readonly integral: readonly [number, number, number];
}

export function plant(params?: ControllerParams): SimulationPlant {
  const config = makeHex({ groundPlane: true });
  const base = defaultControllerParams(config);
  return {
    config,
    params: params
      ? { ...base, position: { ...base.position, ...params.position } }
      : base,
  };
}

export function run(
  p: SimulationPlant,
  steps: number,
  disturbance: DisturbanceParams,
  manualSetpoint?: readonly [number, number, number],
): RunResult {
  let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, disturbance);
  if (manualSetpoint) {
    state = setManualSetpoint(state, { position: [...manualSetpoint], yaw: 0 });
  }
  let errorSum = 0;
  let errorCount = 0;
  let releasedSteps = 0;
  for (let step = 0; step < steps; step++) {
    state = stepOnce(state, p);
    if (state.simTime >= steps * 0.001 - 10) {
      const position = state.multirotor.body.position;
      const setpoint = state.setpoint.position;
      errorSum += Math.hypot(
        position[0] - setpoint[0],
        position[1] - setpoint[1],
        position[2] - setpoint[2],
      );
      errorCount++;
    }
    if (state.controller.position.some((axis) => axis.released)) releasedSteps++;
  }
  const position = state.multirotor.body.position;
  const setpoint = state.setpoint.position;
  return {
    meanTrueError: errorSum / errorCount,
    finalTrueError: Math.hypot(
      position[0] - setpoint[0],
      position[1] - setpoint[1],
      position[2] - setpoint[2],
    ),
    releaseFraction: releasedSteps / steps,
    integral: [
      state.controller.position[0]!.integral,
      state.controller.position[1]!.integral,
      state.controller.position[2]!.integral,
    ],
  };
}

export function withRelease(
  p: SimulationPlant,
  patch: Partial<typeof DESIGN>,
): SimulationPlant {
  return {
    config: p.config,
    params: {
      ...p.params,
      position: { ...p.params.position, ...DESIGN, ...patch },
    },
  };
}

/** A latched position state for testing the setpoint-age guard in isolation. */
export function latchedState(): ControllerState {
  const initial = initialControllerState();
  const latched = (): ControllerState["position"][number] => ({
    ...initial.position[0]!,
    vf: 0,
    stallTime: 2,
    released: true,
    lastSetpoint: 0,
    setpointAge: 100,
  });
  return {
    position: [latched(), latched(), latched()],
    rate: initial.rate,
  };
}

export const hoverMeasurement: Measurement = {
  position: [0, 0, 5],
  velocity: [0, 0, 0],
  orientation: QUAT_IDENTITY,
  angularVelocity: [0, 0, 0],
};

/** Build a gust-free constant-wind disturbance from a named scenario. */
export function constantWind(
  scenario: "calm" | "moderate",
  seed: number,
  sensors: boolean,
): DisturbanceParams {
  const base = disturbanceParamsFor(scenario, seed, { sensors });
  const speed = scenario === "calm" ? 5 : base.wind.mean[0];
  return {
    ...base,
    wind: { ...base.wind, enabled: true, mean: [speed, 0, 0], gustSigma: [0, 0, 0] },
  };
}
