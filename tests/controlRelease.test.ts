import { describe, expect, it } from "vitest";

import {
  defaultControllerParams,
  initialControllerState,
} from "../src/core/control/controller";
import { positionStep } from "../src/core/control/position";
import type {
  ControllerParams,
  ControllerState,
  Measurement,
} from "../src/core/control/types";
import { disturbanceParamsFor } from "../src/core/disturbance/scenario";
import type { DisturbanceParams } from "../src/core/disturbance/types";
import { QUAT_IDENTITY } from "../src/core/math/quat";
import { makeHex } from "../src/core/multirotor/config";
import { TELEMETRY_CAPACITY } from "../src/viewer/sim/constants";
import { MISSIONS } from "../src/viewer/sim/mission";
import {
  createSimulation,
  setManualSetpoint,
  stepOnce,
  type SimulationPlant,
} from "../src/viewer/sim/runner";

const DESIGN: Pick<ControllerParams["position"], "integralDwell" | "integralTrimGain" | "integralReleaseGuard"> = {
  integralDwell: 5,
  integralTrimGain: 3,
  integralReleaseGuard: 2,
};

interface RunResult {
  /** Mean true position error over the final 10 s (never the noisy measurement). */
  readonly meanTrueError: number;
  readonly finalTrueError: number;
  /** Fraction of steps with any position axis in the released state. */
  readonly releaseFraction: number;
  readonly integral: readonly [number, number, number];
}

function plant(params?: ControllerParams): SimulationPlant {
  const config = makeHex({ groundPlane: true });
  const base = defaultControllerParams(config);
  return {
    config,
    params: params ? { ...base, position: { ...base.position, ...params.position } } : base,
  };
}

function run(
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

function withRelease(
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

describe("persistent-error integral release", () => {
  it("dwell=0 is the M3 baseline: the release never arms under constant wind", () => {
    const base = plant();
    const params = disturbanceParamsFor("calm", 1, { sensors: true });
    // calm has no wind; build a constant-wind case from the calm data.
    const constantWind: DisturbanceParams = {
      ...params,
      wind: { ...params.wind, enabled: true, mean: [5, 0, 0], gustSigma: [0, 0, 0] },
    };
    const result = run(withRelease(base, { integralDwell: 0 }), 60_000, constantWind);
    expect(result.releaseFraction).toBe(0);
    // M3 gate behaviour: frozen at a ~3.7 m offset, not trimmed.
    expect(result.meanTrueError).toBeGreaterThan(2);
    expect(result.integral[0]).toBeLessThan(0);
  }, 180_000);

  it("releases a constant-wind offset with sensor noise and trims it below 0.05 m by 120 s", () => {
    const base = withRelease(plant(), {});
    const params = disturbanceParamsFor("moderate", 1, { sensors: true });
    const constantWind: DisturbanceParams = {
      ...params,
      wind: { ...params.wind, gustSigma: [0, 0, 0] },
    };
    const result = run(base, 120_000, constantWind);
    expect(result.releaseFraction).toBeGreaterThan(0);
    expect(result.meanTrueError).toBeLessThan(0.05);
    expect(result.integral[0]).toBeLessThan(-100);
  }, 300_000);

  it("the setpoint-age guard clears the latch on a fresh command", () => {
    const config = makeHex({ groundPlane: true });
    const params = defaultControllerParams(config);
    const initial = initialControllerState();
    const latched = (): ControllerState["position"][number] => ({
      ...initial.position[0]!,
      vf: 0,
      stallTime: 2,
      released: true,
      lastSetpoint: 0,
      setpointAge: 100,
    });
    const state = {
      position: [latched(), latched(), latched()] as ControllerState["position"],
      rate: initial.rate,
    };
    const measurement: Measurement = {
      position: [0, 0, 5],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    };
    const next = positionStep(
      state.position,
      measurement,
      { position: [5, 0, 5], yaw: 0 },
      params,
      0.001,
    );
    expect(next.state[0]!.released).toBe(false);
    expect(next.state[0]!.setpointAge).toBe(0);
  });

  it("stays dormant during a 5 m position step", () => {
    const base = withRelease(plant(), {});
    const result = run(base, 25_000, disturbanceParamsFor("calm", 3), [5, 0, 5]);
    expect(result.releaseFraction).toBe(0);
    expect(result.finalTrueError).toBeLessThan(0.01);
  }, 120_000);

  it("recovers from a centred +20% payload that previously froze on the ground", () => {
    const base = withRelease(plant(), {});
    const params = disturbanceParamsFor("calm", 7, {
      payload: { atTime: 0, payload: { mass: 0.3, position: [0, 0, 0] } },
    });
    const result = run(base, 120_000, params);
    expect(result.releaseFraction).toBeGreaterThan(0);
    expect(result.finalTrueError).toBeLessThan(0.1);
    expect(result.meanTrueError).toBeLessThan(0.1);
  }, 300_000);

  it("recovers from the centred 10 g payload that previously froze at 0.2 m", () => {
    const base = withRelease(plant(), {});
    const params = disturbanceParamsFor("calm", 7, {
      payload: { atTime: 0, payload: { mass: 0.01, position: [0, 0, 0] } },
    });
    const result = run(base, 120_000, params);
    expect(result.finalTrueError).toBeLessThan(0.1);
  }, 300_000);
});
