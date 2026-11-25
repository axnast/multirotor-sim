import { describe, expect, it } from "vitest";

import { defaultControllerParams } from "../../src/core/control/controller";
import type { ControllerParams } from "../../src/core/control/types";
import { STRESS_PACK } from "../../src/core/disturbance/battery";
import { disturbanceParamsFor } from "../../src/core/disturbance/scenario";
import type { DisturbanceParams } from "../../src/core/disturbance/types";
import { makeHex } from "../../src/core/multirotor/config";
import { SIM_DT, TELEMETRY_CAPACITY } from "../../src/viewer/sim/constants";
import { MISSIONS } from "../../src/viewer/sim/mission";
import { createSimulation, stepOnce, tiltAngleRad } from "../../src/viewer/sim/runner";

const SEEDS = [1, 2, 3, 4, 5] as const;
const WINDOWS = [[20, 60], [90, 120]] as const;
const SCENARIOS = ["calm", "light", "moderate", "gusty", "strong"] as const;

function plant(params: ControllerParams = defaultControllerParams(makeHex({ groundPlane: true }))) {
  return { config: makeHex({ groundPlane: true }), params };
}

interface Stats {
  readonly mean: number;
  readonly min: number;
  readonly max: number;
}

function stats(values: readonly number[]): Stats {
  return {
    mean: values.reduce((a, b) => a + b, 0) / values.length,
    min: Math.min(...values),
    max: Math.max(...values),
  };
}

/** True-position RMS in a window of a 120 s run (measurement noise never used). */
function windowRms(
  params: DisturbanceParams,
  start: number,
  end: number,
): number {
  const p = plant();
  let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
  let sumSquared = 0;
  let count = 0;
  for (let step = 0; step < 120_000; step++) {
    state = stepOnce(state, p);
    if (state.simTime >= start && state.simTime <= end) {
      const position = state.multirotor.body.position;
      const error = Math.hypot(position[0], position[1], position[2] - 5);
      sumSquared += error * error;
      count++;
    }
  }
  return Math.sqrt(sumSquared / count);
}

describe("disturbance scenarios (five seeds, 120 s)", () => {
  it("holds a constant 5 m/s wind with the drag-predicted tilt", () => {
    const calm = disturbanceParamsFor("calm", 1);
    const constantWind: DisturbanceParams = {
      ...calm,
      wind: {
        enabled: true,
        mean: [5, 0, 0],
        gustSigma: [0, 0, 0],
        correlationTime: 2,
        cosineGust: null,
      },
    };
    const p = plant();
    let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, constantWind);
    let sumSquared = 0;
    let count = 0;
    let maxTilt = 0;
    for (let step = 0; step < 120_000; step++) {
      state = stepOnce(state, p);
      if (state.simTime >= 110) {
        const position = state.multirotor.body.position;
        sumSquared += position[0] ** 2 + position[1] ** 2 + (position[2] - 5) ** 2;
        count++;
        maxTilt = Math.max(maxTilt, tiltAngleRad(state.multirotor.body.orientation));
      }
    }
    // Noise OFF: the release trims the constant-wind offset; the M3 gate froze at 3.68 m.
    expect(Math.sqrt(sumSquared / count)).toBeLessThan(0.05);
    expect((maxTilt * 180) / Math.PI).toBeGreaterThan(6);
    expect(state.controller.position[0]!.integral).toBeLessThan(-100);
    expect(SIM_DT).toBe(0.001);
  }, 300_000);

  it("orders light < moderate < gusty < strong in both 40 s and 30 s windows", () => {
    const table: Record<string, Record<string, Stats>> = {};
    for (const id of SCENARIOS) {
      table[id] = {};
      for (const [start, end] of WINDOWS) {
        const key = `${start}-${end}`;
        const values = SEEDS.map((seed) =>
          windowRms(disturbanceParamsFor(id, seed), start, end),
        );
        table[id]![key] = stats(values);
        const s = table[id]![key]!;
        console.log(
          `[scenario ${id}] RMS ${key}: mean ${s.mean.toFixed(4)} m ` +
            `[min ${s.min.toFixed(4)}, max ${s.max.toFixed(4)}]`,
        );
      }
    }
    for (const [start, end] of WINDOWS) {
      const key = `${start}-${end}`;
      expect(table.calm![key]!.mean).toBeLessThan(1e-9);
      expect(table.light![key]!.mean).toBeLessThan(table.moderate![key]!.mean);
      expect(table.moderate![key]!.mean).toBeLessThan(table.gusty![key]!.mean);
      expect(table.gusty![key]!.mean).toBeLessThan(table.strong![key]!.mean);
    }
  }, 900_000);

  it("is bit-identical for the same seed and different for another seed", () => {
    const digest = (seed: number): string => {
      const params = disturbanceParamsFor("gusty", seed, {
        sensors: true,
        battery: true,
        pack: STRESS_PACK,
        stateOfCharge: 0.5,
      });
      const p = plant();
      let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
      const rows: string[] = [];
      for (let step = 0; step < 10_000; step++) {
        state = stepOnce(state, p);
        if (step % 20 === 0) {
          rows.push([
            ...state.multirotor.body.position,
            ...state.multirotor.motorSpeeds,
            ...state.diagnostics!.wind,
            state.diagnostics!.stateOfCharge,
            state.diagnostics!.voltage,
            ...state.last!.wrench,
          ].map((value) => value.toFixed(9)).join(","));
        }
      }
      let hash = 2166136261;
      for (const character of rows.join("\n")) {
        hash ^= character.charCodeAt(0);
        hash = Math.imul(hash, 16777619) >>> 0;
      }
      return hash.toString(16);
    };
    expect(digest(12345)).toBe(digest(12345));
    expect(digest(12345)).not.toBe(digest(12346));
  }, 120_000);

  it("keeps motors and state finite in the gusty scenario at 4x-equivalent steps", () => {
    const p = plant();
    const params = disturbanceParamsFor("gusty", 11);
    let state = createSimulation(p, MISSIONS.square, TELEMETRY_CAPACITY, params);
    for (let step = 0; step < 60_000; step++) {
      state = stepOnce(state, p);
      for (const speed of state.multirotor.motorSpeeds) {
        expect(Number.isFinite(speed)).toBe(true);
      }
    }
    expect(Number.isFinite(state.simTime)).toBe(true);
  }, 120_000);
});
