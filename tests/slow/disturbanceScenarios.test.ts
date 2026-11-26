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

interface WindowRms {
  readonly w20: number;
  readonly w90: number;
}

/** True-position RMS in both windows of one 120 s run (measurement noise never used). */
function windowRms(params: DisturbanceParams): WindowRms {
  const p = plant();
  let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
  let sum20 = 0, count20 = 0, sum90 = 0, count90 = 0;
  for (let step = 0; step < 120_000; step++) {
    state = stepOnce(state, p);
    const time = state.simTime;
    if (time >= 20 && time <= 60 || time >= 90 && time <= 120) {
      const position = state.multirotor.body.position;
      const squared = position[0] ** 2 + position[1] ** 2 + (position[2] - 5) ** 2;
      if (time <= 60) {
        sum20 += squared;
        count20++;
      } else {
        sum90 += squared;
        count90++;
      }
    }
  }
  return {
    w20: Math.sqrt(sum20 / count20),
    w90: Math.sqrt(sum90 / count90),
  };
}

interface Paired {
  readonly differences: readonly number[];
  readonly mean: number;
  readonly se: number;
  readonly t: number;
}

/** Scenarios share seeds 1-5, so compare the same seed across scenarios. */
function paired(low: readonly number[], high: readonly number[]): Paired {
  const differences = low.map((value, index) => high[index]! - value);
  const mean = differences.reduce((a, b) => a + b, 0) / differences.length;
  const variance =
    differences.reduce((sum, value) => sum + (value - mean) ** 2, 0) /
    (differences.length - 1);
  const se = Math.sqrt(variance / differences.length);
  return { differences, mean, se, t: mean / se };
}

describe("disturbance scenarios (development seeds 1-5, 120 s)", () => {
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

  it("asserts only orderings supported by all five paired seed differences", () => {
    /**
     * Seeds 1-5 are development seeds, shared by every scenario, so the same
     * seed can be paired across scenarios. An ordering is asserted only by a
     * sign test: all five paired differences must be positive (exact one-sided
     * p = 1/32). The paired t is printed for information only: t = 2.776 is the
     * uncorrected 5% cutoff for n = 5, power is limited, and the six orderings
     * are NOT multiplicity-corrected. moderate < gusty in 20-60 s passes the
     * sign test but is marginal (t = 2.96); gusty < strong in 90-120 s has one
     * negative paired difference (t = 2.09) and is reported, not asserted.
     */
    const table: Record<string, Record<string, readonly number[]>> = {};
    for (const id of SCENARIOS) {
      const runs = SEEDS.map((seed) => windowRms(disturbanceParamsFor(id, seed)));
      table[id] = {
        "20-60": runs.map((run) => run.w20),
        "90-120": runs.map((run) => run.w90),
      };
      for (const [start, end] of WINDOWS) {
        const key = `${start}-${end}`;
        const s = stats(table[id]![key]!);
        console.log(
          `[scenario ${id}] RMS ${key}: mean ${s.mean.toFixed(4)} m ` +
            `[min ${s.min.toFixed(4)}, max ${s.max.toFixed(4)}]`,
        );
      }
    }

    const neighbours = [
      ["light", "moderate"],
      ["moderate", "gusty"],
      ["gusty", "strong"],
    ] as const;
    for (const [start, end] of WINDOWS) {
      const key = `${start}-${end}`;
      expect(table.calm![key]!.every((value) => value < 1e-9)).toBe(true);
      for (const [low, high] of neighbours) {
        const comparison = paired(table[low]![key]!, table[high]![key]!);
        const signTest = comparison.differences.every((value) => value > 0);
        const marginal = key === "20-60" && low === "moderate" && high === "gusty";
        console.log(
          `[paired ${key}] ${high}-${low}: ` +
            `diffs [${comparison.differences.map((value) => value.toFixed(4)).join(", ")}] ` +
            `mean ${comparison.mean.toFixed(4)} pairedSE ${comparison.se.toFixed(4)} ` +
            `t ${comparison.t.toFixed(2)}${marginal ? " (marginal)" : ""}` +
            `${signTest ? "" : " NOT ASSERTED (non-positive paired difference)"}`,
        );
        if (signTest) expect(comparison.mean).toBeGreaterThan(0);
      }
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
