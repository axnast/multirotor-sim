/**
 * Named disturbance scenarios: plain data, shared by the demo, viewer and the
 * M6 benchmarks. `meanSpeed / gustSigma` are in m/s; direction is measured
 * counter-clockwise from world +x. Vertical gust sigma is 0.3× the horizontal
 * value, as specified.
 */

import { batteryParams, REALISTIC_PACK, type BatteryPack } from "./battery";
import { DEFAULT_SENSOR_PARAMS } from "./noise";
import type {
  BatteryParams,
  DisturbanceParams,
  PayloadEvent,
  SensorParams,
} from "./types";

export type ScenarioId = "calm" | "light" | "moderate" | "gusty" | "strong";

export interface ScenarioSpec {
  readonly id: ScenarioId;
  readonly name: string;
  readonly meanSpeed: number;
  readonly gustSigma: number;
  readonly directionDeg: number;
}

export const SCENARIOS: Record<ScenarioId, ScenarioSpec> = {
  calm: { id: "calm", name: "Calm", meanSpeed: 0, gustSigma: 0, directionDeg: 0 },
  light: { id: "light", name: "Light", meanSpeed: 2, gustSigma: 0.5, directionDeg: 0 },
  moderate: { id: "moderate", name: "Moderate", meanSpeed: 5, gustSigma: 1.5, directionDeg: 0 },
  gusty: { id: "gusty", name: "Gusty", meanSpeed: 5, gustSigma: 3, directionDeg: 0 },
  strong: { id: "strong", name: "Strong", meanSpeed: 8, gustSigma: 2.5, directionDeg: 0 },
};

export const SCENARIO_IDS: readonly ScenarioId[] = [
  "calm", "light", "moderate", "gusty", "strong",
];

/** Plain-data overrides; absent fields keep the scenario defaults. */
export interface ScenarioOptions {
  readonly pack?: BatteryPack;
  readonly battery?: boolean;
  readonly sensors?: boolean;
  readonly stateOfCharge?: number;
  readonly payload?: PayloadEvent | null;
}

export function disturbanceParamsFor(
  scenario: ScenarioId,
  seed: number,
  options: ScenarioOptions = {},
): DisturbanceParams {
  const spec = SCENARIOS[scenario];
  const direction = (spec.directionDeg * Math.PI) / 180;
  const horizontalSigma = spec.gustSigma;
  const pack = options.pack ?? REALISTIC_PACK;
  const battery: BatteryParams = batteryParams(
    pack,
    options.stateOfCharge ?? 1,
    options.battery ?? false,
  );
  const sensors: SensorParams = {
    ...DEFAULT_SENSOR_PARAMS,
    enabled: options.sensors ?? false,
  };
  return {
    seed,
    wind: {
      enabled: spec.meanSpeed > 0 || horizontalSigma > 0,
      mean: [
        spec.meanSpeed * Math.cos(direction),
        spec.meanSpeed * Math.sin(direction),
        0,
      ],
      gustSigma: [
        horizontalSigma,
        horizontalSigma,
        0.3 * horizontalSigma,
      ],
      correlationTime: 2,
      cosineGust: null,
    },
    sensors,
    battery,
    payload: options.payload ?? null,
  };
}
