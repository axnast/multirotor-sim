/**
 * Pure viewer-side disturbance settings → core `DisturbanceParams`.
 *
 * Keeping this conversion DOM-free makes the viewer's "everything off" default
 * directly testable against the frozen bit-identical M3 trace.
 */

import {
  REALISTIC_PACK,
  STRESS_PACK,
  type BatteryPack,
} from "../../core/disturbance/battery";
import {
  PAYLOAD_LARGE_STRESS,
  PAYLOAD_SMALL,
} from "../../core/disturbance/payload";
import {
  disturbanceParamsFor,
  type ScenarioId,
} from "../../core/disturbance/scenario";
import type {
  DisturbanceParams,
  PayloadSpec,
} from "../../core/disturbance/types";

export type ViewerPackId = "realistic" | "stress";
export type ViewerPayloadSize = "small" | "stress";

export interface ViewerDisturbanceSettings {
  readonly scenario: ScenarioId;
  readonly seed: number;
  readonly sensors: boolean;
  readonly battery: boolean;
  readonly pack: ViewerPackId;
  /** 0–1, passed to the battery model. */
  readonly stateOfCharge: number;
  /** Which payload the Drop button installs; Small is the safe default. */
  readonly payloadSize: ViewerPayloadSize;
  readonly payload: PayloadSpec | null;
}

/** Every model inactive: the runner stays on the pre-M5 code path. */
export const DEFAULT_VIEWER_DISTURBANCE: ViewerDisturbanceSettings = {
  scenario: "calm",
  seed: 1,
  sensors: false,
  battery: false,
  pack: "realistic",
  stateOfCharge: 1,
  payloadSize: "small",
  payload: null,
};

export function viewerPack(id: ViewerPackId): BatteryPack {
  return id === "stress" ? STRESS_PACK : REALISTIC_PACK;
}

export function viewerPayload(size: ViewerPayloadSize): PayloadSpec {
  return size === "stress" ? PAYLOAD_LARGE_STRESS : PAYLOAD_SMALL;
}

export function buildDisturbanceParams(
  settings: ViewerDisturbanceSettings,
): DisturbanceParams {
  const { seed } = settings;
  if (!Number.isInteger(seed) || seed < 0 || seed > 0xffffffff) {
    throw new RangeError("viewer disturbance seed must be a 32-bit unsigned integer");
  }
  return disturbanceParamsFor(settings.scenario, seed, {
    sensors: settings.sensors,
    battery: settings.battery,
    pack: viewerPack(settings.pack),
    stateOfCharge: settings.stateOfCharge,
    payload:
      settings.payload === null
        ? null
        : { atTime: 0, payload: settings.payload },
  });
}
