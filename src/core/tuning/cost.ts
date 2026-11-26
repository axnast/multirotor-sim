/**
 * The pre-registered M6 tuning cost. Every evaluation is a FULL 120 s suite
 * (no proxy horizon), with sensor noise at the default sigmas and the realistic
 * 6S battery sag enabled. All terms are divided by the default-gain value
 * under identical conditions (with documented floors), so the default scores
 * exactly 1.000. Failures return a finite 1e6 with a reason, never Inf/NaN.
 *
 * This module is pure: a CostSuite owns a memoized baseline table, but every
 * `evaluate` call is a deterministic function of (params, seed subset).
 */

import {
  defaultControllerParams,
  initialControllerState,
} from "../control/controller";
import {
  createDisturbanceState,
  isDisturbanceActive,
} from "../disturbance/step";
import { disturbanceParamsFor, type ScenarioId } from "../disturbance/scenario";
import { REALISTIC_PACK } from "../disturbance/battery";
import { PAYLOAD_MASS_20PCT } from "../disturbance/payload";
import type { DisturbanceParams } from "../disturbance/types";
import { QUAT_IDENTITY } from "../math/quat";
import { hoverMotorSpeeds, makeHex } from "../multirotor/config";
import type { MultirotorConfig } from "../multirotor/config";
import type { MultirotorState } from "../multirotor/dynamics";
import type { ControllerParams } from "../control/types";
import {
  simulateStep,
  type CoreState,
  type PlantOverride,
} from "../simulation/loop";
import { assertFixedDefaults } from "./params";

export const DT = 0.001;
export const FULL_STEPS = 120_000;
export const DEV_SEEDS = [1, 2, 3, 4, 5] as const;
/** Saturation is only a failure when sustained after the startup window. */
export const SATURATION_THRESHOLD = 0.2;
export const SUSTAINED_START_SECONDS = 5;
export const DIVERGENCE_LIMIT = 100;
/** Graded finite failure cost: 100*(1+excess), monotone in severity. */
export const FAILURE_BASE_COST = 100;
/** Non-finite states get a fixed value above every graded penalty. */
export const NONFINITE_COST = 1_000;

export function gradedPenalty(excess: number, nonfinite = false): number {
  if (nonfinite) return NONFINITE_COST;
  if (!Number.isFinite(excess) || excess < 0) {
    throw new RangeError("failure excess must be a finite non-negative number");
  }
  return FAILURE_BASE_COST * (1 + excess);
}

export type CostKey =
  | "step"
  | "square"
  | "light"
  | "moderate"
  | "gusty"
  | "strong"
  | "constant5"
  | "mass20"
  | "hoverJitter"
  | "mismatchKTau"
  | "mismatchMass";

/**
 * Pre-run amendment: weights re-focused on the pre-registered headline.
 * Plain data summing to 1.00; the objective is a weighted GEOMETRIC mean of the
 * per-term ratios, matching the held-out headline.
 */
export const TERM_WEIGHTS: Readonly<Record<CostKey, number>> = {
  step: 0.12,
  square: 0.08,
  light: 0.08,
  moderate: 0.1,
  gusty: 0.12,
  strong: 0.1,
  constant5: 0.08,
  mass20: 0.14,
  hoverJitter: 0.06,
  mismatchKTau: 0.06,
  mismatchMass: 0.06,
};

export const COST_KEYS = Object.keys(TERM_WEIGHTS) as CostKey[];

/** Documented floors: `max(baseline, floor)` is the normalizer per term. */
export const TERM_FLOORS = {
  overshoot: 0.02,
  settling: 0.5,
  ise: 1e-6,
  rms: 1e-4,
  contact: 1,
  jitter: 1e-6,
} as const;

export interface TermBreakdown {
  readonly key: CostKey;
  readonly weight: number;
  /** Raw (weighted-combined) metric value. */
  readonly value: number;
  readonly normalized: number;
  readonly reason: string | null;
}

export interface CostResult {
  readonly total: number;
  readonly terms: readonly TermBreakdown[];
  readonly seeds: readonly number[];
}

export interface RolloutMetrics {
  readonly rms: number;
  readonly earlyRms: number;
  readonly lateRms: number;
  readonly overshoot: number;
  readonly settling: number;
  readonly ise: number;
  readonly maxTilt: number;
  readonly saturationSteps: number;
  readonly contactSteps: number;
  readonly jitter: number;
  /** Peak true position/speed, used to verify divergence margin. */
  readonly maxPositionM: number;
  readonly maxSpeedMps: number;
  /** Sustained mixer-saturation fraction over t >= 5 s. */
  readonly sustainedSaturation: number;
  readonly failure: string | null;
  /** Severity used by `gradedPenalty` when `failure` is non-null. */
  readonly excess: number;
  readonly nonfinite: boolean;
}

function disturbance(
  scenario: ScenarioId,
  seed: number,
  options: Parameters<typeof disturbanceParamsFor>[2] = {},
): DisturbanceParams {
  return disturbanceParamsFor(scenario, seed, {
    sensors: true,
    battery: true,
    pack: REALISTIC_PACK,
    stateOfCharge: 1,
    ...options,
  });
}

function setpointFor(key: CostKey, time: number): readonly [number, number, number] {
  if (key === "step" || key === "mismatchKTau" || key === "mismatchMass") {
    return [5, 0, 5];
  }
  if (key === "square") {
    const corner = Math.min(4, Math.floor(time / 20));
    const corners: readonly (readonly [number, number, number])[] = [
      [5, 0, 5],
      [5, 5, 5],
      [0, 5, 5],
      [0, 0, 5],
      [5, 0, 5],
    ];
    return corners[corner]!;
  }
  return [0, 0, 5];
}

function initialState(config: MultirotorConfig): MultirotorState {
  return {
    body: {
      position: [0, 0, 5],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    },
    motorSpeeds: hoverMotorSpeeds(config),
  };
}

function paramsFor(key: CostKey, seed: number, sensors = true): DisturbanceParams {
  if (key === "mass20") {
    return disturbance("calm", seed, {
      sensors,
      payload: { atTime: 0, payload: PAYLOAD_MASS_20PCT },
    });
  }
  if (key === "mismatchMass") {
    return disturbance("calm", seed, {
      sensors,
      payload: { atTime: 0, payload: { mass: 0.15, position: [0, 0, 0] } },
    });
  }
  if (key === "constant5") {
    const calm = disturbance("calm", seed, { sensors });
    return {
      ...calm,
      wind: { ...calm.wind, enabled: true, mean: [5, 0, 0], gustSigma: [0, 0, 0] },
    };
  }
  const scenario: ScenarioId =
    key === "step" || key === "square" || key === "hoverJitter" ||
    key === "mismatchKTau"
      ? "calm"
      : key;
  return disturbance(scenario, seed, { sensors });
}

function overrideFor(key: CostKey): PlantOverride | null {
  if (key === "mismatchKTau") return { kTScale: 0.9, tauScale: 1.3 };
  return null;
}

/** One full-length rollout, with early stop on a hard failure. */
export function rollout(
  config: MultirotorConfig,
  params: ControllerParams,
  seed: number,
  key: CostKey,
  steps: number,
  extraOverride: PlantOverride | null = null,
  sensors = true,
): RolloutMetrics {
  const disturbanceParams = paramsFor(key, seed, sensors);
  const active =
    disturbanceParams !== null && isDisturbanceActive(disturbanceParams)
      ? disturbanceParams
      : null;
  let state: CoreState = {
    multirotor: initialState(config),
    controller: initialControllerState(),
    disturbance: active === null ? null : createDisturbanceState(active),
    disturbanceParams: active,
    diagnostics: null,
    trueConfig: config,
  };
  const plant = { config, params };
  const override = extraOverride ?? overrideFor(key);
  const allowContact = key === "mass20";
  const target = setpointFor(key, 0);
  const hoverInitial = key !== "step" && key !== "mismatchKTau" && key !== "mismatchMass" &&
    key !== "square";

  let sumSquared = 0;
  let earlySum = 0;
  let lateSum = 0;
  let count = 0;
  let earlyCount = 0;
  let lateCount = 0;
  let ise = 0;
  let lastOutside = -1;
  let reached = false;
  let overshoot = 0;
  let maxTilt = 0;
  let maxPositionM = 0;
  let maxSpeedMps = 0;
  let saturationSteps = 0;
  let sustainedSteps = 0;
  let sustainedSaturationSteps = 0;
  let contactSteps = 0;
  let jitterSum = 0;
  let jitterSumSquared = 0;
  let jitterCount = 0;
  let previousSpeeds: readonly number[] | null = null;
  let failure: string | null = null;
  let excess = 0;
  let nonfinite = false;

  for (let step = 0; step < steps; step++) {
    const time = step * DT;
    const desired = setpointFor(key, time);
    const result = simulateStep(
      state,
      plant,
      { position: desired, yaw: 0 },
      DT,
      override,
    );
    const position = result.state.body.position;
    const velocity = result.state.body.velocity;
    maxPositionM = Math.max(maxPositionM, Math.hypot(...position));
    maxSpeedMps = Math.max(maxSpeedMps, Math.hypot(...velocity));
    nonfinite = [...position, ...velocity].some((value) => !Number.isFinite(value));
    if (
      nonfinite ||
      position.some((value) => Math.abs(value) > DIVERGENCE_LIMIT) ||
      velocity.some((value) => Math.abs(value) > DIVERGENCE_LIMIT)
    ) {
      failure = nonfinite ? "non-finite state" : "divergent state";
      excess = (step + 1) / steps;
      break;
    }

    const error = Math.hypot(
      position[0] - desired[0],
      position[1] - desired[1],
      position[2] - desired[2],
    );
    const nextTime = (step + 1) * DT;
    if (nextTime >= 20 && nextTime <= 120) {
      sumSquared += error * error;
      count++;
      ise += error * error * DT;
    }
    if (nextTime >= 20 && nextTime <= 60) {
      earlySum += error * error;
      earlyCount++;
    }
    if (nextTime >= 90 && nextTime <= 120) {
      lateSum += error * error;
      lateCount++;
    }
    if (error > 0.1) lastOutside = step;
    if (error < 0.1) reached = true;
    if (!hoverInitial && reached && key !== "square") {
      // Step overshoot is measured along the initial->target direction,
      // starting from the 5 m hover, exactly as in the flight demo.
      const deltaX = target[0] - 0;
      const deltaY = target[1] - 0;
      const length = Math.hypot(deltaX, deltaY);
      if (length > 0) {
        overshoot = Math.max(
          overshoot,
          ((position[0] - target[0]) * deltaX + (position[1] - target[1]) * deltaY) / length,
        );
      }
    }
    maxTilt = Math.max(maxTilt, tiltOf(result.state));
    if (result.lastMix.saturated) {
      saturationSteps++;
      if (nextTime >= SUSTAINED_START_SECONDS) sustainedSaturationSteps++;
    }
    if (nextTime >= SUSTAINED_START_SECONDS) sustainedSteps++;
    if (result.ground?.contacted) contactSteps++;
    if (nextTime >= 20 && previousSpeeds !== null) {
      for (let motor = 0; motor < result.state.motorSpeeds.length; motor++) {
        const derivative = result.state.motorSpeeds[motor]! - previousSpeeds[motor]!;
        jitterSum += derivative;
        jitterSumSquared += derivative * derivative;
        jitterCount++;
      }
    }
    previousSpeeds = result.state.motorSpeeds;

    if (!allowContact && result.ground?.contacted) {
      failure = "ground contact in flight";
      excess = (step + 1) / steps;
      break;
    }
    // Sustained saturation is measured over the WHOLE t>=5 s window, not
    // stopped at the first crossing: early-stopping there would report a
    // fraction of ~1 from only a handful of post-startup steps and destroy the
    // severity gradient. NaN/Inf/divergence/contact still stop early above.
    state = {
      ...state,
      multirotor: result.state,
      controller: result.controller,
      disturbance: result.disturbance,
      diagnostics: result.diagnostics,
      trueConfig: result.trueConfig,
    };
  }

  if (failure === null && sustainedSteps > 0) {
    const fraction = sustainedSaturationSteps / sustainedSteps;
    if (fraction > SATURATION_THRESHOLD) {
      failure = "sustained mixer saturation above 20% (t>=5 s)";
      excess = fraction - SATURATION_THRESHOLD;
    }
  }
  const sustainedSaturation = sustainedSteps > 0 ? sustainedSaturationSteps / sustainedSteps : 0;
  const meanJitter = jitterCount > 0 ? jitterSum / jitterCount : 0;
  const jitterVariance =
    jitterCount > 1
      ? Math.max(0, jitterSumSquared / jitterCount - meanJitter * meanJitter)
      : 0;

  return {
    rms: count > 0 ? Math.sqrt(sumSquared / count) : 0,
    earlyRms: earlyCount > 0 ? Math.sqrt(earlySum / earlyCount) : 0,
    lateRms: lateCount > 0 ? Math.sqrt(lateSum / lateCount) : 0,
    overshoot,
    settling: (lastOutside + 1) * DT,
    ise,
    maxTilt,
    maxPositionM,
    maxSpeedMps,
    saturationSteps,
    contactSteps,
    jitter: Math.sqrt(jitterVariance),
    sustainedSaturation,
    failure,
    excess,
    nonfinite,
  };
}

function tiltOf(state: MultirotorState): number {
  const [x, y] = state.body.orientation;
  // Body +z rotated into world: R33 = 1 - 2(x^2 + y^2).
  const worldZ = 1 - 2 * (x! * x! + y! * y!);
  return Math.acos(Math.min(1, Math.max(-1, worldZ)));
}

export interface Baseline {
  readonly rms: number;
  readonly overshoot: number;
  readonly settling: number;
  readonly ise: number;
  readonly jitter: number;
  readonly contactSeconds: number;
}

function baselineOf(metrics: RolloutMetrics): Baseline {
  return {
    rms: metrics.rms,
    overshoot: metrics.overshoot,
    settling: metrics.settling,
    ise: metrics.ise,
    jitter: metrics.jitter,
    contactSeconds: metrics.contactSteps * DT,
  };
}

function normalizedValue(
  key: CostKey,
  metrics: RolloutMetrics,
  baseline: Baseline,
): number {
  switch (key) {
    // A floor-limited subterm is expressed as 1 + (value - baseline)/floor,
    // so the default still scores exactly 1.000 while changes remain comparable.
    case "step":
      return (
        0.5 *
          (1 + (metrics.overshoot - baseline.overshoot) /
            Math.max(baseline.overshoot, TERM_FLOORS.overshoot)) +
        0.3 *
          (1 + (metrics.settling - baseline.settling) /
            Math.max(baseline.settling, TERM_FLOORS.settling)) +
        0.2 *
          (1 + (metrics.ise - baseline.ise) / Math.max(baseline.ise, TERM_FLOORS.ise))
      );
    case "mass20":
      return (
        0.75 *
          (1 + (metrics.rms - baseline.rms) / Math.max(baseline.rms, TERM_FLOORS.rms)) +
        0.25 *
          (1 + (metrics.contactSteps * DT - baseline.contactSeconds) /
            Math.max(baseline.contactSeconds, TERM_FLOORS.contact))
      );
    case "hoverJitter":
      return 1 + (metrics.jitter - baseline.jitter) / Math.max(baseline.jitter, TERM_FLOORS.jitter);
    default:
      return 1 + (metrics.rms - baseline.rms) / Math.max(baseline.rms, TERM_FLOORS.rms);
  }
}

export type BaselineTable = Readonly<Record<string, Baseline>>;

export interface CostSuite {
  readonly weights: Readonly<Record<CostKey, number>>;
  evaluate(params: ControllerParams, seeds: readonly number[]): CostResult;
}

/**
 * Precompute baselines for the rotating seed subsets once, in the parent, so
 * worker processes never repeat the expensive default-gain rollouts.
 */
export function createBaselineTable(
  seedLists: readonly (readonly number[])[],
  config: MultirotorConfig = makeHex({ groundPlane: true }),
  baseParams: ControllerParams = defaultControllerParams(config),
  steps: number = FULL_STEPS,
): BaselineTable {
  const table: Record<string, Baseline> = {};
  for (const seeds of seedLists) {
    for (const key of COST_KEYS) {
      const metrics = rollout(config, baseParams, seeds[0]!, key, steps);
      if (metrics.failure !== null && key !== "mass20") {
        throw new RangeError(`default gains fail ${key}: ${metrics.failure}`);
      }
      table[`${seeds.join(",")}|${key}`] = baselineOf(metrics);
    }
  }
  return table;
}

/**
 * Build a suite. Baselines are computed lazily per seed subset and cached, so
 * the default gains score exactly 1.000 for every rotation subset. Tests may
 * pass a shorter `steps` to keep the fast suite fast; tuning always uses the
 * default FULL_STEPS = 120000.
 */
export function createCostSuite(
  config: MultirotorConfig = makeHex({ groundPlane: true }),
  baseParams: ControllerParams = defaultControllerParams(config),
  steps: number = FULL_STEPS,
  precomputed: BaselineTable | null = null,
): CostSuite {
  assertFixedDefaults(baseParams);
  const baselines = new Map<string, Map<CostKey, Baseline>>();
  if (precomputed !== null) {
    const ids = new Set(
      Object.keys(precomputed).map((key) => key.slice(0, key.lastIndexOf("|"))),
    );
    for (const id of ids) {
      const table = new Map<CostKey, Baseline>();
      for (const key of COST_KEYS) {
        const entry = precomputed[`${id}|${key}`];
        if (entry !== undefined) table.set(key, entry);
      }
      if (table.size === COST_KEYS.length) baselines.set(id, table);
    }
  }

  function baselineFor(seeds: readonly number[]): Map<CostKey, Baseline> {
    const id = seeds.join(",");
    const cached = baselines.get(id);
    if (cached !== undefined) return cached;
    const table = new Map<CostKey, Baseline>();
    const seed = seeds[0]!;
    for (const key of COST_KEYS) {
      const metrics = rollout(config, baseParams, seed, key, steps);
      if (metrics.failure !== null && key !== "mass20") {
        throw new RangeError(`default gains fail ${key}: ${metrics.failure}`);
      }
      table.set(key, baselineOf(metrics));
    }
    baselines.set(id, table);
    return table;
  }

  return {
    weights: TERM_WEIGHTS,
    evaluate(params, seeds) {
      if (seeds.length === 0) throw new RangeError("at least one seed is required");
      assertFixedDefaults(params);
      const baseline = baselineFor(seeds);
      const terms: TermBreakdown[] = [];
      let logTotal = 0;
      for (const key of COST_KEYS) {
        const metrics = rollout(config, params, seeds[0]!, key, steps);
        const failed =
          metrics.failure !== null &&
          !(key === "mass20" && metrics.failure === "ground contact in flight");
        // Graded finite penalty gives CMA-ES a slope out of failure regions.
        const value = failed
          ? gradedPenalty(metrics.excess, metrics.nonfinite)
          : normalizedValue(key, metrics, baseline.get(key)!);
        logTotal += TERM_WEIGHTS[key] * Math.log(value);
        terms.push({
          key,
          weight: TERM_WEIGHTS[key],
          value,
          normalized: value,
          reason: metrics.failure,
        });
      }
      const total = Math.exp(logTotal);
      return { total, terms, seeds: [...seeds] };
    },
  };
}
