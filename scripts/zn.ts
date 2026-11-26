/**
 * Ziegler-Nichols baselines, identified on this simulator at dt = 1 ms.
 * Two variants are reported, neither hand-tuned:
 *
 *  - ZN ultimate-gain: rate -> attitude -> position, each P-only until a
 *    sustained oscillation is found; classic ZN PID/PI rules applied.
 *  - ZN reaction-curve: the rate loop has no finite continuous-time ultimate
 *    gain (tau s^2 + s + kp is stable for all kp > 0), so an ultimate-gain
 *    sweep there finds only discrete-delay/saturation artifacts. The rate loop
 *    uses the open-loop tangent method (R, L, PID Kp = 1.2/(R L), Ti = 2L,
 *    Td = 0.5L); attitude and position use ultimate gain when finite.
 *
 * Run with `npm run zn`. Outputs out/zn_log.md and data/znGains.json.
 */

import { mkdirSync, writeFileSync } from "node:fs";

import {
  attitudeRateCommand,
} from "../src/core/control/attitude";
import {
  defaultControllerParams,
  initialControllerState,
  rateStep,
} from "../src/core/control/controller";
import type { ControllerParams } from "../src/core/control/types";
import { QUAT_IDENTITY, quatFromAxisAngle, quatToEulerZYX } from "../src/core/math/quat";
import { hoverMotorSpeeds, makeHex } from "../src/core/multirotor/config";
import { stepPlant } from "../src/core/simulation/loop";
import { measurementOf, simulateStep, type CoreState } from "../src/core/simulation/loop";
import { mix } from "../src/core/multirotor/mixer";
import {
  findUltimateGain,
  reactionCurve,
  znPi,
  znPid,
  znReactionPid,
  type UltimateGain,
} from "../src/core/tuning/zieglerNichols";

const DT = 0.001;
const STEPS = 6_000;
const config = makeHex({ groundPlane: false });
const base = defaultControllerParams(config);
const weight = config.mass * 9.81;
const inertiaX = config.inertia[0]!;
const rateStepScale = 0.2;

function initialCore(): CoreState {
  return {
    multirotor: {
      body: {
        position: [0, 0, 5],
        velocity: [0, 0, 0],
        orientation: QUAT_IDENTITY,
        angularVelocity: [0, 0, 0],
      },
      motorSpeeds: hoverMotorSpeeds(config),
    },
    controller: initialControllerState(),
    disturbance: null,
    disturbanceParams: null,
    diagnostics: null,
    trueConfig: config,
  };
}

function innerParams(rate: ControllerParams["rate"], attitude?: ControllerParams["attitude"]): ControllerParams {
  return { ...base, attitude: attitude ?? base.attitude, rate };
}

function rateResponse(kp: number, rate: ControllerParams["rate"]): number[] {
  const params = innerParams({ ...rate, gains: { kp, ki: 0, kd: 0 } });
  let core = initialCore();
  let state = initialControllerState().rate;
  const desired = [rateStepScale, 0, 0] as const;
  const samples: number[] = [];
  for (let step = 0; step < STEPS; step++) {
    const command = rateStep(state, desired, measurementOf(core.multirotor), params, DT);
    state = command.state;
    const speeds = mix(config, [weight, command.torque[0], command.torque[1], command.torque[2]]);
    const result = stepPlant(core.multirotor, config, speeds, DT);
    core = { ...core, multirotor: result.state };
    samples.push(result.state.body.angularVelocity[0]);
  }
  return samples.slice(1_000);
}

function attitudeResponse(
  tiltGain: number,
  rate: ControllerParams["rate"],
): number[] {
  const params = innerParams(rate, { ...base.attitude, tiltGain });
  const target = quatFromAxisAngle([1, 0, 0], (10 * Math.PI) / 180);
  let core = initialCore();
  let state = initialControllerState();
  const samples: number[] = [];
  for (let step = 0; step < STEPS; step++) {
    const desiredRate = attitudeRateCommand(core.multirotor.body.orientation, target, params.attitude);
    const command = rateStep(state.rate, desiredRate, measurementOf(core.multirotor), params, DT);
    state = { ...state, rate: command.state };
    const speeds = mix(config, [weight, command.torque[0], command.torque[1], command.torque[2]]);
    const result = stepPlant(core.multirotor, config, speeds, DT);
    core = { ...core, multirotor: result.state };
    samples.push(quatToEulerZYX(result.state.body.orientation)[0]);
  }
  return samples.slice(1_000);
}

function positionResponse(
  kp: number,
  params: ControllerParams,
): number[] {
  const trial: ControllerParams = {
    ...params,
    position: { ...params.position, gains: { kp, ki: 0, kd: 0 } },
  };
  let core = initialCore();
  const samples: number[] = [];
  for (let step = 0; step < STEPS; step++) {
    const result = simulateStep(core, { config, params: trial }, { position: [5, 0, 5], yaw: 0 }, DT);
    core = { ...core, multirotor: result.state, controller: result.controller, trueConfig: result.trueConfig };
    samples.push(result.state.body.position[0]);
  }
  return samples.slice(1_000);
}

function describe(gain: UltimateGain | null): string {
  if (gain === null) return "no finite ultimate gain within safety caps";
  const drift = (Math.max(...gain.halfPeriods) - Math.min(...gain.halfPeriods)) /
    (gain.halfPeriods.reduce((sum, value) => sum + value, 0) / gain.halfPeriods.length);
  return `Ku=${gain.kp.toFixed(3)}, Tu=${gain.tu.toFixed(4)} s, ` +
    `${gain.halfPeriods.length} half-periods, drift=${(drift * 100).toFixed(2)}%, ` +
    `amplitude=${gain.amplitude.toFixed(3)}, trend=${gain.trend.toFixed(3)}`;
}

interface Variant {
  readonly rate: ControllerParams["rate"];
  readonly attitude: ControllerParams["attitude"];
  readonly position: ControllerParams["position"];
  readonly log: string[];
}

function identify(useReactionRate: boolean): Variant {
  const log: string[] = [];
  let rate: ControllerParams["rate"];
  let rateLine: string;

  if (useReactionRate) {
    const alphaStep = 0.05;
    const speeds = mix(config, [weight, inertiaX * alphaStep, 0, 0]);
    let core = initialCore();
    const samples: number[] = [];
    for (let step = 0; step < STEPS; step++) {
      const result = stepPlant(core.multirotor, config, speeds, DT);
      core = { ...core, multirotor: result.state };
      samples.push(result.state.body.angularVelocity[0]);
    }
    const curve = reactionCurve(samples, DT, alphaStep);
    const gains = znReactionPid(curve.slope, curve.deadTime);
    rate = { ...base.rate, gains };
    rateLine = `reaction curve: R=${curve.slope.toFixed(4)} s^-2 per s^-2, ` +
      `L=${curve.deadTime.toFixed(4)} s; Kp=${gains.kp.toFixed(4)}, ` +
      `Ti=${(2 * curve.deadTime).toFixed(4)} s, Td=${(0.5 * curve.deadTime).toFixed(4)} s`;
  } else {
    const found = findUltimateGain((kp) => rateResponse(kp, base.rate), {
      minKp: 0.5,
      maxKp: 2_000,
      dt: DT,
      stepScale: rateStepScale,
      sweepSteps: 60,
    });
    const gains = found.gain === null ? null : znPid(found.gain.kp, found.gain.tu);
    rate = gains === null ? base.rate : { ...base.rate, gains };
    rateLine = `ultimate-gain sweep (expected discrete/saturation artifact): ${describe(found.gain)}` +
      (gains === null ? " -> kept default rate gains" : ` -> Kp=${gains.kp.toFixed(4)}`);
  }
  log.push(`rate loop | ${rateLine}`);

  const attitudeFound = findUltimateGain(
    (kp) => attitudeResponse(kp, rate),
    { minKp: 0.1, maxKp: 100, dt: DT, stepScale: (10 * Math.PI) / 180, sweepSteps: 48 },
  );
  const attitudeGains =
    attitudeFound.gain === null ? null : znPi(attitudeFound.gain.kp, attitudeFound.gain.tu);
  const attitude: ControllerParams["attitude"] = attitudeGains === null
    ? base.attitude
    : { ...base.attitude, tiltGain: attitudeGains.kp, yawGain: attitudeGains.kp };
  log.push(
    `attitude loop (ZN rate closed) | ${describe(attitudeFound.gain)}` +
      (attitudeGains === null
        ? " -> kept default attitude gains"
        : ` -> tilt=yaw Kp=${attitudeGains.kp.toFixed(4)}, Ti=${(0.83 * attitudeFound.gain!.tu).toFixed(4)} s`),
  );

  const inner: ControllerParams = { ...base, rate, attitude };
  const positionFound = findUltimateGain(
    (kp) => positionResponse(kp, inner),
    { minKp: 0.05, maxKp: 50, dt: DT, stepScale: 5, sweepSteps: 48 },
  );
  const positionGains =
    positionFound.gain === null ? null : znPi(positionFound.gain.kp, positionFound.gain.tu);
  const position: ControllerParams["position"] = positionGains === null
    ? base.position
    : {
        ...base.position,
        gains: { kp: positionGains.kp, ki: positionGains.ki, kd: base.position.gains.kd },
      };
  log.push(
    `position loop (both inner loops ZN) | ${describe(positionFound.gain)}` +
      (positionGains === null
        ? " -> kept default position gains"
        : ` -> Kp=${positionGains.kp.toFixed(4)}, Ki=${positionGains.ki.toFixed(4)}, kd unchanged`),
  );
  return { rate, attitude, position, log };
}

function main(): void {
  const ultimate = identify(false);
  const reaction = identify(true);
  const lines = [
    "# Ziegler-Nichols identification",
    "",
    `dt = ${DT} s, no disturbances during identification; scoring uses the noisy full-length suite.`,
    "Oscillation rule: >= 3 cycles, half-period drift < 5%, amplitude > 1% of the step;",
    "the ultimate gain is where peak trend crosses 1 (decaying below, growing above).",
    "",
    "## ZN ultimate-gain",
    ...ultimate.log,
    "",
    "## ZN reaction-curve",
    ...reaction.log,
    "",
    "The rate loop's continuous-time characteristic tau s^2 + s + kp is stable for all",
    "kp > 0, so any finite rate-loop Ku is a discrete-delay/saturation artifact; it is",
    "reported as such and not hand-corrected.",
    "",
  ];
  mkdirSync("out", { recursive: true });
  mkdirSync("data", { recursive: true });
  writeFileSync("out/zn_log.md", lines.join("\n"));
  writeFileSync(
    "data/znGains.json",
    JSON.stringify(
      {
        version: 1,
        dt: DT,
        ultimate: { position: ultimate.position, attitude: ultimate.attitude, rate: ultimate.rate },
        reaction: { position: reaction.position, attitude: reaction.attitude, rate: reaction.rate },
      },
      null,
      2,
    ),
  );
  console.log(lines.join("\n"));
  console.log("wrote out/zn_log.md and data/znGains.json");
}

main();
