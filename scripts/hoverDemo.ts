/**
 * Milestone 2 demo: three independent 10 s flights at 1 kHz, written to one
 * CSV trace. No controller, no graphics, no randomness — just the composed
 * M2 stack (mixer -> motor lag -> RK4 -> drag -> optional ground contact).
 *
 * Run with: npm run demo:hover
 * Output:   out/hover.csv (30003 rows, 41 columns)
 *
 * Phases (each from a fresh state):
 *   hover      — level hover at z = 5 m with hover thrust.
 *   takeoff    — rotors start stopped on the ground at z = 0, then 1.1x hover
 *                thrust; the plane holds the vehicle while T < mg, then it
 *                lifts off. Contact applies no torque, so all commands are
 *                collective (zero roll/pitch/yaw).
 *   roll-pulse — level hover, plus tau_x = 0.01 N·m for 100 steps (t = 1.0 to
 *                1.1 s). Open-loop by design: the residual roll integrates
 *                into lateral drift until a controller exists (M3).
 *
 * CSV conventions:
 * - One row per step, state at t with the command held over [t, t+dt).
 * - `phase` is a string; booleans are 0/1; all other values use toFixed(9).
 * - `*_cmd_*` is requested body wrench; `thrust_N`/`tau_*_Nm` is the achieved
 *   motor wrench, so allocator error and motor lag are visible in the trace.
 * - Ground columns describe the step that ENDED at this row (zero at t = 0).
 * - Euler angles are display-only; the quaternion is authoritative.
 * - The script throws (before writing) on any non-finite value, quaternion
 *   drift, motor-limit violation, broken contact postcondition, or failed
 *   phase sanity check. Determinism is checkable via the printed SHA-256.
 */

import { createHash } from "node:crypto";
import { mkdirSync, writeFileSync } from "node:fs";

import {
  kineticEnergy,
  potentialEnergy,
  totalEnergy,
} from "../src/core/energy";
import {
  QUAT_IDENTITY,
  quatToEulerZYX,
  type Quat,
} from "../src/core/math/quat";
import type { Vec3 } from "../src/core/math/vec3";
import {
  DEFAULT_ARM_LENGTH,
  hoverMotorSpeeds,
  makeHex,
  rigidBodyParamsOf,
  thrustToWeight,
  type MultirotorConfig,
} from "../src/core/multirotor/config";
import {
  motorWrench,
  type MultirotorState,
} from "../src/core/multirotor/dynamics";
import type { GroundContactResult } from "../src/core/multirotor/ground";
import { mixDetailed, type Wrench } from "../src/core/multirotor/mixer";
import { stepPlant } from "../src/core/simulation/loop";

const DT = 1e-3;
const STEPS = 10_000;
const HOVER_ALTITUDE = 5;
const TAKEOFF_THRUST_RATIO = 1.1;
const PULSE_START_STEP = 1000; // t = 1.0 s
const PULSE_END_STEP = 1100; // t = 1.1 s
const PULSE_TORQUE_X = 0.01;
const SETTLE_STEP = 2000; // t = 2.0 s; motor lag is 40 tau past spin-up
const OMEGA_TOLERANCE = 1e-9;
const OUTPUT_PATH = "out/hover.csv";

const CSV_HEADER = [
  "phase", "t_s", "phase_t_s",
  "x_m", "y_m", "z_m",
  "vx_ms", "vy_ms", "vz_ms",
  "qw", "qx", "qy", "qz",
  "roll_rad", "pitch_rad", "yaw_rad",
  "wx_rads", "wy_rads", "wz_rads",
  "thrust_cmd_N", "tau_x_cmd_Nm", "tau_y_cmd_Nm", "tau_z_cmd_Nm",
  "thrust_N", "tau_x_Nm", "tau_y_Nm", "tau_z_Nm",
  "saturated", "contact", "penetration_m",
  "normal_impulse_Ns", "friction_impulse_Ns",
  "ke_J", "pe_J", "total_J",
  "w0_rads", "w1_rads", "w2_rads", "w3_rads", "w4_rads", "w5_rads",
] as const;

type PhaseName = "hover" | "takeoff" | "roll-pulse";

interface CommandSet {
  readonly wrench: Wrench;
  readonly speeds: readonly number[];
}

interface PhaseSpec {
  readonly name: PhaseName;
  readonly config: MultirotorConfig;
  readonly initial: MultirotorState;
  readonly commandAt: (step: number) => CommandSet;
}

interface PhaseStats {
  readonly name: PhaseName;
  readonly steps: number;
  readonly finalPosition: Vec3;
  readonly finalVelocity: Vec3;
  readonly maxDrift: number;
  readonly maxQuatNormError: number;
  readonly maxMotorViolation: number;
  readonly contactSteps: number;
  readonly maxPenetration: number;
  readonly maxNormalImpulse: number;
  readonly liftoffTime: number | null;
  readonly peakAngularSpeed: number;
  readonly peakOmegaX: number;
  readonly peakRoll: number;
  readonly omegaRef: number | null;
  readonly maxOmegaXDeviation: number;
  readonly maxOmegaYZ: number;
}

function at<T>(values: readonly T[], index: number): T {
  const value = values[index];
  if (value === undefined) {
    throw new Error(`missing value at index ${index}`);
  }
  return value;
}

function csvRow(phase: PhaseName, values: readonly (number | boolean)[]): string {
  const cells = values.map((value) =>
    typeof value === "boolean" ? (value ? "1" : "0") : value.toFixed(9),
  );
  return [phase, ...cells].join(",");
}

function quatNorm(q: Quat): number {
  return Math.sqrt(
    at(q, 0) ** 2 + at(q, 1) ** 2 + at(q, 2) ** 2 + at(q, 3) ** 2,
  );
}

function initialState(z: number, motorSpeeds: readonly number[]): MultirotorState {
  return {
    body: {
      position: [0, 0, z],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    },
    motorSpeeds,
  };
}

/** Precompute a command set and prove it is feasible before the run starts. */
function makeCommandSet(config: MultirotorConfig, wrench: Wrench): CommandSet {
  const result = mixDetailed(config, wrench);
  if (result.saturated) {
    throw new Error(`command set is saturated: ${JSON.stringify(wrench)}`);
  }
  const requested = [wrench[0], wrench[1], wrench[2], wrench[3]];
  for (const [index, achieved] of result.achieved.entries()) {
    const expected = at(requested, index);
    if (Math.abs(achieved - expected) > 1e-9) {
      throw new Error(
        `mixer round-trip failed on axis ${index}: ${achieved} vs ${expected}`,
      );
    }
  }
  return { wrench, speeds: result.motorSpeeds };
}

function assertFiniteState(state: MultirotorState, config: MultirotorConfig): void {
  const fields: ReadonlyArray<readonly [string, readonly number[]]> = [
    ["position", state.body.position],
    ["velocity", state.body.velocity],
    ["orientation", state.body.orientation],
    ["angularVelocity", state.body.angularVelocity],
    ["motorSpeeds", state.motorSpeeds],
  ];
  for (const [name, values] of fields) {
    for (const [index, value] of values.entries()) {
      if (!Number.isFinite(value)) {
        throw new Error(`non-finite ${name}[${index}]`);
      }
    }
  }
  if (state.motorSpeeds.length !== config.motors.length) {
    throw new Error("motor speed count does not match the config");
  }
}

/** Fail loudly on any state that is physically or numerically broken. */
function validateStep(
  state: MultirotorState,
  config: MultirotorConfig,
  ground: GroundContactResult | null,
): void {
  assertFiniteState(state, config);
  if (Math.abs(quatNorm(state.body.orientation) - 1) > 1e-9) {
    throw new Error("quaternion drifted away from unit length");
  }
  for (const [index, speed] of state.motorSpeeds.entries()) {
    const motor = at(config.motors, index);
    if (speed < motor.minOmega || speed > motor.maxOmega) {
      throw new Error(`motor ${index} speed ${speed} is outside its limits`);
    }
  }
  if (!config.groundPlane && ground !== null) {
    throw new Error("ground result produced with the plane disabled");
  }
  if (ground?.contacted) {
    if (state.body.position[2] !== 0) {
      throw new Error("contact left the body off the plane");
    }
    if (state.body.velocity[2] < 0) {
      throw new Error("contact left the body with inward velocity");
    }
  }
}

function runPhase(
  spec: PhaseSpec,
  globalStart: number,
  rows: string[],
): PhaseStats {
  let state = spec.initial;
  let lastGround: GroundContactResult | null = null;
  const startPosition = spec.initial.body.position;

  let maxDrift = 0;
  let maxQuatNormError = 0;
  let maxMotorViolation = 0;
  let contactSteps = 0;
  let maxPenetration = 0;
  let maxNormalImpulse = 0;
  let liftoffTime: number | null = null;
  let peakAngularSpeed = 0;
  let peakOmegaX = 0;
  let peakRoll = 0;
  let omegaRef: number | null = null;
  let maxOmegaXDeviation = 0;
  let maxOmegaYZ = 0;

  for (let step = 0; step <= STEPS; step++) {
    const phaseTime = step * DT;
    const set = spec.commandAt(step);

    const [roll, pitch, yaw] = quatToEulerZYX(state.body.orientation);
    const wrench = motorWrench(state, spec.config);
    const params = rigidBodyParamsOf(spec.config);
    rows.push(csvRow(spec.name, [
      globalStart + phaseTime,
      phaseTime,
      ...state.body.position,
      ...state.body.velocity,
      ...state.body.orientation,
      roll,
      pitch,
      yaw,
      ...state.body.angularVelocity,
      ...set.wrench,
      wrench.forceBody[2],
      ...wrench.torqueBody,
      false, // command sets were proven unsaturated before the run
      lastGround?.contacted ?? false,
      lastGround?.penetration ?? 0,
      lastGround?.normalImpulse ?? 0,
      lastGround?.frictionImpulse ?? 0,
      kineticEnergy(state.body, params),
      potentialEnergy(state.body, params),
      totalEnergy(state.body, params),
      ...state.motorSpeeds,
    ]));

    maxDrift = Math.max(maxDrift, Math.hypot(
      state.body.position[0] - startPosition[0],
      state.body.position[1] - startPosition[1],
      state.body.position[2] - startPosition[2],
    ));
    maxQuatNormError = Math.max(
      maxQuatNormError,
      Math.abs(quatNorm(state.body.orientation) - 1),
    );
    for (const [index, speed] of state.motorSpeeds.entries()) {
      const motor = at(spec.config.motors, index);
      maxMotorViolation = Math.max(
        maxMotorViolation,
        speed - motor.maxOmega,
        motor.minOmega - speed,
      );
    }
    peakAngularSpeed = Math.max(
      peakAngularSpeed,
      Math.hypot(...state.body.angularVelocity),
    );
    const omegaX = state.body.angularVelocity[0];
    peakOmegaX = Math.max(peakOmegaX, Math.abs(omegaX));
    peakRoll = Math.max(peakRoll, Math.abs(roll));

    if (step >= SETTLE_STEP) {
      // Motor lag is asymptotic; at 2.0 s (40 tau from spin-up, 18 tau after
      // the pulse) the residual torque integrates to < 1e-9 rad/s.
      const reference: number = omegaRef ?? omegaX;
      omegaRef = reference;
      maxOmegaXDeviation = Math.max(maxOmegaXDeviation, Math.abs(omegaX - reference));
      maxOmegaYZ = Math.max(
        maxOmegaYZ,
        Math.abs(state.body.angularVelocity[1]),
        Math.abs(state.body.angularVelocity[2]),
      );
    }

    if (step === STEPS) break;

    const result = stepPlant(state, spec.config, set.speeds, DT);
    validateStep(result.state, spec.config, result.ground);
    if (result.ground?.contacted) {
      contactSteps++;
      maxPenetration = Math.max(maxPenetration, result.ground.penetration);
      maxNormalImpulse = Math.max(maxNormalImpulse, result.ground.normalImpulse);
    } else if (
      contactSteps > 0 &&
      liftoffTime === null &&
      result.state.body.position[2] > 0
    ) {
      liftoffTime = (step + 1) * DT;
    }
    state = result.state;
    lastGround = result.ground;
  }

  return {
    name: spec.name,
    steps: STEPS,
    finalPosition: state.body.position,
    finalVelocity: state.body.velocity,
    maxDrift,
    maxQuatNormError,
    maxMotorViolation,
    contactSteps,
    maxPenetration,
    maxNormalImpulse,
    liftoffTime,
    peakAngularSpeed,
    peakOmegaX,
    peakRoll,
    omegaRef,
    maxOmegaXDeviation,
    maxOmegaYZ,
  };
}

function main(): void {
  const config = makeHex();
  const groundConfig = makeHex({ groundPlane: true });
  const weight = config.mass * 9.81;
  const stopped = config.motors.map(() => 0);

  const hoverSet = makeCommandSet(config, [weight, 0, 0, 0]);
  const takeoffSet = makeCommandSet(groundConfig, [
    TAKEOFF_THRUST_RATIO * weight,
    0,
    0,
    0,
  ]);
  const pulseSet = makeCommandSet(config, [weight, PULSE_TORQUE_X, 0, 0]);

  const specs: readonly PhaseSpec[] = [
    {
      name: "hover",
      config,
      initial: initialState(HOVER_ALTITUDE, hoverSet.speeds),
      commandAt: () => hoverSet,
    },
    {
      name: "takeoff",
      config: groundConfig,
      initial: initialState(0, stopped),
      commandAt: () => takeoffSet,
    },
    {
      name: "roll-pulse",
      config,
      initial: initialState(HOVER_ALTITUDE, hoverSet.speeds),
      commandAt: (step) =>
        step >= PULSE_START_STEP && step < PULSE_END_STEP ? pulseSet : hoverSet,
    },
  ];

  const rows: string[] = [];
  const stats: PhaseStats[] = [];
  for (const [index, spec] of specs.entries()) {
    stats.push(runPhase(spec, index * STEPS * DT, rows));
  }
  const hover = at(stats, 0);
  const takeoff = at(stats, 1);
  const pulse = at(stats, 2);

  const tau = at(config.motors, 0).motorTimeConstant;
  const analyticLiftoff =
    -tau * Math.log(1 - 1 / Math.sqrt(TAKEOFF_THRUST_RATIO));
  if (takeoff.liftoffTime === null) {
    throw new Error("takeoff never lifted off");
  }
  if (Math.abs(takeoff.liftoffTime - analyticLiftoff) >= 5 * DT) {
    throw new Error(
      `liftoff ${takeoff.liftoffTime} s vs analytic ${analyticLiftoff} s`,
    );
  }
  if (hover.maxDrift >= 1e-6) {
    throw new Error(`hover drift ${hover.maxDrift} m is too large`);
  }
  if (takeoff.contactSteps === 0 || takeoff.finalPosition[2] <= 1 || takeoff.finalVelocity[2] <= 0) {
    throw new Error("takeoff did not rest on and leave the plane");
  }
  if (pulse.peakOmegaX <= 0 || pulse.peakRoll <= 0) {
    throw new Error("roll pulse had no effect");
  }
  if (pulse.omegaRef === null) {
    throw new Error("settle window was never sampled");
  }
  if (pulse.maxOmegaXDeviation >= OMEGA_TOLERANCE || pulse.maxOmegaYZ >= OMEGA_TOLERANCE) {
    throw new Error(
      `angular rates not constant after settling: dx=${pulse.maxOmegaXDeviation}, yz=${pulse.maxOmegaYZ}`,
    );
  }
  const maxQuatError = Math.max(...stats.map((s) => s.maxQuatNormError));
  const maxViolation = Math.max(...stats.map((s) => s.maxMotorViolation));
  if (maxQuatError >= 1e-9 || maxViolation > 0) {
    throw new Error("quaternion or motor-limit invariant failed");
  }

  const content = `${[CSV_HEADER.join(","), ...rows].join("\n")}\n`;
  const hash = createHash("sha256").update(content, "utf8").digest("hex");
  mkdirSync("out", { recursive: true });
  writeFileSync(OUTPUT_PATH, content, "utf8");

  const inertia = config.inertia;
  const drag = config.drag;
  console.log("multirotor hover demo — 3 phases at 1000 Hz\n");
  console.log(
    `airframe : hex, m = ${config.mass.toFixed(3)} kg, arm = ${DEFAULT_ARM_LENGTH.toFixed(3)} m`,
  );
  console.log(
    `inertia  : Ixx = Iyy = ${at(inertia, 0).toFixed(6)}, Izz = ${at(inertia, 8).toFixed(6)} kg·m²`,
  );
  console.log(
    `motors   : kT = ${at(config.motors, 0).kT.toExponential(1)}, kQ = ${at(config.motors, 0).kQ.toExponential(1)}, ω ∈ [0, ${at(config.motors, 0).maxOmega}] rad/s, τ = ${tau.toFixed(3)} s`,
  );
  console.log(
    `drag     : linear [${drag.linear.join(", ")}], quadratic [${drag.quadratic.join(", ")}]`,
  );
  console.log(
    `hover    : ω = ${at(hoverSet.speeds, 0).toFixed(3)} rad/s (factory ${at(hoverMotorSpeeds(config), 0).toFixed(3)}), T = ${weight.toFixed(3)} N, T/W = ${thrustToWeight(config).toFixed(3)}\n`,
  );

  const duration = (STEPS * DT).toFixed(1);
  console.log(
    `phase hover      : ${STEPS} steps (${duration} s) | drift ${hover.maxDrift.toExponential(3)} m | final z ${hover.finalPosition[2].toFixed(3)} m`,
  );
  console.log(
    `phase takeoff    : ${STEPS} steps (${duration} s) | ${takeoff.contactSteps} contact steps | max pen ${takeoff.maxPenetration.toExponential(3)} m | max Jn ${takeoff.maxNormalImpulse.toExponential(3)} N·s | liftoff ${takeoff.liftoffTime.toFixed(3)} s (analytic ${analyticLiftoff.toFixed(3)} s) | final z ${takeoff.finalPosition[2].toFixed(3)} m, vz ${takeoff.finalVelocity[2].toFixed(3)} m/s`,
  );
  console.log(
    `phase roll-pulse : ${STEPS} steps (${duration} s) | peak |ωx| ${pulse.peakOmegaX.toFixed(4)} rad/s | peak roll ${pulse.peakRoll.toFixed(4)} rad | final y ${pulse.finalPosition[1].toFixed(3)} m`,
  );
  console.log(
    `                   ωx invariance (step >= ${SETTLE_STEP}): ref ${pulse.omegaRef.toFixed(12)} rad/s | max Δωx ${pulse.maxOmegaXDeviation.toExponential(3)} rad/s | max |ωy|,|ωz| ${pulse.maxOmegaYZ.toExponential(3)} rad/s`,
  );
  console.log(
    `checks           : max |q| error ${maxQuatError.toExponential(3)}, motor violations ${maxViolation}, all values finite`,
  );
  console.log(
    `wrote ${rows.length} rows × ${CSV_HEADER.length} columns to ${OUTPUT_PATH} (${(Buffer.byteLength(content, "utf8") / 1e6).toFixed(2)} MB)`,
  );
  console.log(`sha256           : ${hash}`);
  console.log(
    "note             : phase C is open-loop; without a controller the residual roll integrates into lateral drift (M3 closes the loop).",
  );
}

main();
