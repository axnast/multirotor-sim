/**
 * Milestone 1 demo: drop a tumbling rigid box for 10 s at 1 kHz and write a
 * CSV trace. No graphics, no motors, no controller.
 *
 * Run with: npm run demo
 * Output:   out/drop-box.csv
 *
 * Columns are SI units: time (s), position (m), Euler angles (rad, Z-Y-X),
 * kinetic/potential/total energy (J). Euler angles are for display only; the
 * simulation itself uses the unit quaternion. There is no ground contact, so
 * the box keeps falling below z = 0.
 */

import { mkdirSync, writeFileSync } from "node:fs";

import {
  kineticEnergy,
  potentialEnergy,
  totalEnergy,
} from "../src/core/energy";
import { step } from "../src/core/integrator";
import { mat3, type Mat3 } from "../src/core/math/mat3";
import { QUAT_IDENTITY, quatToEulerZYX } from "../src/core/math/quat";
import type { Vec3 } from "../src/core/math/vec3";
import {
  ZERO_INPUTS,
  type RigidBodyParams,
  type RigidBodyState,
} from "../src/core/rigidBody";

/** Inertia of a uniform box about its centre of mass: Ixx = m(sy²+sz²)/12. */
function boxInertia(mass: number, size: Vec3): Mat3 {
  const [sx, sy, sz] = size;
  return mat3(
    (mass / 12) * (sy * sy + sz * sz), 0, 0,
    0, (mass / 12) * (sx * sx + sz * sz), 0,
    0, 0, (mass / 12) * (sx * sx + sy * sy),
  );
}

function csvRow(values: readonly number[]): string {
  return values.map((value) => value.toFixed(9)).join(",");
}

const mass = 1.2;
const size: Vec3 = [0.3, 0.2, 0.1];
const params: RigidBodyParams = {
  mass,
  inertia: boxInertia(mass, size),
};

let state: RigidBodyState = {
  position: [0, 0, 10],
  velocity: [0, 0, 0],
  orientation: QUAT_IDENTITY,
  angularVelocity: [4, 2, 3],
};

const dt = 1e-3;
const steps = 10_000;
const initialTotalEnergy = totalEnergy(state, params);

const rows: string[] = [
  "t_s,x_m,y_m,z_m,roll_rad,pitch_rad,yaw_rad,ke_J,pe_J,total_J",
];
let maxEnergyDrift = 0;

for (let i = 0; i <= steps; i++) {
  const [roll, pitch, yaw] = quatToEulerZYX(state.orientation);
  rows.push(
    csvRow([
      i * dt,
      state.position[0],
      state.position[1],
      state.position[2],
      roll,
      pitch,
      yaw,
      kineticEnergy(state, params),
      potentialEnergy(state, params),
      totalEnergy(state, params),
    ]),
  );

  maxEnergyDrift = Math.max(
    maxEnergyDrift,
    Math.abs(totalEnergy(state, params) - initialTotalEnergy) /
      Math.abs(initialTotalEnergy),
  );

  if (i < steps) {
    state = step(state, params, ZERO_INPUTS, dt);
  }
}

mkdirSync("out", { recursive: true });
const outputPath = "out/drop-box.csv";
writeFileSync(outputPath, `${rows.join("\n")}\n`, "utf8");

console.log(`Simulation: ${steps} steps at ${1 / dt} Hz (${steps * dt} s)`);
console.log(`Final position: [${state.position.map((v) => v.toFixed(3)).join(", ")}] m`);
console.log(`Max relative energy drift: ${maxEnergyDrift.toExponential(3)}`);
console.log(`Wrote ${rows.length - 1} rows to ${outputPath}`);
