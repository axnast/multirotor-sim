/**
 * Control allocation: map a desired wrench to motor speed commands.
 *
 * The allocation matrix A is 4×N and maps rotor thrusts to
 * `[T, τx, τy, τz]` in the body frame:
 *
 *   row 0:  1              (T   = Σ f_i)
 *   row 1:  y_i            (τx  = Σ y_i f_i)
 *   row 2: -x_i            (τy  = Σ -x_i f_i)
 *   row 3: -spin_i kQ_i/kT_i  (τz = Σ -spin_i kQ_i ω_i²)
 *
 * The mixer is the pseudo-inverse M = Aᵀ(AAᵀ)⁻¹, so A·M = I₄ when A has full
 * row rank. For N = 4 it coincides with A⁻¹. AAᵀ is inverted here with a
 * pure 4×4 Gauss–Jordan elimination using scaled partial pivoting; a singular
 * matrix throws a RangeError instead of producing NaNs.
 *
 * Saturation policy (in this order):
 * 1. Clamp the requested total thrust into [Σ kT minΩ², Σ kT maxΩ²] BEFORE
 *    solving for torque.
 * 2. With torque fixed, every motor thrust is affine in T: f_i(T) = q_i T +
 *    p_i. Intersect all feasible T intervals and use the value nearest the
 *    clamped request, so the requested torque is preserved whenever possible.
 * 3. If no T works, scale yaw τz down to the largest feasible value; roll and
 *    pitch stay intact.
 * 4. Only if roll/pitch are still infeasible, scale them down too.
 * 5. Clamp every thrust to its motor's limits, convert with ω = sqrt(f/kT)
 *    (negative thrust clamps to zero), and clamp the resulting speeds.
 *
 * This is the specified minimum-norm allocator, not a general box-QP. For
 * unequal per-motor limits it can leave thrust authority unused; that is a
 * documented limitation, pinned by a test. A capability-weighted optimizer is
 * deliberately out of scope for Milestone 2.
 */

import {
  validateConfig,
  type MotorConfig,
  type MultirotorConfig,
} from "./config";
import { clampMotorCommand, motorThrust } from "./motors";

/** Wrench in the body frame: total thrust (N) and torque (N·m), Z-Y-X axes. */
export type Wrench = readonly [
  thrust: number,
  torqueX: number,
  torqueY: number,
  torqueZ: number,
];

/** Full result of a mix, including what was actually achieved. */
export interface MixResult {
  /** Commands in rad/s, in the same order as `config.motors`. */
  readonly motorSpeeds: number[];
  /** Achieved rotor thrusts (N), after clamping and speed quantization. */
  readonly motorThrusts: number[];
  /** Achieved wrench, i.e. A · motorThrusts. */
  readonly achieved: Wrench;
  /** True when the requested thrust or torque had to be modified. */
  readonly saturated: boolean;
}

const SINGULAR_RATIO_TOL = 1e-12;
const FEASIBILITY_EPS = 1e-12;
const BISECTION_ITERATIONS = 60;

interface Interval {
  readonly lo: number;
  readonly hi: number;
}

function contains(interval: Interval, thrust: number): boolean {
  return (
    thrust >= interval.lo - FEASIBILITY_EPS &&
    thrust <= interval.hi + FEASIBILITY_EPS
  );
}

function at<T>(values: readonly T[], index: number): T {
  const value = values[index];
  if (value === undefined) {
    throw new RangeError(`missing value at index ${index}`);
  }
  return value;
}

function matrixCell(
  matrix: readonly (readonly number[])[],
  row: number,
  col: number,
): number {
  const values = matrix[row];
  if (values === undefined) {
    throw new RangeError(`missing matrix row ${row}`);
  }
  return at(values, col);
}

function cell(matrix: readonly number[], row: number, col: number): number {
  return at(matrix, row * 4 + col);
}

function assertValidWrench(wrench: Wrench): void {
  if (wrench.length !== 4) {
    throw new RangeError("wrench must have exactly four components");
  }
  for (let index = 0; index < 4; index++) {
    const value = at(wrench, index);
    if (!Number.isFinite(value)) {
      throw new RangeError(`wrench[${index}] must be a finite number`);
    }
  }
}

/** Unvalidated 4×N allocation matrix. */
function buildAllocationMatrix(config: MultirotorConfig): number[][] {
  const row0: number[] = [];
  const row1: number[] = [];
  const row2: number[] = [];
  const row3: number[] = [];

  for (const motor of config.motors) {
    row0.push(1);
    row1.push(motor.position[1]);
    row2.push(-motor.position[0]);
    row3.push(-motor.spin * (motor.kQ / motor.kT));
  }

  return [row0, row1, row2, row3];
}

/** Allocation matrix A, 4 rows of length N. */
export function allocationMatrix(config: MultirotorConfig): number[][] {
  validateConfig(config);
  return buildAllocationMatrix(config);
}

/** Pure 4×4 inverse via Gauss–Jordan with scaled partial pivoting. */
function invert4(matrix: readonly number[]): number[] {
  const a = new Array<number>(16);
  const inverse = [
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1,
  ];
  const scale = [0, 0, 0, 0];

  for (let row = 0; row < 4; row++) {
    let largest = 0;
    for (let col = 0; col < 4; col++) {
      const value = cell(matrix, row, col);
      a[row * 4 + col] = value;
      largest = Math.max(largest, Math.abs(value));
    }
    scale[row] = largest;
  }

  for (let col = 0; col < 4; col++) {
    let bestRow = -1;
    let bestRatio = -1;
    for (let row = col; row < 4; row++) {
      const rowScale = at(scale, row);
      const ratio = rowScale === 0 ? 0 : Math.abs(cell(a, row, col)) / rowScale;
      if (ratio > bestRatio) {
        bestRatio = ratio;
        bestRow = row;
      }
    }
    if (bestRow < 0 || bestRatio <= SINGULAR_RATIO_TOL) {
      throw new RangeError("allocation matrix is singular and cannot be inverted");
    }

    if (bestRow !== col) {
      for (let j = 0; j < 4; j++) {
        const aSwap = cell(a, bestRow, j);
        a[bestRow * 4 + j] = cell(a, col, j);
        a[col * 4 + j] = aSwap;
        const iSwap = cell(inverse, bestRow, j);
        inverse[bestRow * 4 + j] = cell(inverse, col, j);
        inverse[col * 4 + j] = iSwap;
      }
      const scaleSwap = at(scale, bestRow);
      scale[bestRow] = at(scale, col);
      scale[col] = scaleSwap;
    }

    const pivot = cell(a, col, col);
    if (Math.abs(pivot) <= Number.EPSILON * at(scale, col)) {
      throw new RangeError("allocation matrix is singular and cannot be inverted");
    }
    for (let j = 0; j < 4; j++) {
      a[col * 4 + j] = cell(a, col, j) / pivot;
      inverse[col * 4 + j] = cell(inverse, col, j) / pivot;
    }
    for (let row = 0; row < 4; row++) {
      if (row === col) continue;
      const factor = cell(a, row, col);
      if (factor === 0) continue;
      for (let j = 0; j < 4; j++) {
        a[row * 4 + j] = cell(a, row, j) - factor * cell(a, col, j);
        inverse[row * 4 + j] = cell(inverse, row, j) - factor * cell(inverse, col, j);
      }
    }
  }

  return inverse;
}

/** Unvalidated mixer matrix M = Aᵀ(AAᵀ)⁻¹, N rows of 4 columns. */
function buildMixerMatrix(config: MultirotorConfig): number[][] {
  const a = buildAllocationMatrix(config);
  const count = config.motors.length;
  const gram = new Array<number>(16).fill(0);

  for (let row = 0; row < 4; row++) {
    for (let col = 0; col < 4; col++) {
      let sum = 0;
      for (let index = 0; index < count; index++) {
        sum += matrixCell(a, row, index) * matrixCell(a, col, index);
      }
      gram[row * 4 + col] = sum;
    }
  }

  const inverse = invert4(gram);
  const mixer: number[][] = [];
  for (let index = 0; index < count; index++) {
    const row: number[] = [];
    for (let col = 0; col < 4; col++) {
      let sum = 0;
      for (let k = 0; k < 4; k++) {
        sum += matrixCell(a, k, index) * cell(inverse, k, col);
      }
      row.push(sum);
    }
    mixer.push(row);
  }
  return mixer;
}

/** Mixer matrix A⁺ = Aᵀ(AAᵀ)⁻¹, N×4. */
export function mixerMatrix(config: MultirotorConfig): number[][] {
  validateConfig(config);
  return buildMixerMatrix(config);
}

function thrustLimits(motor: MotorConfig): { readonly min: number; readonly max: number } {
  return {
    min: motorThrust(motor.minOmega, motor),
    max: motorThrust(motor.maxOmega, motor),
  };
}

/**
 * Total-thrust interval for which the minimum-norm family is inside every
 * motor's limits at the requested torque. `null` means no T works.
 */
function feasibleInterval(
  config: MultirotorConfig,
  mixer: readonly (readonly number[])[],
  torqueX: number,
  torqueY: number,
  torqueZ: number,
): Interval | null {
  let maxQ = 0;
  for (const row of mixer) {
    maxQ = Math.max(maxQ, Math.abs(at(row, 0)));
  }
  const quantumEps = FEASIBILITY_EPS * Math.max(1, maxQ);

  let lo = -Infinity;
  let hi = Infinity;
  for (const [index, motor] of config.motors.entries()) {
    const q = matrixCell(mixer, index, 0);
    const p =
      matrixCell(mixer, index, 1) * torqueX +
      matrixCell(mixer, index, 2) * torqueY +
      matrixCell(mixer, index, 3) * torqueZ;
    const limits = thrustLimits(motor);

    if (Math.abs(q) <= quantumEps) {
      const tolerance = FEASIBILITY_EPS * Math.max(1, limits.max);
      if (p < limits.min - tolerance || p > limits.max + tolerance) {
        return null;
      }
    } else {
      const first = (limits.min - p) / q;
      const second = (limits.max - p) / q;
      lo = Math.max(lo, Math.min(first, second));
      hi = Math.min(hi, Math.max(first, second));
      if (lo > hi) return null;
    }
  }
  return { lo, hi };
}

/** Largest α in [0, 1] for which the predicate holds; fixed iterations. */
function largestFeasibleScale(feasible: (alpha: number) => boolean): number {
  if (feasible(1)) return 1;
  if (!feasible(0)) return 0;
  let lo = 0;
  let hi = 1;
  for (let iteration = 0; iteration < BISECTION_ITERATIONS; iteration++) {
    const mid = (lo + hi) / 2;
    if (feasible(mid)) lo = mid;
    else hi = mid;
  }
  return lo;
}

/** Result of applying A to an array of per-motor thrusts. */
function applyAllocation(
  config: MultirotorConfig,
  thrusts: readonly number[],
): Wrench {
  const rows = buildAllocationMatrix(config);
  const out = [0, 0, 0, 0];
  for (let row = 0; row < 4; row++) {
    let sum = 0;
    for (let index = 0; index < thrusts.length; index++) {
      sum += matrixCell(rows, row, index) * at(thrusts, index);
    }
    out[row] = sum;
  }
  return [at(out, 0), at(out, 1), at(out, 2), at(out, 3)];
}

/** Mix with full diagnostics: achieved thrusts, achieved wrench, saturation flag. */
export function mixDetailed(config: MultirotorConfig, wrench: Wrench): MixResult {
  validateConfig(config);
  assertValidWrench(wrench);

  const requestedThrust = wrench[0];
  const torqueX = wrench[1];
  const torqueY = wrench[2];
  const torqueZ = wrench[3];
  const count = config.motors.length;
  const mixer = buildMixerMatrix(config);

  const limits = config.motors.map(thrustLimits);
  const thrustMin = limits.reduce((sum, limit) => sum + limit.min, 0);
  const thrustMax = limits.reduce((sum, limit) => sum + limit.max, 0);
  const clampedThrust = Math.min(thrustMax, Math.max(thrustMin, requestedThrust));

  let alphaZ = 1;
  let alphaXY = 1;
  let usedFallback = false;
  let interval = feasibleInterval(config, mixer, torqueX, torqueY, torqueZ);

  const feasibleAtRequestedThrust = (
    x: number,
    y: number,
    z: number,
  ): boolean => {
    const candidate = feasibleInterval(config, mixer, x, y, z);
    return candidate !== null && contains(candidate, clampedThrust);
  };

  if (interval === null) {
    // Yaw is sacrificed before roll/pitch. Once torque must be reduced, the
    // clamped thrust is held fixed; only torque is scaled down.
    alphaZ = largestFeasibleScale((alpha) =>
      feasibleAtRequestedThrust(torqueX, torqueY, alpha * torqueZ),
    );
    if (!feasibleAtRequestedThrust(torqueX, torqueY, alphaZ * torqueZ)) {
      // Full roll/pitch was already infeasible; scale those next.
      alphaXY = largestFeasibleScale((alpha) =>
        feasibleAtRequestedThrust(
          alpha * torqueX,
          alpha * torqueY,
          alphaZ * torqueZ,
        ),
      );
    }
    interval = feasibleInterval(
      config,
      mixer,
      alphaXY * torqueX,
      alphaXY * torqueY,
      alphaZ * torqueZ,
    );
  }

  let thrusts: number[];
  let thrustMoved = false;
  if (interval === null) {
    // Pathological heterogeneous bounds: no torque-zero solution exists in the
    // minimum-norm family. Fall back to a clamped, always-feasible split.
    usedFallback = true;
    const uniform = clampedThrust / count;
    thrusts = config.motors.map((_, index) => {
      const limit = at(limits, index);
      return Math.min(limit.max, Math.max(limit.min, uniform));
    });
  } else {
    const total = Math.min(interval.hi, Math.max(interval.lo, clampedThrust));
    thrustMoved = total !== clampedThrust;
    const appliedX = alphaXY * torqueX;
    const appliedY = alphaXY * torqueY;
    const appliedZ = alphaZ * torqueZ;
    thrusts = config.motors.map((_, index) => {
      const q = matrixCell(mixer, index, 0);
      const p =
        matrixCell(mixer, index, 1) * appliedX +
        matrixCell(mixer, index, 2) * appliedY +
        matrixCell(mixer, index, 3) * appliedZ;
      const limit = at(limits, index);
      return Math.min(limit.max, Math.max(limit.min, q * total + p));
    });
  }

  const motorSpeeds = config.motors.map((motor, index) => {
    const thrust = at(thrusts, index);
    return clampMotorCommand(Math.sqrt(Math.max(0, thrust) / motor.kT), motor);
  });
  const actualThrusts = config.motors.map((motor, index) =>
    motorThrust(at(motorSpeeds, index), motor),
  );
  const achieved = applyAllocation(config, actualThrusts);

  const saturated =
    clampedThrust !== requestedThrust ||
    thrustMoved ||
    alphaZ < 1 ||
    alphaXY < 1 ||
    usedFallback;

  return {
    motorSpeeds,
    motorThrusts: actualThrusts,
    achieved,
    saturated,
  };
}

/** Mix to motor speed commands (rad/s). Thin wrapper over `mixDetailed`. */
export function mix(config: MultirotorConfig, wrench: Wrench): number[] {
  return [...mixDetailed(config, wrench).motorSpeeds];
}
