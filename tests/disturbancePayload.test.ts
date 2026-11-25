import { describe, expect, it } from "vitest";

import { defaultControllerParams } from "../src/core/control/controller";
import type { ControllerParams } from "../src/core/control/types";
import {
  applyPayload,
  PAYLOAD_2CM,
  PAYLOAD_LARGE_STRESS,
  PAYLOAD_SMALL,
  payloadCenterOfMass,
  parallelAxisTensor,
} from "../src/core/disturbance/payload";
import { disturbanceParamsFor } from "../src/core/disturbance/scenario";
import { makeHex } from "../src/core/multirotor/config";
import { quatRotateVector } from "../src/core/math/quat";
import { TELEMETRY_CAPACITY } from "../src/viewer/sim/constants";
import { MISSIONS } from "../src/viewer/sim/mission";
import {
  createSimulation,
  stepOnce,
  tiltAngleRad,
  type SimulationPlant,
} from "../src/viewer/sim/runner";

function plant(): SimulationPlant {
  const config = makeHex({ groundPlane: true });
  return { config, params: defaultControllerParams(config) };
}

interface EnvelopeResult {
  readonly survive: boolean;
  readonly contactSteps: number;
  readonly maxErrorLast10s: number;
}

/**
 * Survival envelope: a point mass is added at a body position chosen so that
 * the resulting COM offset is `offset`. Survive = no ground contact in 120 s
 * AND 3D position error < 0.1 m for every sample in the last 10 s.
 */
function envelope(
  mass: number,
  offset: number,
  controllerParams?: ControllerParams,
  seed = 7,
): EnvelopeResult {
  const base = plant();
  const p = controllerParams === undefined ? base : { ...base, params: controllerParams };
  const total = base.config.mass + mass;
  const position = [offset === 0 ? 0 : (offset * total) / mass, 0, 0] as const;
  const params = disturbanceParamsFor("calm", seed, {
    payload: { atTime: 0, payload: { mass, position } },
  });
  let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
  let maxError = 0;
  for (let step = 0; step < 120_000; step++) {
    state = stepOnce(state, p);
    if (state.simTime >= 110) {
      const position = state.multirotor.body.position;
      maxError = Math.max(
        maxError,
        Math.hypot(position[0], position[1], position[2] - 5),
      );
    }
  }
  return {
    survive: state.stats.contactSteps === 0 && maxError < 0.1,
    contactSteps: state.stats.contactSteps,
    maxErrorLast10s: maxError,
  };
}

describe("payload shift", () => {
  it("matches the hand-computed parallel-axis inertia exactly", () => {
    const config = makeHex();
    const shifted = applyPayload(config, PAYLOAD_2CM);
    const com = payloadCenterOfMass(config, PAYLOAD_2CM);
    expect(com[0]).toBeCloseTo(0.02, 12);
    expect(com[1]).toBe(0);
    expect(com[2]).toBe(0);
    expect(shifted.mass).toBeCloseTo(1.8, 12);

    // Ixx = 0.5*m0*R^2, unchanged because the offset lies on body +x.
    expect(shifted.inertia[0]).toBeCloseTo(0.046875, 12);
    // Body term m0*d^2 = 6.0e-4; payload term mp*(r_p-d)^2 = 3.0e-3.
    const bodyTerm = 1.5 * 0.02 * 0.02;
    const payloadTerm = 0.3 * 0.1 * 0.1;
    expect(shifted.inertia[4]).toBeCloseTo(0.046875 + bodyTerm + payloadTerm, 12);
    expect(shifted.inertia[4]).toBeCloseTo(0.050475, 12);
    expect(shifted.inertia[8]).toBeCloseTo(0.09375 + bodyTerm + payloadTerm, 12);
    expect(shifted.inertia[8]).toBeCloseTo(0.09735, 12);
    for (const index of [1, 2, 3, 5, 6, 7]) {
      expect(shifted.inertia[index]!).toBeCloseTo(0, 12);
    }
    expect(parallelAxisTensor([0.02, 0, 0])[4]).toBeCloseTo(0.0004, 12);

    for (const [index, motor] of shifted.motors.entries()) {
      expect(motor.position[0]).toBeCloseTo(
        config.motors[index]!.position[0] - 0.02, 12,
      );
      expect(motor.position[1]).toBe(config.motors[index]!.position[1]);
    }
    expect(config.mass).toBe(1.5);
    expect(config.motors[0]!.position[0]).toBe(0.25);
  });

  it("survival envelope with the integral release: all offsets up to 1 cm survive below 20% mass", () => {
    // Re-pinned after the fix(control) commit. The persistent-error release
    // removes the frozen-integral equilibrium, so 1–10% payloads at 2–10 mm
    // COM shift now survive. The 20 mm column still contacts the ground during
    // the transient, and 20% mass contacts at every offset; both are M6
    // targets for the inner loop and are not claimed as fixed here.
    const masses = [0.01, 0.02, 0.05, 0.10, 0.20]; // fractions of 1.5 kg
    const offsets = [0.002, 0.005, 0.01, 0.02]; // metres of COM shift
    const expected = [
      [true, true, true, false],
      [true, true, true, false],
      [true, true, true, false],
      [true, true, true, false],
      [false, false, false, false],
    ] as const;
    const measured = masses.map((fraction) =>
      offsets.map((offset) => envelope(fraction * 1.5, offset).survive),
    );
    expect(measured).toEqual(expected);
  }, 900_000);

  it("baseline (payload-small): 0.2% mass with a 1 mm COM shift survives (M6 target)", () => {
    // Named surviving case from the finer envelope sweep; also a baseline
    // observation rather than a controller requirement.
    const result = envelope(PAYLOAD_SMALL.mass, 0.001);
    expect(result.survive).toBe(true);
    expect(result.contactSteps).toBe(0);
    expect(result.maxErrorLast10s).toBeLessThan(0.1);
  }, 60_000);

  it("baseline limitation (payload-large stress): 20% mass with a 2 cm COM shift fails (M6 target)", () => {
    // Named failing case. Do not "fix" by loosening the envelope; fix the
    // controller (integral separation / inner-loop bandwidth) in a later
    // milestone and then flip this expectation.
    expect(PAYLOAD_LARGE_STRESS).toBe(PAYLOAD_2CM);
    const result = envelope(PAYLOAD_LARGE_STRESS.mass, 0.02);
    expect(result.survive).toBe(false);
    expect(result.contactSteps).toBeGreaterThan(0);
  }, 60_000);

  it("baseline limitation: integral separation freezes the position integral (M6 target)", () => {
    // With the M3 gate (`integralDwell = 0`) the centred +20% mass case froze
    // at an integral of 71.454 m*s. The release now grows it past 190 m*s and
    // recovers the altitude. This test pins the current behaviour; the
    // separation-only variant is still available by setting dwell to 0.
    const base = plant().params;
    const frozen = envelope(0.3, 0, base);
    const open = envelope(0.3, 0, {
      ...base,
      position: { ...base.position, integralSeparation: 1e9 },
    });
    expect(frozen.survive).toBe(false);
    expect(open.maxErrorLast10s).toBeLessThan(0.1);
    // Flip these two when the default controller rejects the same mismatch.
  }, 180_000);

  it("keeps the airframe level under a small off-centre payload", () => {
    // The rate integrator supplies the trim; this part of the model is correct
    // even though the outer loop does not fully reject the mass mismatch.
    const p = plant();
    const params = disturbanceParamsFor("calm", 8, {
      payload: { atTime: 0, payload: { mass: 0.01, position: [0.3, 0, 0] } },
    });
    let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
    for (let step = 0; step < 60_000; step++) state = stepOnce(state, p);
    const tiltDegrees = (tiltAngleRad(state.multirotor.body.orientation) * 180) / Math.PI;
    expect(tiltDegrees).toBeLessThan(1);
    const bodyZ = quatRotateVector(state.multirotor.body.orientation, [0, 0, 1]);
    expect(Number.isFinite(bodyZ[2])).toBe(true);
    const speeds = state.multirotor.motorSpeeds;
    const spread = Math.max(...speeds) - Math.min(...speeds);
    expect(spread).toBeGreaterThan(1);
    expect(state.diagnostics!.trueMass).toBeCloseTo(1.51, 12);
  }, 120_000);
});
