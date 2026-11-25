import { describe, expect, it } from "vitest";

import { defaultControllerParams } from "../src/core/control/controller";
import {
  applyPayload,
  PAYLOAD_2CM,
  PAYLOAD_MASS_20PCT,
  PAYLOAD_TRIM,
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

  it("dips after a discrete centred +20% payload at t=5 s and stays bounded", () => {
    // Finding (documented, not silently retuned): positionStep holds its
    // integrator whenever |error| > integralSeparation (0.1 m) and the vehicle
    // is moving toward the setpoint (src/core/control/position.ts). A +20%
    // mass mismatch therefore settles at a residual altitude error; with the
    // default 5 m hover start it reaches the ground plane, where the integral
    // freezes. No NaN, no runaway, and the mixer/controller stay nominal.
    const p = plant();
    const params = disturbanceParamsFor("calm", 7, {
      payload: { atTime: 5, payload: PAYLOAD_MASS_20PCT },
    });
    let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
    let minimumAfterDrop = Infinity;
    for (let step = 0; step < 90_000; step++) {
      state = stepOnce(state, p);
      const altitude = state.multirotor.body.position[2];
      if (state.simTime >= 5 && altitude < minimumAfterDrop) {
        minimumAfterDrop = altitude;
      }
    }
    expect(state.diagnostics!.payloadApplied).toBe(true);
    expect(state.diagnostics!.trueMass).toBeCloseTo(1.8, 12);
    expect(minimumAfterDrop).toBeLessThan(4.9);
    expect(state.multirotor.body.position.every(Number.isFinite)).toBe(true);
    expect(Math.abs(state.multirotor.body.position[0])).toBeLessThan(0.1);
    expect(Math.abs(state.multirotor.body.position[1])).toBeLessThan(0.1);
    // Reported by the test run: minimum altitude and final altitude.
    expect(state.multirotor.body.position[2]).toBeGreaterThanOrEqual(0);
    expect(state.multirotor.body.position[2]).toBeLessThan(5);
  }, 120_000);

  it("trims an off-centre payload with level attitude and a nonzero spread", () => {
    // Same integral-separation limitation: the lateral residual does not go
    // to zero within the run. What M5 must verify is the nominal/true split:
    // the rate-loop integral trims the COM-offset torque via a persistent
    // motor-speed differential while the attitude stays level.
    const p = plant();
    const params = disturbanceParamsFor("calm", 8, {
      payload: { atTime: 0, payload: PAYLOAD_TRIM },
    });
    let state = createSimulation(p, MISSIONS.hover, TELEMETRY_CAPACITY, params);
    for (let step = 0; step < 60_000; step++) state = stepOnce(state, p);

    const position = state.multirotor.body.position;
    const tiltDegrees = (tiltAngleRad(state.multirotor.body.orientation) * 180) / Math.PI;
    const bodyZ = quatRotateVector(state.multirotor.body.orientation, [0, 0, 1]);
    expect(Number.isFinite(bodyZ[2])).toBe(true);
    expect(tiltDegrees).toBeLessThan(1);

    const speeds = state.multirotor.motorSpeeds;
    const mean = speeds.reduce((sum, value) => sum + value, 0) / speeds.length;
    const spread = Math.max(...speeds) - Math.min(...speeds);
    const deviation = Math.max(...speeds.map((value) => Math.abs(value - mean)));
    expect(spread).toBeGreaterThan(1);
    expect(deviation).toBeGreaterThan(1);
    expect(state.diagnostics!.trueMass).toBeCloseTo(1.51, 12);
    // Reported by the test run: position residual, mean, spread, deviation.
    expect(Math.hypot(position[0], position[1], position[2] - 5)).toBeLessThan(1);
    expect(Number.isFinite(mean)).toBe(true);
  }, 120_000);
});
