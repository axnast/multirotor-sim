/**
 * Ground plane: resolver unit tests (G1-G11) and step-integration tests
 * (S1-S10). The resolver is a pure post-step impulse, so these tests pin both
 * its algebra and its interaction with the motor lag and RK4 integrator.
 */

import { describe, expect, test } from "vitest";

import { totalEnergy } from "../src/core/energy";
import { QUAT_IDENTITY, quatFromAxisAngle } from "../src/core/math/quat";
import type { Vec3 } from "../src/core/math/vec3";
import {
  makeHex,
  rigidBodyParamsOf,
  type MultirotorConfig,
  type MultirotorOptions,
} from "../src/core/multirotor/config";
import {
  multirotorDerivatives,
  stepMultirotor,
  stepMultirotorDetailed,
  STILL_AIR,
  type MultirotorState,
} from "../src/core/multirotor/dynamics";
import {
  DEFAULT_GROUND_PARAMS,
  resolveGroundContact,
  validateGroundParams,
  type GroundContactResult,
  type GroundParams,
} from "../src/core/multirotor/ground";
import { mix, type Wrench } from "../src/core/multirotor/mixer";
import type { RigidBodyState } from "../src/core/rigidBody";
import {
  at,
  close,
  makeState,
  stepN,
  vecClose,
} from "./helpers/multirotor";

const DT = 1e-3;
const MU = DEFAULT_GROUND_PARAMS.friction;

function makeBody(overrides: Partial<RigidBodyState> = {}): RigidBodyState {
  return {
    position: [0, 0, 0],
    velocity: [0, 0, 0],
    orientation: QUAT_IDENTITY,
    angularVelocity: [0, 0, 0],
    ...overrides,
  };
}

/** Drag-free airframe so the ground is the only force besides gravity/thrust. */
function grounded(overrides: MultirotorOptions = {}): MultirotorConfig {
  return makeHex({
    groundPlane: true,
    linearDrag: [0, 0, 0],
    quadraticDrag: [0, 0, 0],
    ...overrides,
  });
}

function zeroCommands(config: MultirotorConfig): number[] {
  const commands = mix(config, [0, 0, 0, 0]);
  expect(commands.every((speed) => speed === 0)).toBe(true);
  return commands;
}

function expectZeroSpeeds(state: MultirotorState): void {
  expect(state.motorSpeeds.every((speed) => speed === 0)).toBe(true);
}

describe("ground contact resolver", () => {
  test("G1. leaves a body above the plane untouched", () => {
    const body = makeBody({
      position: [1, 2, 1],
      velocity: [1, 2, -3],
      orientation: quatFromAxisAngle([0.1, 0.2, 0.3], 0.5),
      angularVelocity: [0.1, -0.2, 0.3],
    });
    const before = JSON.stringify(body);
    const result = resolveGroundContact(body, 2);

    expect(result.body).toEqual(body);
    expect(result.contacted).toBe(false);
    expect(result.penetration).toBe(0);
    expect(result.normalImpulse).toBe(0);
    expect(result.frictionImpulse).toBe(0);
    expect(JSON.stringify(body)).toBe(before);
  });

  test("G2. projects penetration and removes inward normal velocity", () => {
    const result = resolveGroundContact(
      makeBody({ position: [0, 0, -0.01], velocity: [0.5, -0.25, -2] }),
      2,
    );

    expect(result.contacted).toBe(true);
    close(result.penetration, 0.01, 1e-12);
    close(result.normalImpulse, 4, 1e-12);
    // The small slip is smaller than μ J_n, so friction stops it completely.
    close(result.frictionImpulse, 2 * Math.hypot(0.5, -0.25), 1e-12);
    expect(result.body.position).toEqual([0, 0, 0]);
    vecClose(result.body.velocity, [0, 0, 0], 1e-12);
  });

  test("G3. applies Coulomb friction up to μ J_n", () => {
    const result = resolveGroundContact(
      makeBody({ position: [0, 0, -0.01], velocity: [1, 0, -1] }),
      2,
    );

    expect(result.contacted).toBe(true);
    close(result.normalImpulse, 2, 1e-12);
    close(result.frictionImpulse, 1.2, 1e-12);
    close(at(result.body.velocity, 0), 0.4, 1e-12);
    expect(at(result.body.velocity, 2)).toBe(0);
    expect(result.body.position).toEqual([0, 0, 0]);
  });

  test("G4. stops slow tangential motion without reversing it", () => {
    const result = resolveGroundContact(
      makeBody({ position: [0, 0, -0.01], velocity: [0.1, 0, -1] }),
      2,
    );

    close(result.normalImpulse, 2, 1e-12);
    close(result.frictionImpulse, 0.2, 1e-12);
    expect(at(result.body.velocity, 0)).toBe(0);
    expect(at(result.body.velocity, 0)).toBeGreaterThanOrEqual(0);
  });

  test("G5. preserves tangential direction while reducing speed", () => {
    const result = resolveGroundContact(
      makeBody({ position: [0, 0, -0.01], velocity: [-1, 2, -1] }),
      2,
    );
    const vx = at(result.body.velocity, 0);
    const vy = at(result.body.velocity, 1);

    close(result.normalImpulse, 2, 1e-12);
    close(result.frictionImpulse, 1.2, 1e-12);
    close(vy / vx, -2, 1e-12);
    close(Math.hypot(vx, vy), Math.sqrt(5) - 1.2 / 2, 1e-12);
  });

  test("G6. no friction without an inward normal velocity", () => {
    const result = resolveGroundContact(
      makeBody({ position: [0, 0, -0.01], velocity: [1, 0, 1] }),
      2,
    );

    expect(result.contacted).toBe(true);
    expect(result.body.position).toEqual([0, 0, 0]);
    vecClose(result.body.velocity, [1, 0, 1], 1e-12);
    expect(result.normalImpulse).toBe(0);
    expect(result.frictionImpulse).toBe(0);
  });

  test("G7. applies restitution to the normal velocity", () => {
    const result = resolveGroundContact(
      makeBody({ position: [0, 0, -0.01], velocity: [0, 0, -2] }),
      2,
      { restitution: 0.5, friction: MU },
    );

    close(at(result.body.velocity, 2), 1, 1e-12);
    close(result.normalImpulse, 6, 1e-12);
    expect(result.frictionImpulse).toBe(0);
  });

  test("G8. resolves a body exactly on the plane and closing", () => {
    const result = resolveGroundContact(
      makeBody({ position: [0, 0, 0], velocity: [0, 0, -1] }),
      2,
    );

    expect(result.contacted).toBe(true);
    expect(result.penetration).toBe(0);
    expect(at(result.body.velocity, 2)).toBe(0);
    close(result.normalImpulse, 2, 1e-12);
  });

  test("G9. leaves orientation and angular velocity unchanged", () => {
    const orientation = quatFromAxisAngle([0.2, 0.1, 0.9], 0.4);
    const angularVelocity: Vec3 = [0.1, -0.2, 0.3];
    const body = makeBody({
      position: [0, 0, -0.01],
      velocity: [0, 0, -1],
      orientation,
      angularVelocity,
    });
    const result = resolveGroundContact(body, 2);

    // Contact applies no torque: no tipping, no support polygon, no damping.
    expect(result.body.orientation).toEqual(orientation);
    expect(result.body.angularVelocity).toEqual(angularVelocity);
  });

  test("G10. is pure and deterministic", () => {
    const body = makeBody({
      position: [0.3, -0.2, -0.01],
      velocity: [0.7, -0.4, -1.3],
      orientation: quatFromAxisAngle([0, 1, 0], 0.8),
      angularVelocity: [0.1, 0.2, -0.3],
    });
    const before = JSON.stringify(body);
    const first = resolveGroundContact(body, 2);
    const second = resolveGroundContact(body, 2);

    expect(first).toEqual(second);
    expect(JSON.stringify(body)).toBe(before);
  });

  test("G11. validates mass, body and ground params", () => {
    for (const mass of [0, -1, Number.NaN, Number.POSITIVE_INFINITY]) {
      expect(() => resolveGroundContact(makeBody(), mass)).toThrow(RangeError);
    }
    expect(() =>
      resolveGroundContact(makeBody({ position: [0, Number.NaN, 0] }), 2),
    ).toThrow(RangeError);
    expect(() =>
      resolveGroundContact(makeBody({ velocity: [0, 0, Number.NaN] }), 2),
    ).toThrow(RangeError);
    expect(() =>
      resolveGroundContact(makeBody({ angularVelocity: [0, 0, Number.NaN] }), 2),
    ).toThrow(RangeError);
    expect(() =>
      resolveGroundContact(makeBody({ orientation: [0, 0, 0, 0] }), 2),
    ).toThrow(RangeError);

    for (const restitution of [-0.1, 1.1, Number.NaN]) {
      expect(() =>
        validateGroundParams({ restitution, friction: MU }),
      ).toThrow(RangeError);
    }
    for (const friction of [-0.1, Number.NaN, Number.POSITIVE_INFINITY]) {
      expect(() =>
        validateGroundParams({ restitution: 0, friction }),
      ).toThrow(RangeError);
    }
    expect(() =>
      resolveGroundContact(makeBody(), 2, { restitution: -1, friction: MU }),
    ).toThrow(RangeError);
  });
});

describe("ground contact through the stepper", () => {
  test("S1. does nothing when groundPlane is false", () => {
    const config = grounded({ groundPlane: false });
    const commands = zeroCommands(config);
    const state = makeState(config, {
      position: [0, 0, 0.01],
      velocity: [0, 0, -10],
      motorSpeeds: commands,
    });

    const result = stepMultirotorDetailed(state, config, commands, 0.01);
    expect(result.ground).toBeNull();
    expect(result.state.body.position[2]).toBeLessThan(0);
  });

  test("S2. holds a resting vehicle exactly at z = 0", () => {
    // Zero motor speeds: a hover start would leave T ≈ mg during spin-down,
    // making the normal impulse (and the exact z === 0) roundoff-dependent.
    const config = grounded();
    const commands = zeroCommands(config);
    let state = makeState(config, {
      position: [0, 0, 0],
      motorSpeeds: commands,
    });
    expectZeroSpeeds(state);
    let ground: GroundContactResult | null = null;

    for (let index = 0; index < 1000; index++) {
      const result = stepMultirotorDetailed(state, config, commands, DT);
      state = result.state;
      ground = result.ground;
    }

    expect(state.body.position).toEqual([0, 0, 0]);
    expect(state.body.velocity).toEqual([0, 0, 0]);
    expectZeroSpeeds(state);
    expect(ground?.contacted).toBe(true);
  });

  test("S3. allows takeoff when thrust exceeds weight", () => {
    const config = grounded();
    const wrench: Wrench = [1.1 * config.mass * 9.81, 0, 0, 0];
    const commands = mix(config, wrench);
    let state = makeState(config, {
      position: [0, 0, 0],
      motorSpeeds: commands,
    });
    expect(state.motorSpeeds).toEqual(commands);
    let ground: GroundContactResult | null = null;

    for (let index = 0; index < 2000; index++) {
      const result = stepMultirotorDetailed(state, config, commands, DT);
      state = result.state;
      ground = result.ground;
    }

    close(state.body.position[2], 1.962, 1e-9);
    close(at(state.body.velocity, 2), 1.962, 1e-9);
    expect(ground?.contacted).toBe(false);
  });

  test("S4. prevents a zero-thrust vehicle from falling through", () => {
    const config = grounded();
    const commands = zeroCommands(config);
    let state = makeState(config, {
      position: [0, 0, 5],
      motorSpeeds: commands,
    });
    expectZeroSpeeds(state);
    const params = rigidBodyParamsOf(config);
    const initialEnergy = totalEnergy(state.body, params);
    close(initialEnergy, 73.575, 1e-9);

    state = stepN(state, config, commands, 3000, DT);

    expect(state.body.position).toEqual([0, 0, 0]);
    expect(state.body.velocity).toEqual([0, 0, 0]);
    const finalEnergy = totalEnergy(state.body, params);
    expect(finalEnergy).toBeLessThan(initialEnergy);
    close(finalEnergy, 0, 1e-12);
  });

  test("S5. does not affect free flight above the plane", () => {
    const withGround = grounded();
    const withoutGround = grounded({ groundPlane: false });
    const commands = mix(withGround, [1.1 * withGround.mass * 9.81, 0, 0, 0]);

    const run = (config: MultirotorConfig): MultirotorState =>
      stepN(
        makeState(config, { position: [0, 0, 5], motorSpeeds: commands }),
        config,
        commands,
        1000,
        DT,
      );

    expect(run(withGround)).toEqual(run(withoutGround));
  });

  test("S6. applies Coulomb friction to a sliding vehicle", () => {
    const config = grounded();
    const commands = zeroCommands(config);
    let state = makeState(config, {
      position: [0, 0, 0],
      velocity: [1, 0, 0],
      motorSpeeds: commands,
    });
    expectZeroSpeeds(state);

    state = stepN(state, config, commands, 1000, DT);

    // Friction is applied after the position update, so the exact discrete
    // distance is v0·dt/2 larger than the continuous v0²/(2μg) = 0.08494733 m:
    //   N = ceil(v0/(μg·dt)) = 170
    //   x = dt · (N·v0 - μg·dt·N(N-1)/2) = 0.08544761 m
    const steps = Math.ceil(1 / (MU * 9.81 * DT));
    const exact = DT * (steps - MU * 9.81 * DT * (steps * (steps - 1)) / 2);
    expect(steps).toBe(170);
    close(state.body.position[0], exact, 1e-9);
    expect(at(state.body.velocity, 0)).toBe(0);
    expect(state.body.position[2]).toBe(0);
  });

  test("S7. supports zero friction", () => {
    const config = grounded();
    const commands = zeroCommands(config);
    const frictionless: GroundParams = { restitution: 0, friction: 0 };
    let state = makeState(config, {
      position: [0, 0, 0],
      velocity: [1, 0, 0],
      motorSpeeds: commands,
    });
    expectZeroSpeeds(state);

    state = stepN(state, config, commands, 1000, DT, STILL_AIR, frictionless);

    close(at(state.body.velocity, 0), 1, 1e-9);
    close(state.body.position[0], 1, 1e-9);
    expect(state.body.position[2]).toBe(0);
  });

  test("S8. derivatives remain the free-flight field", () => {
    const config = grounded();
    const commands = zeroCommands(config);
    const state = makeState(config, {
      position: [0, 0, 0],
      motorSpeeds: commands,
    });

    const derivative = multirotorDerivatives(state, config, commands);
    close(at(derivative.body.velocity, 2), -9.81, 1e-9);
  });

  test("S9. detailed step is consistent with the simple step", () => {
    const config = grounded();
    const commands = zeroCommands(config);
    const initial = makeState(config, {
      position: [0, 0, 0],
      motorSpeeds: commands,
    });

    let simple = initial;
    let detailed = initial;
    for (let index = 0; index < 10; index++) {
      simple = stepMultirotor(simple, config, commands, DT);
      detailed = stepMultirotorDetailed(detailed, config, commands, DT).state;
    }
    expect(detailed).toEqual(simple);

    const disabled = grounded({ groundPlane: false });
    expect(stepMultirotorDetailed(initial, disabled, commands, DT).ground).toBeNull();
    const above = makeState(config, { position: [0, 0, 5], motorSpeeds: commands });
    expect(stepMultirotorDetailed(above, config, commands, DT).ground?.contacted).toBe(false);
  });

  test("S10. contact applies no angular impulse", () => {
    const config = grounded();
    const freeFlight = grounded({ groundPlane: false });
    // 0.9× hover thrust with the motors already at their commanded speeds:
    // there is no lag transient in T, so the vehicle stays pressed down.
    const commands = mix(config, [0.9 * config.mass * 9.81, 0, 0, 0.01]);
    let onGround = makeState(config, {
      position: [0, 0, 0],
      motorSpeeds: commands,
    });
    let inAir = makeState(freeFlight, {
      position: [0, 0, 0],
      motorSpeeds: commands,
    });
    expect(onGround.motorSpeeds).toEqual(commands);
    expect(inAir.motorSpeeds).toEqual(commands);

    for (let index = 0; index < 100; index++) {
      onGround = stepMultirotor(onGround, config, commands, DT);
      inAir = stepMultirotor(inAir, freeFlight, commands, DT);
    }

    expect(onGround.body.position[2]).toBe(0);
    expect(at(onGround.body.angularVelocity, 2)).toBeGreaterThan(0);
    vecClose(onGround.body.angularVelocity, inAir.body.angularVelocity, 1e-9);
  });
});
