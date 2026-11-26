/**
 * Worker body for scripts/measureParallel.ts: one no-disturbance hex hover
 * rollout through the same `simulateStep` used by tuning and the viewer.
 */

import { parentPort, workerData } from "node:worker_threads";

import {
  defaultControllerParams,
  initialControllerState,
} from "../../src/core/control/controller";
import { QUAT_IDENTITY } from "../../src/core/math/quat";
import { makeHex } from "../../src/core/multirotor/config";
import { simulateStep, type CoreState } from "../../src/core/simulation/loop";

const steps = workerData as number;
const config = makeHex({ groundPlane: false });
const params = defaultControllerParams(config);
const setpoint = { position: [0, 0, 5] as const, yaw: 0 };

let core: CoreState = {
  multirotor: {
    body: {
      position: [0, 0, 5],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    },
    motorSpeeds: config.motors.map(() => 0),
  },
  controller: initialControllerState(),
  disturbance: null,
  disturbanceParams: null,
  diagnostics: null,
  trueConfig: config,
};

const start = performance.now();
for (let step = 0; step < steps; step++) {
  const result = simulateStep(core, { config, params }, setpoint, 0.001);
  core = {
    ...core,
    multirotor: result.state,
    controller: result.controller,
    trueConfig: result.trueConfig,
  };
}
const milliseconds = performance.now() - start;
parentPort?.postMessage({ steps, milliseconds });
