/**
 * Viewer bootstrap: config → runner → Three.js scene. Physics and control are
 * imported from src/core only; this file wires rendering and input.
 */

import "./style.css";

import { defaultControllerParams } from "../core/control/controller";
import { makeHex } from "../core/multirotor/config";
import { createAxisLegend } from "./scene/axisLegend";
import { createCameraRig } from "./scene/cameraRig";
import { buildDroneModel, updateDrone } from "./scene/drone";
import { createSetpointMarker } from "./scene/setpointMarker";
import { createViewerScene } from "./scene/scene";
import { createTrail } from "./scene/trail";
import { MISSIONS } from "./sim/mission";
import { ringToArray } from "./sim/ringBuffer";
import {
  advanceSimulation,
  createSimulation,
  resetSimulation,
  setPaused,
  type SimulationPlant,
} from "./sim/runner";

const app = document.querySelector<HTMLDivElement>("#app");
if (app === null) {
  throw new Error("Missing #app root element");
}

const config = makeHex({ groundPlane: true });
const plant: SimulationPlant = { config, params: defaultControllerParams(config) };
let state = createSimulation(plant, MISSIONS.hover);

const viewer = createViewerScene(app);
const drone = buildDroneModel(config);
updateDrone(drone, state.multirotor, 0);
viewer.scene.add(drone.group);

const trail = createTrail(state.trail.capacity);
viewer.scene.add(trail.line);
const marker = createSetpointMarker();
viewer.scene.add(marker.group);
const legend = createAxisLegend(app);
const cameraRig = createCameraRig(
  viewer.camera,
  viewer.canvas,
  drone.group.position.clone(),
);

let follow = false;
let lastTime = performance.now();

viewer.renderer.setAnimationLoop((timeMs: number): void => {
  const realDt = Math.min(0.25, Math.max(0, (timeMs - lastTime) / 1000));
  lastTime = timeMs;

  state = advanceSimulation(state, plant, realDt).state;
  updateDrone(drone, state.multirotor, state.paused ? 0 : realDt * state.speed);
  trail.update(ringToArray(state.trail));
  marker.update(state.setpoint, timeMs / 1000);
  cameraRig.update(drone.group.position, realDt, follow);

  viewer.renderer.render(viewer.scene, viewer.camera);
});

window.addEventListener("resize", viewer.resize);

window.addEventListener("keydown", (event: KeyboardEvent): void => {
  if (event.target instanceof HTMLInputElement) return;
  switch (event.code) {
    case "Space":
      event.preventDefault();
      state = setPaused(state, !state.paused);
      break;
    case "KeyR":
      state = resetSimulation(state, plant);
      cameraRig.reset(drone.group.position.clone());
      break;
    case "KeyF":
      follow = !follow;
      break;
    case "KeyA":
      drone.axes.visible = !drone.axes.visible;
      legend.setVisible(drone.axes.visible);
      break;
    default:
      break;
  }
});
