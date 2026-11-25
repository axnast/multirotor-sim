/**
 * Viewer bootstrap: config → runner → Three.js scene + DOM controls.
 * Physics and control are imported from src/core only; this file wires input
 * and rendering, including bumpless live gain changes.
 */

import "./style.css";

import * as THREE from "three";

import { defaultControllerParams } from "../core/control/controller";
import type { ControllerParams, Setpoint } from "../core/control/types";
import { vec3Length, vec3Sub } from "../core/math/vec3";
import { makeHex } from "../core/multirotor/config";
import { threeToCore } from "./coords";
import { createPlotsPanel } from "./plots/plotsPanel";
import { createAxisLegend } from "./scene/axisLegend";
import { createCameraRig } from "./scene/cameraRig";
import { buildDroneModel, updateDrone } from "./scene/drone";
import { createSetpointMarker } from "./scene/setpointMarker";
import { createViewerScene } from "./scene/scene";
import { createTrail, type TrailVisual } from "./scene/trail";
import { bumplessKiChange as rescaleIntegrals } from "./sim/bumpless";
import { MISSIONS } from "./sim/mission";
import { ringToArray } from "./sim/ringBuffer";
import {
  advanceSimulation,
  createSimulation,
  resetSimulation,
  selectMission,
  setManualSetpoint,
  setPaused,
  setSpeed,
  stepOnce,
  withTrailCapacity,
  type SimulationPlant,
} from "./sim/runner";
import { createControls, type ControlsView } from "./ui/controls";
import { createHud } from "./ui/hud";
import {
  applyParamValue,
  isKiSlider,
  type ParamKey,
} from "./ui/paramSpecs";

const app = document.querySelector<HTMLDivElement>("#app");
if (app === null) {
  throw new Error("Missing #app root element");
}

const config = makeHex({ groundPlane: true });
let params: ControllerParams = defaultControllerParams(config);
let plant: SimulationPlant = { config, params };
let state = createSimulation(plant, MISSIONS.hover);

const viewer = createViewerScene(app);
const drone = buildDroneModel(config);
updateDrone(drone, state.multirotor, 0);
viewer.scene.add(drone.group);

let trail: TrailVisual = createTrail(state.trail.capacity);
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
let showAxes = true;
let lastTime = performance.now();
let lastStatusTime = 0;
let lastPlotsTime = 0;

const controls: ControlsView = createControls(app, params, {
  onMission(mission): void {
    state = selectMission(state, plant, mission);
    controls.setMission(mission.id);
    controls.setWaypoint(state.setpoint);
  },
  onPause(paused): void {
    state = setPaused(state, paused);
    controls.setPaused(state.paused);
  },
  onStep(): void {
    if (!state.paused) {
      state = setPaused(state, true);
      controls.setPaused(true);
    }
    state = stepOnce(state, plant);
  },
  onReset(): void {
    state = resetSimulation(state, plant);
    controls.setMission(state.mission?.mission.id ?? null);
    controls.setWaypoint(state.setpoint);
  },
  onSpeed(speed): void {
    state = setSpeed(state, speed);
    controls.setSpeed(state.speed);
  },
  onAltitude(altitude): void {
    const setpoint: Setpoint = {
      position: [
        state.setpoint.position[0],
        state.setpoint.position[1],
        altitude,
      ],
      yaw: state.setpoint.yaw,
    };
    state = setManualSetpoint(state, setpoint);
    controls.setMission(null);
    controls.setWaypoint(setpoint);
  },
  onYaw(yawRadians): void {
    const setpoint: Setpoint = {
      position: [
        state.setpoint.position[0],
        state.setpoint.position[1],
        state.setpoint.position[2],
      ],
      yaw: yawRadians,
    };
    state = setManualSetpoint(state, setpoint);
    controls.setMission(null);
    controls.setWaypoint(setpoint);
  },
  onFollow(next): void {
    follow = next;
    controls.setFollow(follow);
  },
  onShowAxes(visible): void {
    showAxes = visible;
    drone.axes.visible = showAxes;
    legend.setVisible(showAxes);
    controls.setShowAxes(showAxes);
  },
  onTrailCapacity(capacity): void {
    viewer.scene.remove(trail.line);
    trail.dispose();
    state = withTrailCapacity(state, capacity);
    trail = createTrail(capacity);
    viewer.scene.add(trail.line);
  },
  onSlider(id: ParamKey, value): void {
    const oldParams = params;
    const next = applyParamValue(oldParams, id, value);
    if (isKiSlider(id)) {
      // Keep ki·integral constant so the live change is bumpless.
      state = { ...state, controller: rescaleIntegrals(state.controller, oldParams, next) };
    }
    params = next;
    plant = { config, params };
    controls.setParams(params);
  },
  onResetGains(): void {
    const oldParams = params;
    params = defaultControllerParams(config);
    state = { ...state, controller: rescaleIntegrals(state.controller, oldParams, params) };
    plant = { config, params };
    controls.setParams(params);
  },
});

const hud = createHud(app);
const plots = createPlotsPanel(app, config.motors.length);

controls.setMission(MISSIONS.hover.id);
controls.setPaused(false);
controls.setSpeed(1);

// Click the ground plane to command a waypoint; dragging orbits the camera.
const raycaster = new THREE.Raycaster();
const groundPlane = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const pointer = new THREE.Vector2();
let pointerDown: { readonly x: number; readonly y: number; readonly at: number } | null = null;
viewer.canvas.addEventListener("pointerdown", (event: PointerEvent): void => {
  pointerDown = { x: event.clientX, y: event.clientY, at: performance.now() };
});
viewer.canvas.addEventListener("pointerup", (event: PointerEvent): void => {
  if (pointerDown === null) return;
  const moved = Math.hypot(event.clientX - pointerDown.x, event.clientY - pointerDown.y);
  const held = performance.now() - pointerDown.at;
  pointerDown = null;
  if (moved > 5 || held > 400) return;

  const rect = viewer.canvas.getBoundingClientRect();
  pointer.set(
    ((event.clientX - rect.left) / rect.width) * 2 - 1,
    -((event.clientY - rect.top) / rect.height) * 2 + 1,
  );
  raycaster.setFromCamera(pointer, viewer.camera);
  const hit = new THREE.Vector3();
  if (raycaster.ray.intersectPlane(groundPlane, hit) === null) return;
  const core = threeToCore([hit.x, hit.y, hit.z]);
  const { altitude, yawRadians } = controls.getWaypointInputs();
  const clamp = (value: number): number => Math.min(25, Math.max(-25, value));
  const setpoint: Setpoint = {
    position: [clamp(core[0]), clamp(core[1]), altitude],
    yaw: yawRadians,
  };
  state = setManualSetpoint(state, setpoint);
  controls.setMission(null);
  controls.setWaypoint(setpoint);
});

viewer.renderer.setAnimationLoop((timeMs: number): void => {
  const realDt = Math.min(0.25, Math.max(0, (timeMs - lastTime) / 1000));
  lastTime = timeMs;

  state = advanceSimulation(state, plant, realDt).state;
  updateDrone(drone, state.multirotor, state.paused ? 0 : realDt * state.speed);
  trail.update(ringToArray(state.trail));
  marker.update(state.setpoint, timeMs / 1000);
  cameraRig.update(drone.group.position, realDt, follow);

  if (timeMs - lastStatusTime > 100) {
    lastStatusTime = timeMs;
    const missionName = state.mission === null ? "Manual" : state.mission.mission.name;
    const error = vec3Length(
      vec3Sub(state.multirotor.body.position, state.setpoint.position),
    );
    controls.setStatus(
      `${missionName} · ${state.paused ? "paused" : `${state.speed}×`} · ` +
        `t ${state.simTime.toFixed(1)} s · |e| ${error.toFixed(2)} m`,
    );
  }

  if (timeMs - lastPlotsTime > 33) {
    lastPlotsTime = timeMs;
    hud.update(state);
    plots.update(state);
  }

  viewer.renderer.render(viewer.scene, viewer.camera);
});

window.addEventListener("resize", viewer.resize);

window.addEventListener("keydown", (event: KeyboardEvent): void => {
  if (event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement) return;
  switch (event.code) {
    case "Space":
      event.preventDefault();
      state = setPaused(state, !state.paused);
      controls.setPaused(state.paused);
      break;
    case "KeyS":
      if (!state.paused) {
        state = setPaused(state, true);
        controls.setPaused(true);
      }
      state = stepOnce(state, plant);
      break;
    case "KeyR":
      state = resetSimulation(state, plant);
      controls.setMission(state.mission?.mission.id ?? null);
      controls.setWaypoint(state.setpoint);
      cameraRig.reset(drone.group.position.clone());
      break;
    case "KeyF":
      follow = !follow;
      controls.setFollow(follow);
      break;
    case "KeyA":
      showAxes = !showAxes;
      drone.axes.visible = showAxes;
      legend.setVisible(showAxes);
      controls.setShowAxes(showAxes);
      break;
    default:
      break;
  }
});
