/** DOM control panel: missions, playback, waypoint, camera and gain sliders. */

import type { ControllerParams, Setpoint } from "../../core/control/types";
import { MISSIONS, type Mission, type MissionId } from "../sim/mission";
import {
  PARAM_GROUPS,
  PARAM_SLIDERS,
  type ParamKey,
} from "./paramSpecs";

export interface ControlCallbacks {
  onMission(mission: Mission): void;
  onPause(paused: boolean): void;
  onStep(): void;
  onReset(): void;
  onSpeed(speed: number): void;
  onAltitude(altitude: number): void;
  onYaw(yawRadians: number): void;
  onFollow(follow: boolean): void;
  onShowAxes(visible: boolean): void;
  onTrailCapacity(capacity: number): void;
  onSlider(id: ParamKey, value: number): void;
  onResetGains(): void;
}

export interface ControlsView {
  readonly element: HTMLElement;
  setMission(id: MissionId | null): void;
  setPaused(paused: boolean): void;
  setFollow(follow: boolean): void;
  setShowAxes(visible: boolean): void;
  setSpeed(speed: number): void;
  setParams(params: ControllerParams): void;
  setWaypoint(setpoint: Setpoint): void;
  getWaypointInputs(): { readonly altitude: number; readonly yawRadians: number };
  setStatus(text: string): void;
}

const SPEEDS: readonly number[] = [0.25, 0.5, 1, 2, 4];
const TRAIL_CAPACITIES: readonly number[] = [200, 600, 1200, 3000];

function makeButton(label: string, title?: string): HTMLButtonElement {
  const button = document.createElement("button");
  button.type = "button";
  button.textContent = label;
  if (title !== undefined) button.title = title;
  return button;
}

function decimalsFor(step: number): number {
  if (step < 0.01) return 3;
  if (step < 0.1) return 2;
  if (step < 1) return 1;
  return 0;
}

/** Drive the CSS gradient so the filled portion of a range slider is visible. */
function updateRangeFill(input: HTMLInputElement): void {
  const min = Number(input.min);
  const max = Number(input.max);
  const value = Number(input.value);
  const fraction = max === min ? 0 : (value - min) / (max - min);
  input.style.setProperty("--fill", `${(fraction * 100).toFixed(2)}%`);
}

function createControls(
  parent: HTMLElement,
  params: ControllerParams,
  callbacks: ControlCallbacks,
): ControlsView {
  const element = document.createElement("aside");
  element.id = "control-panel";
  parent.appendChild(element);

  const title = document.createElement("div");
  title.className = "panel-title";
  title.innerHTML = "multirotor-sim <small>cascaded PID viewer</small>";
  element.appendChild(title);

  const status = document.createElement("div");
  status.id = "viewer-status";
  element.appendChild(status);

  let paused = false;
  const missionButtons = new Map<MissionId, HTMLButtonElement>();
  const missionSection = document.createElement("section");
  missionSection.innerHTML = "<h2>Mission</h2>";
  const missionRow = document.createElement("div");
  missionRow.className = "button-row";
  for (const mission of Object.values(MISSIONS)) {
    const button = makeButton(mission.name, mission.description);
    button.addEventListener("click", () => callbacks.onMission(mission));
    missionButtons.set(mission.id, button);
    missionRow.appendChild(button);
  }
  missionSection.appendChild(missionRow);
  element.appendChild(missionSection);

  const playback = document.createElement("section");
  playback.innerHTML = "<h2>Playback</h2>";
  const playbackRow = document.createElement("div");
  playbackRow.className = "button-row";
  const pauseButton = makeButton("Pause", "Space");
  pauseButton.addEventListener("click", () => callbacks.onPause(!paused));
  const stepButton = makeButton("Step", "Single 1 ms step");
  stepButton.addEventListener("click", () => callbacks.onStep());
  const resetButton = makeButton("Reset flight", "R");
  resetButton.addEventListener("click", () => callbacks.onReset());
  playbackRow.append(pauseButton, stepButton, resetButton);
  playback.appendChild(playbackRow);

  const speedRow = document.createElement("div");
  speedRow.className = "button-row speed-row";
  const speedButtons = new Map<number, HTMLButtonElement>();
  for (const speed of SPEEDS) {
    const button = makeButton(`${speed}×`);
    button.addEventListener("click", () => callbacks.onSpeed(speed));
    speedButtons.set(speed, button);
    speedRow.appendChild(button);
  }
  playback.appendChild(speedRow);
  element.appendChild(playback);

  const waypoint = document.createElement("section");
  waypoint.innerHTML =
    "<h2>Waypoint</h2><p class='hint'>Click the ground to set x/y. Sliders set z and yaw.</p>";
  const altitudeInput = document.createElement("input");
  altitudeInput.type = "range";
  altitudeInput.min = "0";
  altitudeInput.max = "10";
  altitudeInput.step = "0.1";
  altitudeInput.value = "5";
  const altitudeOutput = document.createElement("output");
  const altitudeRow = document.createElement("label");
  altitudeRow.className = "slider-row";
  altitudeRow.append(
    Object.assign(document.createElement("span"), { textContent: "Altitude" }),
    altitudeInput,
    altitudeOutput,
  );
  altitudeInput.addEventListener("input", () => {
    updateRangeFill(altitudeInput);
    callbacks.onAltitude(Number(altitudeInput.value));
  });

  const yawInput = document.createElement("input");
  yawInput.type = "range";
  yawInput.min = "-180";
  yawInput.max = "180";
  yawInput.step = "1";
  yawInput.value = "0";
  const yawOutput = document.createElement("output");
  const yawRow = document.createElement("label");
  yawRow.className = "slider-row";
  yawRow.append(
    Object.assign(document.createElement("span"), { textContent: "Yaw" }),
    yawInput,
    yawOutput,
  );
  yawInput.addEventListener("input", () => {
    updateRangeFill(yawInput);
    callbacks.onYaw((Number(yawInput.value) * Math.PI) / 180);
  });
  waypoint.append(altitudeRow, yawRow);
  element.appendChild(waypoint);

  const camera = document.createElement("section");
  camera.innerHTML = "<h2>Camera &amp; display</h2>";
  const followLabel = document.createElement("label");
  followLabel.className = "check-row";
  const followInput = document.createElement("input");
  followInput.type = "checkbox";
  followInput.addEventListener("change", () => callbacks.onFollow(followInput.checked));
  followLabel.append(followInput, document.createTextNode(" Follow drone (F)"));

  const axesLabel = document.createElement("label");
  axesLabel.className = "check-row";
  const axesInput = document.createElement("input");
  axesInput.type = "checkbox";
  axesInput.checked = true;
  axesInput.addEventListener("change", () => callbacks.onShowAxes(axesInput.checked));
  axesLabel.append(axesInput, document.createTextNode(" Body axes (A)"));

  const trailLabel = document.createElement("label");
  trailLabel.className = "select-row";
  const trailSelect = document.createElement("select");
  for (const capacity of TRAIL_CAPACITIES) {
    const option = document.createElement("option");
    option.value = String(capacity);
    option.textContent = `${capacity} points`;
    if (capacity === 600) option.selected = true;
    trailSelect.appendChild(option);
  }
  trailSelect.addEventListener("change", () =>
    callbacks.onTrailCapacity(Number(trailSelect.value)),
  );
  trailLabel.append(document.createTextNode("Trail"), trailSelect);
  camera.append(followLabel, axesLabel, trailLabel);
  element.appendChild(camera);

  const gains = document.createElement("section");
  gains.innerHTML = "<h2>Gains</h2>";
  const sliderInputs = new Map<ParamKey, HTMLInputElement>();
  const sliderOutputs = new Map<ParamKey, HTMLOutputElement>();
  for (const group of PARAM_GROUPS) {
    const details = document.createElement("details");
    details.open = group.id === "position";
    const summary = document.createElement("summary");
    summary.textContent = group.label;
    details.appendChild(summary);
    for (const spec of PARAM_SLIDERS.filter((candidate) => candidate.group === group.id)) {
      const row = document.createElement("label");
      row.className = "slider-row";
      const input = document.createElement("input");
      input.type = "range";
      input.min = String(spec.min);
      input.max = String(spec.max);
      input.step = String(spec.step);
      input.value = String(spec.get(params));
      const output = document.createElement("output");
      input.addEventListener("input", () => {
        updateRangeFill(input);
        callbacks.onSlider(spec.id, Number(input.value));
      });
      row.append(
        Object.assign(document.createElement("span"), { textContent: spec.label }),
        input,
        output,
      );
      details.appendChild(row);
      sliderInputs.set(spec.id, input);
      sliderOutputs.set(spec.id, output);
    }
    gains.appendChild(details);
  }
  const resetGains = makeButton("Reset gains to defaults");
  resetGains.className = "wide-button";
  resetGains.addEventListener("click", () => callbacks.onResetGains());
  gains.appendChild(resetGains);
  element.appendChild(gains);

  const setParams = (next: ControllerParams): void => {
    for (const spec of PARAM_SLIDERS) {
      const input = sliderInputs.get(spec.id);
      const output = sliderOutputs.get(spec.id);
      if (input === undefined || output === undefined) continue;
      const value = spec.get(next);
      input.value = String(value);
      updateRangeFill(input);
      output.textContent = `${value.toFixed(decimalsFor(spec.step))} ${spec.unit}`;
    }
  };
  const setWaypoint = (setpoint: Setpoint): void => {
    const altitude = setpoint.position[2];
    altitudeInput.value = String(altitude);
    updateRangeFill(altitudeInput);
    altitudeOutput.textContent = `${altitude.toFixed(1)} m`;
    const yawDegrees = (setpoint.yaw * 180) / Math.PI;
    yawInput.value = String(yawDegrees);
    updateRangeFill(yawInput);
    yawOutput.textContent = `${yawDegrees.toFixed(0)}°`;
  };

  setParams(params);
  setWaypoint({ position: [0, 0, 5], yaw: 0 });

  return {
    element,
    setMission(id: MissionId | null): void {
      for (const [missionId, button] of missionButtons) {
        button.classList.toggle("active", missionId === id);
      }
    },
    setPaused(next: boolean): void {
      paused = next;
      pauseButton.textContent = paused ? "Resume" : "Pause";
      pauseButton.classList.toggle("active", paused);
    },
    setFollow(follow: boolean): void {
      followInput.checked = follow;
    },
    setShowAxes(visible: boolean): void {
      axesInput.checked = visible;
    },
    setSpeed(speed: number): void {
      for (const [value, button] of speedButtons) {
        button.classList.toggle("active", value === speed);
      }
    },
    setParams,
    setWaypoint,
    getWaypointInputs(): { readonly altitude: number; readonly yawRadians: number } {
      return {
        altitude: Number(altitudeInput.value),
        yawRadians: (Number(yawInput.value) * Math.PI) / 180,
      };
    },
    setStatus(text: string): void {
      status.textContent = text;
    },
  };
}

export { createControls };
