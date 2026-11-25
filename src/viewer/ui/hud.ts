/** Compact live HUD: time, speed, tracking error, tilt and saturation. */

import { quatRotateVector } from "../../core/math/quat";
import { positionErrorM } from "../sim/metrics";
import type { SimulationState } from "../sim/runner";

export interface HudView {
  readonly element: HTMLElement;
  update(state: SimulationState): void;
}

export function createHud(parent: HTMLElement): HudView {
  const element = document.createElement("aside");
  element.id = "hud";
  element.innerHTML = `
    <div class="hud-title">Flight telemetry</div>
    <div class="hud-grid">
      <span>Time</span><strong data-hud="time">0.0 s</strong>
      <span>Speed</span><strong data-hud="speed">1×</strong>
      <span>Position error</span><strong data-hud="error">0.00 m</strong>
      <span>Tilt (max)</span><strong data-hud="tilt">0.0° (0.0°)</strong>
      <span>Mixer</span><strong data-hud="mix">OK</strong>
      <span>Ground</span><strong data-hud="ground">airborne</strong>
    </div>
  `;
  parent.appendChild(element);
  const field = (name: string): HTMLElement => {
    const found = element.querySelector<HTMLElement>(`[data-hud="${name}"]`);
    if (found === null) throw new Error(`missing HUD field ${name}`);
    return found;
  };
  const time = field("time");
  const speed = field("speed");
  const error = field("error");
  const tilt = field("tilt");
  const mix = field("mix");
  const ground = field("ground");

  return {
    element,
    update(state: SimulationState): void {
      const positionError = positionErrorM(
        state.multirotor.body.position,
        state.setpoint.position,
      );
      const degrees = 180 / Math.PI;
      const currentTilt = quatRotateVector(
        state.multirotor.body.orientation,
        [0, 0, 1],
      );
      const tiltRad = Math.acos(
        Math.min(1, Math.max(-1, currentTilt[2])),
      );
      time.textContent = `${state.simTime.toFixed(2)} s`;
      speed.textContent = state.paused
        ? "paused"
        : `${state.speed}×${state.cappedLastFrame ? " (capped)" : ""}`;
      error.textContent = `${positionError.toFixed(3)} m`;
      tilt.textContent =
        `${(tiltRad * degrees).toFixed(2)}° ` +
        `(${(state.stats.maxTiltRad * degrees).toFixed(2)}°)`;
      const saturated = state.lastMix?.saturated ?? false;
      mix.textContent = saturated
        ? `SATURATED (${state.stats.saturationSteps})`
        : `OK (${state.stats.saturationSteps})`;
      mix.classList.toggle("bad", saturated);
      ground.textContent = state.contact ? "contact" : "airborne";
      ground.classList.toggle("bad", state.contact);
    },
  };
}
