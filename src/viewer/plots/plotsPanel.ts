/** DOM panel holding the three live canvas plots. */

import type { SimulationState } from "../sim/runner";
import { ringToArray } from "../sim/ringBuffer";
import type { TelemetrySample } from "../sim/telemetry";
import { drawPlotFrame } from "./plotCanvas";
import { buildPlotFrame, type PlotPoint, type PlotSeries } from "./plotModel";

const DEGREES = 180 / Math.PI;
const COLORS: readonly string[] = [
  "#38bdf8", "#fb7185", "#4ade80", "#fbbf24", "#a78bfa", "#22d3ee",
  "#f472b6", "#facc15",
];

function pointsOf(
  samples: readonly TelemetrySample[],
  value: (sample: TelemetrySample) => number,
): readonly PlotPoint[] {
  return samples.map((sample) => ({ t: sample.t, value: value(sample) }));
}

function series(
  id: string,
  label: string,
  color: string,
  dashed: boolean,
  unwrap: boolean,
  points: readonly PlotPoint[],
  dotted = false,
): PlotSeries {
  return { id, label, color, dashed, dotted, unwrap, points };
}

/** Noisy measurement samples, only present while the sensor model is active. */
function measuredPoints(
  samples: readonly TelemetrySample[],
  axis: 0 | 1 | 2,
): readonly PlotPoint[] {
  const points: PlotPoint[] = [];
  for (const sample of samples) {
    const measured = sample.measuredPosition;
    if (measured !== null) points.push({ t: sample.t, value: measured[axis]! });
  }
  return points;
}

export interface PlotsPanel {
  readonly element: HTMLElement;
  update(state: SimulationState): void;
}

export function createPlotsPanel(
  parent: HTMLElement,
  motorCount: number,
): PlotsPanel {
  const element = document.createElement("aside");
  element.id = "plot-panel";
  parent.appendChild(element);

  const header = document.createElement("div");
  header.className = "plot-header";
  header.innerHTML = "<strong>Live telemetry</strong>";
  const windowSelect = document.createElement("select");
  for (const seconds of [2, 5, 10]) {
    const option = document.createElement("option");
    option.value = String(seconds);
    option.textContent = `${seconds} s`;
    if (seconds === 5) option.selected = true;
    windowSelect.appendChild(option);
  }
  header.appendChild(windowSelect);
  element.appendChild(header);

  const positionCanvas = document.createElement("canvas");
  const attitudeCanvas = document.createElement("canvas");
  const motorCanvas = document.createElement("canvas");
  for (const canvas of [positionCanvas, attitudeCanvas, motorCanvas]) {
    canvas.className = "plot-canvas";
    element.appendChild(canvas);
  }

  return {
    element,
    update(state: SimulationState): void {
      const samples = ringToArray(state.telemetry);
      const now = state.simTime;
      const windowSeconds = Number(windowSelect.value);

      const positionSeries: PlotSeries[] = [
        series("x", "x", "#f87171", false, false, pointsOf(samples, (s) => s.position[0])),
        series("y", "y", "#4ade80", false, false, pointsOf(samples, (s) => s.position[1])),
        series("z", "z", "#60a5fa", false, false, pointsOf(samples, (s) => s.position[2])),
        series("x sp", "x sp", "#fca5a5", true, false, pointsOf(samples, (s) => s.positionSp[0])),
        series("y sp", "y sp", "#86efac", true, false, pointsOf(samples, (s) => s.positionSp[1])),
        series("z sp", "z sp", "#93c5fd", true, false, pointsOf(samples, (s) => s.positionSp[2])),
      ];
      if (state.disturbanceParams?.sensors.enabled === true) {
        // Dotted measured position over solid true position, position plot only.
        positionSeries.push(
          series("x meas", "x meas", "#f87171", false, false, measuredPoints(samples, 0), true),
          series("y meas", "y meas", "#4ade80", false, false, measuredPoints(samples, 1), true),
          series("z meas", "z meas", "#60a5fa", false, false, measuredPoints(samples, 2), true),
        );
      }
      const attitudeSeries: readonly PlotSeries[] = [
        series("roll", "roll", "#f87171", false, false, pointsOf(samples, (s) => s.euler[0] * DEGREES)),
        series("pitch", "pitch", "#4ade80", false, false, pointsOf(samples, (s) => s.euler[1] * DEGREES)),
        series("yaw", "yaw", "#60a5fa", false, true, pointsOf(samples, (s) => s.euler[2] * DEGREES)),
        series("roll sp", "roll sp", "#fca5a5", true, false, pointsOf(samples, (s) => s.eulerSp[0] * DEGREES)),
        series("pitch sp", "pitch sp", "#86efac", true, false, pointsOf(samples, (s) => s.eulerSp[1] * DEGREES)),
        series("yaw sp", "yaw sp", "#93c5fd", true, true, pointsOf(samples, (s) => s.eulerSp[2] * DEGREES)),
      ];
      const motorSeries: readonly PlotSeries[] = Array.from(
        { length: motorCount },
        (_, index) =>
          series(
            `m${index}`,
            `M${index + 1}`,
            COLORS[index % COLORS.length]!,
            false,
            false,
            pointsOf(samples, (s) => s.motorSpeeds[index] ?? 0),
          ),
      );

      drawPlotFrame(
        positionCanvas,
        buildPlotFrame({
          width: positionCanvas.clientWidth,
          height: positionCanvas.clientHeight,
          now,
          windowSeconds,
          title: "Position",
          unit: "m",
          minSpan: 0.05,
          series: positionSeries,
        }),
      );
      drawPlotFrame(
        attitudeCanvas,
        buildPlotFrame({
          width: attitudeCanvas.clientWidth,
          height: attitudeCanvas.clientHeight,
          now,
          windowSeconds,
          title: "Attitude (yaw unwrapped)",
          unit: "deg",
          minSpan: 1,
          series: attitudeSeries,
        }),
      );
      drawPlotFrame(
        motorCanvas,
        buildPlotFrame({
          width: motorCanvas.clientWidth,
          height: motorCanvas.clientHeight,
          now,
          windowSeconds,
          title: "Motor speeds",
          unit: "rad/s",
          minSpan: 100,
          series: motorSeries,
        }),
      );
    },
  };
}
