/**
 * Pure data → draw-command conversion for the live plots. No DOM or Canvas
 * types here, so scaling, windowing and yaw unwrapping are unit-testable.
 */

export interface PlotPoint {
  readonly t: number;
  readonly value: number;
}

export interface PlotSeries {
  readonly id: string;
  readonly label: string;
  readonly color: string;
  readonly dashed: boolean;
  /** Unwrap degree-valued series across the ±180° boundary. */
  readonly unwrap: boolean;
  readonly points: readonly PlotPoint[];
}

export interface PlotSpec {
  readonly width: number;
  readonly height: number;
  readonly now: number;
  readonly windowSeconds: number;
  readonly title: string;
  readonly unit: string;
  readonly minSpan: number;
  readonly series: readonly PlotSeries[];
}

export interface PlotCommandRect {
  readonly type: "rect";
  readonly x: number;
  readonly y: number;
  readonly width: number;
  readonly height: number;
  readonly fill: string;
}

export interface PlotCommandLine {
  readonly type: "line";
  readonly points: readonly (readonly [number, number])[];
  readonly stroke: string;
  readonly lineWidth: number;
  readonly dash: boolean;
}

export interface PlotCommandText {
  readonly type: "text";
  readonly x: number;
  readonly y: number;
  readonly text: string;
  readonly fill: string;
  readonly font: string;
  readonly align: "left" | "right";
}

export interface PlotCommandCircle {
  readonly type: "circle";
  readonly x: number;
  readonly y: number;
  readonly radius: number;
  readonly fill: string;
}

export interface PlotCommandClip {
  readonly type: "clip";
  readonly x: number;
  readonly y: number;
  readonly width: number;
  readonly height: number;
}

export interface PlotCommandUnclip {
  readonly type: "unclip";
}

export type PlotCommand =
  | PlotCommandRect
  | PlotCommandLine
  | PlotCommandText
  | PlotCommandCircle
  | PlotCommandClip
  | PlotCommandUnclip;

export interface PlotTick {
  readonly value: number;
  readonly y: number;
  readonly label: string;
}

export interface PlotFrame {
  readonly plot: { readonly x: number; readonly y: number; readonly width: number; readonly height: number };
  readonly xRange: readonly [number, number];
  readonly yRange: readonly [number, number];
  readonly ticks: readonly PlotTick[];
  readonly commands: readonly PlotCommand[];
}

/** Convert [179°, -179°] into [179°, 181°] (and the reverse for -180°). */
export function unwrapDegrees(points: readonly PlotPoint[]): readonly PlotPoint[] {
  if (points.length === 0) return points;
  let previous = points[0]!.value;
  return points.map((point, index) => {
    if (index === 0) return point;
    let value = point.value;
    const delta = value - previous;
    if (delta > 180) value -= 360;
    else if (delta < -180) value += 360;
    previous = value;
    return { t: point.t, value };
  });
}

function formatTick(value: number, span: number): string {
  if (!Number.isFinite(value)) return "";
  const decimals = span >= 100 ? 0 : span >= 10 ? 1 : span >= 1 ? 2 : 3;
  return value.toFixed(decimals);
}

/** Build device-independent draw commands from telemetry series. */
export function buildPlotFrame(spec: PlotSpec): PlotFrame {
  const padding = { left: 44, right: 8, top: 28, bottom: 18 };
  const plot = {
    x: padding.left,
    y: padding.top,
    width: Math.max(1, spec.width - padding.left - padding.right),
    height: Math.max(1, spec.height - padding.top - padding.bottom),
  };
  const start = Math.max(0, spec.now - spec.windowSeconds);
  const end = Math.max(start + 1e-6, spec.now);
  const xToPx = (t: number): number =>
    plot.x + ((t - start) / (end - start)) * plot.width;

  const visible = spec.series.map((series) => {
    const values = series.unwrap
      ? unwrapDegrees(series.points)
      : series.points;
    return {
      series,
      points: values.filter(
        (point) =>
          Number.isFinite(point.t) &&
          Number.isFinite(point.value) &&
          point.t >= start - 1e-9 &&
          point.t <= end + 1e-9,
      ),
    };
  });

  const finiteValues = visible.flatMap((entry) =>
    entry.points.map((point) => point.value),
  );
  let low = finiteValues.length > 0 ? Math.min(...finiteValues) : -spec.minSpan / 2;
  let high = finiteValues.length > 0 ? Math.max(...finiteValues) : spec.minSpan / 2;
  const initialSpan = high - low;
  const pad = initialSpan * 0.1;
  low -= pad;
  high += pad;
  if (high - low < spec.minSpan) {
    const centre = (low + high) / 2;
    low = centre - spec.minSpan / 2;
    high = centre + spec.minSpan / 2;
  }
  const span = Math.max(high - low, 1e-12);
  const yToPx = (value: number): number =>
    plot.y + plot.height - ((value - low) / span) * plot.height;

  const commands: PlotCommand[] = [
    {
      type: "rect",
      x: 0,
      y: 0,
      width: spec.width,
      height: spec.height,
      fill: "rgba(15, 23, 42, 0.55)",
    },
    {
      type: "text",
      x: 4,
      y: 12,
      text: spec.title,
      fill: "#cbd5e1",
      font: "600 11px ui-sans-serif, system-ui",
      align: "left",
    },
    {
      type: "text",
      x: spec.width - 4,
      y: 12,
      text: spec.unit,
      fill: "#94a3b8",
      font: "10px ui-sans-serif, system-ui",
      align: "right",
    },
  ];

  const tickCount = 4;
  const ticks: PlotTick[] = [];
  for (let index = 0; index <= tickCount; index++) {
    const value = low + (span * index) / tickCount;
    const y = yToPx(value);
    ticks.push({ value, y, label: formatTick(value, span) });
    commands.push({
      type: "line",
      points: [[plot.x, y], [plot.x + plot.width, y]],
      stroke: "rgba(148, 163, 184, 0.14)",
      lineWidth: 1,
      dash: false,
    });
    commands.push({
      type: "text",
      x: plot.x - 6,
      y: y + 3,
      text: formatTick(value, span),
      fill: "#94a3b8",
      font: "9px ui-sans-serif, system-ui",
      align: "right",
    });
  }
  for (const [t, align] of [
    [start, "left"],
    [end, "right"],
  ] as const) {
    const x = xToPx(t);
    commands.push({
      type: "line",
      points: [[x, plot.y], [x, plot.y + plot.height]],
      stroke: "rgba(148, 163, 184, 0.14)",
      lineWidth: 1,
      dash: false,
    });
    commands.push({
      type: "text",
      x: align === "left" ? x : x,
      y: spec.height - 4,
      text: `${t.toFixed(1)}s`,
      fill: "#94a3b8",
      font: "9px ui-sans-serif, system-ui",
      align,
    });
  }

  commands.push({ type: "clip", ...plot });
  for (const entry of visible) {
    const pixels = entry.points.map(
      (point) => [xToPx(point.t), yToPx(point.value)] as const,
    );
    if (pixels.length === 1) {
      commands.push({
        type: "circle",
        x: pixels[0]![0],
        y: pixels[0]![1],
        radius: 2,
        fill: entry.series.color,
      });
    } else if (pixels.length > 1) {
      commands.push({
        type: "line",
        points: pixels,
        stroke: entry.series.color,
        lineWidth: entry.series.dashed ? 1 : 1.6,
        dash: entry.series.dashed,
      });
    }
  }

  commands.push({ type: "unclip" });

  // Legend in one row at the top right, laid out right-to-left.
  let cursor = spec.width - 6;
  for (const series of [...spec.series].reverse()) {
    const estimatedWidth = series.label.length * 5.4 + 18;
    cursor -= estimatedWidth;
    if (cursor < plot.x) break;
    commands.push({
      type: "rect",
      x: cursor,
      y: 17,
      width: 10,
      height: 3,
      fill: series.color,
    });
    commands.push({
      type: "text",
      x: cursor + 14,
      y: 22,
      text: series.label,
      fill: series.color,
      font: "9px ui-sans-serif, system-ui",
      align: "left",
    });
  }

  return {
    plot,
    xRange: [start, end],
    yRange: [low, high],
    ticks,
    commands,
  };
}
