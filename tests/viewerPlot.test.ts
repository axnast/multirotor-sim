import { describe, expect, it } from "vitest";

import {
  buildPlotFrame,
  unwrapDegrees,
  type PlotCommandLine,
  type PlotPoint,
  type PlotSeries,
  type PlotSpec,
} from "../src/viewer/plots/plotModel";

function series(
  points: readonly (readonly [number, number])[],
  overrides: Partial<PlotSeries> = {},
): PlotSeries {
  return {
    id: "s",
    label: "s",
    color: "#f87171",
    dashed: false,
    unwrap: false,
    points: points.map(([t, value]) => ({ t, value })),
    ...overrides,
  };
}

function spec(seriesList: readonly PlotSeries[], overrides: Partial<PlotSpec> = {}): PlotSpec {
  return {
    width: 300,
    height: 120,
    now: 1,
    windowSeconds: 1,
    title: "Test",
    unit: "m",
    minSpan: 1,
    series: seriesList,
    ...overrides,
  };
}

function seriesLine(
  commands: readonly { readonly type: string }[],
  color: string,
): readonly (readonly [number, number])[] {
  for (const command of commands) {
    if (
      command.type === "line" &&
      "stroke" in command &&
      command.stroke === color &&
      "points" in command
    ) {
      return command.points as readonly (readonly [number, number])[];
    }
  }
  throw new Error(`no series line with color ${color}`);
}

describe("pure plot model", () => {
  it("auto-scales with 10% padding", () => {
    const frame = buildPlotFrame(
      spec([series([[0, 1], [1, 3]])]),
    );
    expect(frame.yRange[0]).toBeCloseTo(0.8, 9);
    expect(frame.yRange[1]).toBeCloseTo(3.2, 9);
  });

  it("expands flat data to minSpan", () => {
    const frame = buildPlotFrame(
      spec([series([[0, 5], [1, 5]])]),
    );
    expect(frame.yRange[0]).toBeCloseTo(4.5, 9);
    expect(frame.yRange[1]).toBeCloseTo(5.5, 9);
  });

  it("keeps empty data finite", () => {
    const frame = buildPlotFrame(spec([series([])]));
    expect(frame.yRange.every(Number.isFinite)).toBe(true);
    expect(frame.yRange[1]! - frame.yRange[0]!).toBeGreaterThanOrEqual(1);
  });

  it("drops points outside the rolling window", () => {
    const frame = buildPlotFrame(
      spec([series([[-1, 0], [0, 1], [0.5, 2], [0.9, 3], [2, 4]])]),
    );
    expect(seriesLine(frame.commands, "#f87171").length).toBe(3);
    expect(frame.xRange[0]).toBe(0);
    expect(frame.xRange[1]).toBeCloseTo(1, 9);
  });

  it("maps time to the horizontal plot rectangle", () => {
    const frame = buildPlotFrame(
      spec([series([[0, 1], [1, 3]])]),
    );
    const pixels = seriesLine(frame.commands, "#f87171");
    expect(pixels[0]![0]).toBeCloseTo(frame.plot.x, 9);
    expect(pixels[1]![0]).toBeCloseTo(frame.plot.x + frame.plot.width, 9);
  });

  it("unwraps yaw across +180°", () => {
    const points: readonly PlotPoint[] = [
      { t: 0, value: 179 },
      { t: 1, value: -179 },
      { t: 2, value: -175 },
    ];
    expect(unwrapDegrees(points).map((point) => point.value)).toEqual([179, 181, 185]);
  });

  it("unwraps yaw across -180°", () => {
    const points: readonly PlotPoint[] = [
      { t: 0, value: -179 },
      { t: 1, value: 179 },
      { t: 2, value: 175 },
    ];
    expect(unwrapDegrees(points).map((point) => point.value)).toEqual([-179, -181, -185]);
  });

  it("uses unwrapped values for yaw axis scaling", () => {
    const points = [
      [0, 170],
      [1, -170],
    ] as const;
    const wrapped = buildPlotFrame(spec([series(points)]));
    const unwrapped = buildPlotFrame(
      spec([series(points, { unwrap: true })]),
    );
    expect(wrapped.yRange[1]! - wrapped.yRange[0]!).toBeGreaterThan(300);
    expect(unwrapped.yRange[1]! - unwrapped.yRange[0]!).toBeLessThan(30);
  });

  it("skips non-finite samples", () => {
    const frame = buildPlotFrame(
      spec([series([[0, 1], [0.5, Number.NaN], [1, 2]])]),
    );
    expect(seriesLine(frame.commands, "#f87171").length).toBe(2);
    expect(frame.yRange.every(Number.isFinite)).toBe(true);
  });

  it("distinguishes dotted measured series from dashed setpoints", () => {
    const frame = buildPlotFrame(
      spec([
        series([[0, 1], [1, 2]], { label: "x meas", color: "#fb7185", dotted: true }),
        series([[0, 3], [1, 4]], { label: "x sp", color: "#60a5fa", dashed: true }),
      ]),
    );
    const lines = frame.commands.filter(
      (command): command is PlotCommandLine => command.type === "line",
    );
    const measured = lines.find((line) => line.stroke === "#fb7185");
    const setpoint = lines.find((line) => line.stroke === "#60a5fa");
    expect(measured?.dotted).toBe(true);
    expect(measured?.dash).toBe(false);
    expect(setpoint?.dash).toBe(true);
  });

  it("emits a legend and clip commands", () => {
    const frame = buildPlotFrame(
      spec([
        series([[0, 1], [1, 2]], { label: "actual", dashed: false }),
        series([[0, 1], [1, 2]], { label: "sp", color: "#60a5fa", dashed: true }),
      ]),
    );
    const texts = frame.commands.filter((command) => command.type === "text" && "text" in command);
    expect(texts.some((command) => "text" in command && command.text === "actual")).toBe(true);
    expect(texts.some((command) => "text" in command && command.text === "sp")).toBe(true);
    expect(frame.commands.some((command) => command.type === "clip")).toBe(true);
    expect(frame.commands.some((command) => command.type === "unclip")).toBe(true);
  });

  it("draws a single visible sample as a circle command", () => {
    const frame = buildPlotFrame(
      spec([series([[0.5, 1]])], { windowSeconds: 1 }),
    );
    expect(frame.commands.some((command) => command.type === "circle")).toBe(true);
  });
});
