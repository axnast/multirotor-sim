/** Thin DPR-aware canvas executor for the pure plot commands. */

import type { PlotCommand, PlotFrame } from "./plotModel";

function drawCommand(ctx: CanvasRenderingContext2D, command: PlotCommand): void {
  switch (command.type) {
    case "rect":
      ctx.fillStyle = command.fill;
      ctx.fillRect(command.x, command.y, command.width, command.height);
      break;
    case "line":
      if (command.points.length < 2) break;
      ctx.strokeStyle = command.stroke;
      ctx.lineWidth = command.lineWidth;
      ctx.setLineDash(command.dash ? [4, 3] : []);
      ctx.beginPath();
      ctx.moveTo(command.points[0]![0], command.points[0]![1]);
      for (let index = 1; index < command.points.length; index++) {
        ctx.lineTo(command.points[index]![0], command.points[index]![1]);
      }
      ctx.stroke();
      ctx.setLineDash([]);
      break;
    case "text":
      ctx.fillStyle = command.fill;
      ctx.font = command.font;
      ctx.textAlign = command.align;
      ctx.textBaseline = "alphabetic";
      ctx.fillText(command.text, command.x, command.y);
      break;
    case "circle":
      ctx.fillStyle = command.fill;
      ctx.beginPath();
      ctx.arc(command.x, command.y, command.radius, 0, 2 * Math.PI);
      ctx.fill();
      break;
    case "clip":
      ctx.save();
      ctx.beginPath();
      ctx.rect(command.x, command.y, command.width, command.height);
      ctx.clip();
      break;
    case "unclip":
      ctx.restore();
      break;
  }
}

/** Render a PlotFrame into a canvas, keeping CSS-pixel geometry. */
export function drawPlotFrame(
  canvas: HTMLCanvasElement,
  frame: PlotFrame,
): void {
  const ctx = canvas.getContext("2d");
  if (ctx === null) return;
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  if (width === 0 || height === 0) return;
  const pixelWidth = Math.round(width * dpr);
  const pixelHeight = Math.round(height * dpr);
  if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
    canvas.width = pixelWidth;
    canvas.height = pixelHeight;
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  for (const command of frame.commands) drawCommand(ctx, command);
}
