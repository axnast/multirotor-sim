/** Small on-screen legend so body-axis handedness can be checked by eye. */

export interface AxisLegend {
  readonly element: HTMLElement;
  setVisible(visible: boolean): void;
}

export function createAxisLegend(parent: HTMLElement): AxisLegend {
  const element = document.createElement("div");
  element.id = "axis-legend";
  element.innerHTML = `
    <div class="axis-legend-title">Body axes · FLU / right-handed</div>
    <div><span class="axis-swatch axis-x"></span>X forward = red</div>
    <div><span class="axis-swatch axis-y"></span>Y left = green</div>
    <div><span class="axis-swatch axis-z"></span>Z up = blue</div>
  `;
  parent.appendChild(element);
  return {
    element,
    setVisible(visible: boolean): void {
      element.style.display = visible ? "block" : "none";
    },
  };
}
