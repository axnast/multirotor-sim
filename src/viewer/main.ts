/**
 * Viewer bootstrap. The Three.js scene and UI are added in later Milestone 4
 * commits; this scaffold verifies that Vite builds a static site from src/core.
 */

import "./style.css";

const app = document.querySelector<HTMLDivElement>("#app");
if (app === null) {
  throw new Error("Missing #app root element");
}

app.innerHTML = `
  <div class="boot">
    <div>
      <h1>multirotor-sim viewer</h1>
      <p>Vite scaffold ready — 3D scene arrives in the next commit.</p>
    </div>
  </div>
`;
