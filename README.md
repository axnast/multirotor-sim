# multirotor-sim

A multirotor flight simulator with a cascaded PID controller and an
automatic PID tuner.

> **Status:** Milestone 4 (browser 3D viewer) is complete. The core is pure,
> deterministic TypeScript. At the time of writing `npm test` reports 237
> passing tests and 1 deliberately skipped (no motor-failure model); run it for
> the current count. No autotuner yet; see the roadmap.

<!-- Plot placeholder: once the PNG exists, replace this comment with
     ![Hover, takeoff and roll-pulse trace](docs/hover-demo.png) -->

## Goals

- Simulate rigid-body multirotor dynamics (6-DOF, motor lag, drag, wind).
- Fly it with a cascaded controller (position → velocity → attitude → rate).
- Automatically find good PID gains by optimising a step-response cost.
- Visualise flights in a 3D browser viewer.

## Roadmap

- [x] **M1 — Rigid-body physics:** 6-DOF state, Newton–Euler dynamics with the
      gyroscopic term, RK4/Euler integrators, conservation test suite
- [x] **M2 — Multirotor model:** motor thrust/torque from RPM², quad/hex/octo
      layouts, mixer matrix, motor lag, drag, ground contact
- [x] **M3 — Cascaded PID controller:** rate → angle → position loops,
      filtered derivative, anti-windup, waypoints
- [x] **M4 — 3D viewer:** Three.js scene, trail, live plots, gain sliders
- [ ] **M5 — Disturbances:** wind gusts, sensor noise, battery sag, payload shift
- [ ] **M6–7 — Autotuner:** step-response scoring, CMA-ES / Bayesian
      optimisation, Ziegler–Nichols baseline, before/after comparison
- [ ] **Polish:** README GIF, "tune your own drone" config

## Install (development)

Requires Node 20.19 or newer (Vite 8).

```bash
git clone https://github.com/axnast/multirotor-sim.git
cd multirotor-sim
npm install
```

## How to run tests

```bash
npm test           # full Vitest suite (single run)
npm run test:watch # watch mode during development
npm run typecheck  # strict TypeScript check
npm run demo       # 10 s tumbling-box CSV trace -> out/drop-box.csv
npm run demo:hover  # 3 x 10 s hover, takeoff and roll-pulse trace -> out/hover.csv
npm run demo:flight # hex takeoff, square waypoints and hold -> out/flight.csv
npm run dev        # browser viewer at http://localhost:5173/
npm run build      # static viewer site in dist/
npm run preview    # serve the built site locally
```

`npm run demo:hover` writes a 1 kHz, 41-column CSV (30 003 rows) with
commanded and achieved wrenches, ground impulses and energies; the model and
sign conventions are documented in [docs/multirotor.md](docs/multirotor.md).
`npm run demo:flight` writes a 1 kHz, 35-column CSV of setpoint and actual
flight states under the cascaded controller; see [docs/control.md](docs/control.md).

CI runs `npm ci`, `npm run typecheck`, and `npm test` on Node 20, 22, and 24,
plus `npm run build` on Node 22. A separate workflow builds with Vite's
configurable `VITE_BASE` and publishes the viewer to GitHub Pages on every push
to `main`.

## 3D viewer

[Live demo](https://axnast.github.io/multirotor-sim/) · [viewer notes](docs/viewer.md)

<!-- Live demo link is a placeholder; replace it with the deployed Pages URL if different. -->
<!-- ![3D viewer, trail and live plots](docs/viewer-demo.gif) -->

The viewer runs the same core simulator and cascaded controller at a fixed
1 kHz. It shows the hex airframe built from its config, spin-direction-coloured
rotors, body axes with a permanent +x nose marker, a trail, an amber setpoint
marker, orbit/follow camera, live plots and a HUD.

```bash
npm run dev      # http://localhost:5173/
npm run build    # dist/ (set VITE_BASE for GitHub Pages)
npm run preview  # serve dist/ locally
```

Use the mission buttons (Hover, 5 m step, Square, Takeoff), click the ground to
set a waypoint, adjust the altitude/yaw sliders, change gain sliders live, or
pick a speed from 0.25× to 4×. `Space` pauses, `S` steps one millisecond,
`R` resets the flight, `F` toggles follow camera and `A` toggles the body axes.
Gain changes are bumpless: stored integrals are rescaled so `ki·integral` is
preserved. See [docs/viewer.md](docs/viewer.md) for the runner, coordinate
mapping and deployment details.

## Project layout

```
src/core/math/        pure Vec3, Quat, Mat3 primitives
src/core/             rigid-body dynamics, integrators, energy diagnostics
src/core/multirotor/  config, motors, mixer, dynamics, ground contact
src/core/control/     PID, attitude, position loops and the cascade
src/viewer/           Three.js viewer, runner, plots and controls
tests/                Vitest unit and physics tests
scripts/              runnable demo scripts
docs/                 conventions, physics, multirotor, control and viewer notes
index.html            Vite entry point for the viewer
vite.config.ts        base path and build configuration
out/                  generated CSV traces (gitignored)
dist/                 built static viewer (gitignored)
```

## Contributing / workflow

- Work on a branch per feature: `git checkout -b feature/<name>`
- Small commits with Conventional Commit messages (`feat(core): rigid body dynamics`)
- Open a pull request into `main`; CI runs typecheck and tests

## License

MIT, see [LICENSE](LICENSE).
