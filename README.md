# multirotor-sim

A multirotor flight simulator with a cascaded PID controller and an
automatic PID tuner.

> **Status:** Milestone 1 (rigid-body physics) is complete. The core is pure,
> deterministic TypeScript with 31 passing tests. No motors, controller, or 3D
> viewer yet; see the roadmap.

## Goals

- Simulate rigid-body multirotor dynamics (6-DOF, motor lag, drag, wind).
- Fly it with a cascaded controller (position → velocity → attitude → rate).
- Automatically find good PID gains by optimising a step-response cost.
- Visualise flights in a 3D browser viewer.

## Roadmap

- [x] **M1 — Rigid-body physics:** 6-DOF state, Newton–Euler dynamics with the
      gyroscopic term, RK4/Euler integrators, conservation test suite
- [ ] **M2 — Multirotor model:** motor thrust/torque from RPM², quad/hex/octo
      layouts, mixer matrix, motor lag, drag
- [ ] **M3 — Cascaded PID controller:** rate → angle → position loops,
      filtered derivative, anti-windup, waypoints
- [ ] **M4 — 3D viewer:** Three.js scene, trail, live plots, gain sliders
- [ ] **M5 — Disturbances:** wind gusts, sensor noise, battery sag, payload shift
- [ ] **M6–7 — Autotuner:** step-response scoring, CMA-ES / Bayesian
      optimisation, Ziegler–Nichols baseline, before/after comparison
- [ ] **Polish:** README GIF, GitHub Pages demo, "tune your own drone" config

## Install (development)

Requires Node 20 or newer.

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
```

CI runs `npm ci`, `npm run typecheck`, and `npm test` on Node 20, 22, and 24.

## Project layout

```
src/core/math/   pure Vec3, Quat, Mat3 primitives
src/core/        rigid-body dynamics, integrators, energy diagnostics
tests/           Vitest unit and physics tests
scripts/         runnable demo scripts
docs/            conventions and physics notes
```

## Contributing / workflow

- Work on a branch per feature: `git checkout -b feature/<name>`
- Small commits with Conventional Commit messages (`feat(core): rigid body dynamics`)
- Open a pull request into `main`; CI runs typecheck and tests

## License

MIT, see [LICENSE](LICENSE).
