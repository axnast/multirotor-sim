# multirotor-sim

A multirotor (hexacopter) flight simulator with a cascaded PID controller and an
automatic PID tuner.

> **Status:** work in progress. Nothing is implemented yet, see the roadmap.

## Goals

- Simulate rigid-body hexacopter dynamics (6-DOF, motor lag, drag, wind).
- Fly it with a cascaded controller (position → velocity → attitude → rate).
- Automatically find good PID gains by optimising a step-response cost.
- Visualise flights in a 3D browser viewer.

## Roadmap

- [ ] Rigid-body dynamics + motor model + mixer
- [ ] PID class (anti-windup, filtered derivative)
- [ ] Cascaded flight controller
- [ ] Simulation loop + logging
- [ ] Autotuner (cost function + optimiser)
- [ ] Plots / metrics (rise time, overshoot, settling time)
- [ ] 3D browser visualisation
- [ ] Wind gusts and robustness tests

## Install (development)

```bash
git clone https://github.com/axnast/multirotor-sim.git
cd multirotor-sim
python -m venv .venv
source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -e ".[dev]"
pytest
```

## Project layout

```
src/multirotor/   library code
tests/            pytest tests
examples/         runnable demo scripts
docs/             notes, derivations, images
```

## Contributing / workflow

- Work on a branch per feature: `git checkout -b feature/<name>`
- Small commits with clear messages (`Add rigid-body dynamics`)
- Open a pull request into `main`; CI runs `pytest`

## License

MIT, see [LICENSE](LICENSE).
