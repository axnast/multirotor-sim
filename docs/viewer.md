# Browser 3D viewer (Milestones 4–5)

The viewer is a presentation layer over the unchanged simulator core. It runs
the real `controllerStepDetailed` → `mixDetailed` → `stepMultirotorDetailed`
sequence and never re-implements physics or control.

## Layout

| Area | Module |
|---|---|
| Bootstrap and rAF loop | `src/viewer/main.ts` |
| Fixed-step runner, missions, metrics | `src/viewer/sim/` |
| Coordinate conversion | `src/viewer/coords.ts` |
| Three.js scene, drone, camera, trail | `src/viewer/scene/` |
| Pure plot model and canvas renderer | `src/viewer/plots/` |
| DOM controls and HUD | `src/viewer/ui/` |
| Pure disturbance settings builder | `src/viewer/sim/disturbanceConfig.ts` |
| World-frame wind arrow | `src/viewer/scene/windArrow.ts` |

`src/core` stays DOM-free and compiles under the original `tsconfig.json`.
`tsconfig.viewer.json` adds DOM and `vite/client` for the viewer only; both
configs are checked by `npm run typecheck`.

## Runner

`src/viewer/sim/runner.ts` is pure: no DOM, no `requestAnimationFrame`, no wall
clock. It takes a real elapsed time from the caller and consumes whole 1 ms
steps with an accumulator:

```
accumulator += min(realDt, 0.25 s) · speed
while accumulator ≥ 1 ms and steps < 256:
    one core step; accumulator -= 1 ms
excess whole steps are dropped and counted, not carried
```

Speeds are 0.25×–4×. Pause freezes the accumulator; Step advances exactly one
1 ms step (and auto-pauses). The 256-step cap and 0.25 s frame clamp prevent a
slow or resumed tab from spiralling; `capped` and `droppedSteps` are reported.

Telemetry **and the trail** are sampled inside the runner every 20 core steps
(20 ms), so they are deterministic and independent of the render frame rate.
The samples live in fixed-capacity ring buffers (60 s telemetry, 600-point
default trail).

Missions are declarative timelines with integer step counts:

- **Hover**: start and hold `[0, 0, 5]`.
- **5 m step**: start `[0, 0, 5]`, target `[5, 0, 5]`.
- **Square**: four 5 m corners, 20 s per leg, hold at the last corner.
- **Takeoff**: start on the ground with zero motors, target `[0, 0, 5]`.

Selecting a mission resets the flight and unpauses. Reset preserves pause and
speed. Manual waypoints keep the last mission as `homeMission`.

## Coordinate conversion

All conversions live in `src/viewer/coords.ts`. Core world is right-handed Z-up
FLU; Three.js is right-handed Y-up. The rotation

```
C = [ 1  0  0 ]      p_three = (x, z, -y)
    [ 0  0  1 ]      q_three = [x, z, -y, w]   (Three x,y,z,w)
    [ 0 -1  0 ]
```

has `det(C)=+1`, so handedness is preserved: body `+x` → Three `+x`, body `+y`
→ Three `-z`, body `+z` → Three `+y`. Tests round-trip vectors and
quaternions, check lengths and handedness, and verify known roll/pitch/yaw
rotations and composition order.

The permanent yellow nose marker sits on the `+x` arm. It is attached to the
airframe, not the toggleable axis arrows. The body-axis triad is red `+x`,
green `+y`, blue `+z`, mirrored by the on-screen legend.

## Scene

The hex model is built from `MultirotorConfig`: arms, hubs and rotor discs are
placed at the motor positions through `coords.ts`. Rotor colour encodes spin
(+1 cyan, −1 rose), brightness and opacity encode speed, and a blade stripe
rotates at an illustrative capped rate (real rotor speeds alias at 60 Hz).
The mesh is lifted by a small visual landing-gear offset so it clears the
ground plane; **physics still uses the centre of mass at `z = 0`**.

The trail uses the runner's ring buffer; the amber setpoint marker shows a
ring, a yaw arrow and an altitude guide. Orbit controls have damping and a
follow mode that translates both target and camera by the same smoothed delta,
preserving the user's orbit offset.

## Controls, plots and HUD

- Mission buttons, pause/step/reset, speed buttons (0.25×–4×).
- Click the ground plane to command x/y; altitude (0–10 m) and yaw (−180°…180°)
  sliders set z and heading. Any manual change switches to Manual mode.
- 21 gain sliders with visible values and units, grouped by loop, plus
  **Reset gains to defaults**. The integral separation, dwell, release guard and
  trim-gain sliders are included. `ki` changes rescale stored integrals by
  `ki_old / ki_new` (`src/viewer/sim/bumpless.ts`), so `ki·integral` is
  preserved and live tuning is bumpless without changing core behaviour.
- **Disturbances** section: scenario select (calm/light/moderate/gusty/strong),
  seed, sensor-noise toggle, battery toggle with realistic/stress pack choice
  and starting-SoC slider, and a **Drop payload** button that installs
  `PAYLOAD_LARGE_STRESS` (and toggles to **Remove payload**). All defaults are
  off, so the runner stays on the bit-identical pre-M5 path. Changing the
  scenario, seed or toggles restarts the disturbance streams; dropping the
  payload uses `installPayload()` and preserves the flight and running streams.
  A payload installed before reset is installed again at t = 0 by the reset.
- Plots are pure `buildPlotFrame(spec) → draw commands` (`plotModel.ts`); the
  canvas file only applies DPR and executes commands. Three canvases show
  position x/y/z, attitude roll/pitch/yaw and motor speeds, setpoint dashed
  versus actual solid, with auto-scaled axes, a legend and a 2/5/10 s window.
  When sensor noise is on, the position plot adds dotted measured-position
  overlays (x/y/z) from the telemetry samples; the other two plots are
  unchanged. Yaw is unwrapped across ±180° so a shortest-path yaw command does
  not draw a false vertical jump; this is unit tested.
- The HUD shows sim time, speed, position error, current/max tilt, mixer
  saturation count, world wind vector, battery SoC/voltage, plant saturation
  count and ground contact. The status line and HUD use the same
  `positionErrorM` helper and the same throttled state sample, so they cannot
  disagree by a frame. An arrow in the 3D scene follows the drone and shows the
  current wind vector; it is hidden when wind is off.

## Build and deployment

```bash
npm run dev      # http://localhost:5173/
npm run build    # static site in dist/
npm run preview  # serve dist/ locally
```

`vite.config.ts` reads `VITE_BASE`, defaulting to `/`. The Pages workflow sets
`VITE_BASE=/<repo-name>/` and publishes `dist/` on every push to `main` using
Node 22. For a custom domain, set the base to `/` instead. To enable this, go
to **Settings → Pages → Build and deployment → Source: GitHub Actions**.

The separate `tests` workflow still runs typecheck and the fast suite on Node
20/22/24, plus `npm run build` on Node 22 so a broken viewer is caught before
deployment. `npm run test:slow` runs in its own Node 22 job on pushes to `main`
only, with a 25-minute timeout.

## Tests

Node-only Vitest tests cover the fixed-step runner (determinism, accumulator,
speed, cap, pause, reset, direct-core equivalence), coordinate conversion,
ring buffer, bumpless gains, plot scaling and yaw unwrapping (including dotted
measured series), slider specs, the disturbance-settings builder, the mid-flight
payload clock, and the shared error metric. The frozen M3 digest is also checked
through the viewer's all-off disturbance settings. No browser is required.

## Limitations

- Telemetry is sampled every 20 sim steps, not every render frame; at 4× the
  plotted window still represents sim time correctly.
- Rotor spin is illustrative, not physical-rate accurate.
- M5 disturbances are available in the viewer; no autotuner yet (M6).
- Changing scenario/seed/toggles mid-flight restarts the disturbance streams:
  the wind gust sequence and sensor PRNG are re-seeded, though the flight and
  setpoint continue. The payload button deliberately does not restart them.
- Follow mode smooths the target, so during very fast manoeuvres the drone can
  briefly lead the camera.
