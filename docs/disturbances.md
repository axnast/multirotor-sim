# Disturbances (Milestone 5)

All disturbance models are deterministic, seeded and plain data. The controller
and mixer always receive the **nominal** config and params; the plant alone
receives the **true** config (mass, COM, inertia, voltage-limited motor ceilings)
and the wind vector. The controller fix documented in
[control.md](control.md) removes the frozen-integral equilibrium without
changing the M3 flight profile.

## Models and defaults

| Model | Files | Defaults |
|---|---|---|
| PRNG | `src/core/disturbance/prng.ts` | xoshiro128**, splitmix32 seeding, Box–Muller with cached spare |
| Wind | `wind.ts` | OU per world axis: `a = exp(-dt/Tc)`, `x' = a x + σ√(1-a²)N`; Tc 2 s; vertical σ = 0.3·horizontal; optional 1-cosine gust |
| Sensors | `noise.ts` | position σ 0.02 m, velocity σ 0.05 m/s, attitude σ 0.2°, gyro white σ 0.005 rad/s, bias walk 2e-5 rad/s/√s |
| Battery | `battery.ts` | 6S-style placeholder: V_nom 22.2, realistic V_oc/V_nom = 0.85 + 0.15·SoC, R 0.15 Ω, η 0.75, 2.2 Ah |
| Payload | `payload.ts` | point mass; `d = mp·r_p/(m0+mp)`; `I' = I0 + m0·PA(d) + mp·PA(r_p-d)` |
| Scenarios | `scenario.ts` | calm 0/0, light 2/0.5, moderate 5/1.5, gusty 5/3, strong 8/2.5 m/s (mean/gust σ, +x) |

Wind and sensors use **independent PRNG streams** derived from distinct
splitmix32 offsets of the scenario seed (`seeds.ts`). With every model disabled,
the runner passes the same objects to the same core calls, so the 45 s M3 flight
digest is unchanged (`8ed20e2e…` from pre-M5 HEAD `15cd29b`). The integral
release never fires on that flight profile, so the digest remains unchanged
after the fix commit (`525c80f`).

The battery is a placeholder, not a chemistry model. The realistic pack never
saturates a 4:1 thrust-to-weight hex in hover (margin 2.57 at SoC 0). The named
**stress pack** (`V_oc/V_nom = 0.45 + 0.55·SoC`) is deliberately non-physical and
exists only to make saturation observable in tests/demo/viewer; at SoC 0.15 its
hover margin is 0.751. Terminal voltage solves `V² - V_oc V + P R/η = 0`
exactly with `P = Σ kQ ω³`.

## Constant wind: noise off vs noise on

True position error, 5 fixed seeds (1–5), 120 s runs, mean over the final 10 s
(range across seeds in brackets). Three controller variants:

- **M3 gate**: `integralDwell = 0` (the pre-fix gate, bit-identical to M3).
- **default**: the release is active (`integralDwell = 5 s`, `integralTrimGain = 3`).
- **gate off**: `integralSeparation = 1e9`, plain conditional integration,
  **reference only** — it overshoots a 5 m step by 12.7 % and is not the M3 baseline.

| Constant 5 m/s +x | M3 gate | default | gate off |
|---|---|---|---|
| Sensors **off**, 60 s | 3.681 | 0.0617 | 0.4699 |
| Sensors **off**, 120 s | 3.680 | **0.0028** | 0.0173 |
| Sensors **on**, 60 s | 2.396 [2.392–2.404] | 1.075 [1.067–1.083] | 0.469 [0.466–0.472] |
| Sensors **on**, 120 s | 1.045 [1.040–1.049] | **0.0109** [0.006–0.013] | 0.0154 [0.014–0.018] |

Noise changes the M3 gate qualitatively: the perturbed equilibrium no longer
holds the integral (Ix ≈ −95 rather than −25.8) and the offset decays slowly,
whereas noise-off is permanently frozen at 3.68 m. The release reaches <0.05 m
by 120 s with noise; at 60 s the noisy residual is still 1.075 m, so the test
uses 120 s. The steady tilt is the drag prediction
`atan(1.75/(1.5·9.81)) = 6.782°` in all cases.

## Named scenarios (5 seeds, 120 s)

True-position RMS (m), mean [min, max] across seeds 1–5:

| Scenario | Window | M3 gate | default |
|---|---|---|---|
| light | 20–60 s | 0.919 [0.786, 0.996] | 0.863 [0.651, 0.996] |
| light | 90–120 s | 0.418 [0.313, 0.553] | 0.369 [0.299, 0.493] |
| moderate | 20–60 s | 2.956 [2.295, 3.421] | 2.957 [2.298, 3.421] |
| moderate | 90–120 s | 1.531 [1.098, 2.160] | 1.529 [1.096, 2.162] |
| gusty | 20–60 s | 4.043 [2.484, 5.427] | 4.044 [2.486, 5.427] |
| gusty | 90–120 s | 2.859 [1.917, 4.052] | 2.859 [1.916, 4.055] |
| strong | 20–60 s | 5.895 [4.310, 7.072] | 5.896 [4.316, 7.072] |
| strong | 90–120 s | 3.201 [2.147, 4.840] | 3.199 [2.147, 4.840] |

Noise on/off changes these RMS values by <0.002 m. The release makes essentially
no difference to moderate/gusty/strong: their residual is driven by the wind
process (OU with Tc = 2 s gives roughly five independent samples per 10 s
window, so single-seed numbers have wide spread), **not** by the frozen
integrator. A mean-velocity trigger was also evaluated and left 2.5–3.6 m under
gusts; gust rejection is an M6 target, not a claimed M5 fix.

## Payload envelope

Survive = no ground contact in 120 s **and** true position error <0.1 m over the
last 10 s. Rows = payload mass as a fraction of 1.5 kg; columns = COM offset.
Re-pinned after the controller fix:

| mass % \ offset | 2 mm | 5 mm | 10 mm | 20 mm |
|---|---|---|---|---|
| 1 % | yes | yes | yes | no (contact) |
| 2 % | yes | yes | yes | no (contact) |
| 5 % | yes | yes | yes | no (contact) |
| 10 % | yes | yes | yes | no (contact) |
| 20 % | no (contact) | no (contact) | no (contact) | no (contact) |

`PAYLOAD_SMALL` (0.003 kg, 1 mm COM) survives. `PAYLOAD_LARGE_STRESS` (0.3 kg,
2 cm COM) now **recovers** from 4.99 m to 0.003 m (integral ≈196 m·s) but still
contacts the ground for ~7.7k steps during the transient. The 20 mm column and
all 20 % cases remain M6 targets for the inner loop; no tolerance was loosened.

Earlier baseline measurements kept for context (M3 gate): centred +20 % mass
froze at an integral of 71.454 m·s on the ground; centred 1 % mass froze at
0.202 m error (integral 1.495 m·s); constant wind froze at 3.68 m. The 2 cm
offset case has a disturbance torque ≈0.35 N·m, rate-integral torque reaching
50 % at 3.6 s and 90 % at 9.9 s, with a 109° peak attitude excursion during the
initial mass drop.

See [control.md](control.md) for the release equations, the M6 warning about the
discontinuous tuning cost surface, and the unchanged M3 numbers.
