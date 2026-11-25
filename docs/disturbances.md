# Disturbances (Milestone 5)

All disturbance models are deterministic, seeded and plain data. No controller
or airframe physics was changed in M5. The controller and mixer always receive
the **nominal** config and params; the plant alone receives the **true** config
(mass, COM, inertia, voltage-limited motor ceilings) and the wind vector.

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
splitmix32 offsets of the scenario seed (`seeds.ts`); toggling one model does
not shift the other's samples. With every model disabled, the runner passes the
same objects to the same core calls, so the 45 s M3 flight digest is unchanged
(`8ed20e2e…` from pre-M5 HEAD `15cd29b`).

The battery is a placeholder, not a chemistry model. The realistic pack never
saturates a 4:1 thrust-to-weight hex in hover (margin 2.57 at SoC 0). The named
**stress pack** (`V_oc/V_nom = 0.45 + 0.55·SoC`) is deliberately non-physical and
exists only to make saturation observable in tests/demo/viewer; at SoC 0.15 its
hover margin is 0.751 and plant saturation grows while the nominal mixer stays
saturated=false. Terminal voltage solves `V² - V_oc V + P R/η = 0` exactly with
`P = Σ kQ ω³`. SoC drains as the integrated current divided by capacity.

## Current controller limitations (measured, not fixed in M5)

The controller was deliberately not changed. The following are baseline
observations and M6 targets, not correct behaviour:

- **Integral separation freezes steady errors above 0.1 m.** For centred +20%
  mass, the required extra acceleration is `a_z ≈ 0.2·g = 1.96 m/s²`, which at
  `ki = 0.01` needs an integral of **≈196 m·s**. Starting from 5 m, the vehicle
  reaches the ground long before then (integral only 71.454 m·s at touchdown).
  The question "why does it not lift off later on the ground if the integral
  should exceed 196 m·s after ~40 s at e = 5 m?" is answered by the integration
  condition: at touchdown `error = +5 m` and velocity is zero or positive, so
  `neither |error| < 0.1 nor error·v <= 0` holds and the integral is held. It
  stays frozen at 71.454 m·s for the remaining 100 s. Setting
  `integralSeparation = 1e9` lets it grow past 195 m·s and the vehicle climbs
  back to ≈4.96 m by 120 s; `ki × 10` alone settles at 2.60 m (also frozen);
  widening `maxAccelZ` alone changes nothing. In words: **integral separation
  is the switch that changes the outcome; ki or the acceleration limit do not.**
- **Small mismatches converge slowly or not at all.** A centred +1% mass gives
  a residual z error of 0.202 m that is never re-entered within 600 s
  (integral stuck at 1.495 m·s). With `integralSeparation = 1e9`, the band is
  re-entered at t = 30.488 s and final z = 5.000 m.
- **Constant wind does not return to the setpoint either.** A steady 5 m/s +x
  wind produces the drag-predicted tilt `atan(1.75/14.715) = 6.782°` but a
  ~3.67 m steady x offset, for the same integral-separation reason. The earlier
  assumption that the position integrator removes wind offsets is **false**
  with the shipped default.
- **The inner rate loop is soft for large COM offsets.** For the 2 cm offset
  case the disturbance torque is ≈0.35 N·m; with P-only trim the offset would
  be `α_d/(kp_rate·tiltGain)`. The measured rate-integral torque ratio reaches
  0.5 at 3.6 s and 0.9 at 9.9 s, with a peak attitude excursion of **109°**
  (mostly during the initial mass drop) before the level trim settles. At
  0.01 kg / 0.3 m (d = 2 mm) the steady attitude is level (<1°) with a 7.9 rad/s
  motor-speed spread, but the lateral position remains at −0.19 m.
- **Survival envelope:** with the default controller, only ≤0.2% mass with
  ≤1 mm COM offset survives the 120 s / 0.1 m envelope; +20% / 2 cm lands.
  These are recorded as baseline limitations in
  `tests/disturbancePayload.test.ts`, with expectations that can be flipped
  when the controller improves.

No tolerance was loosened and no controller gain was retuned to hide these
results. See [control.md](control.md) for the loop equations.
