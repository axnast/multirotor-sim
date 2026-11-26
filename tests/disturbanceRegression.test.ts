import { describe, expect, it } from "vitest";

import { isDisturbanceActive } from "../src/core/disturbance/step";
import {
  buildDisturbanceParams,
  DEFAULT_VIEWER_DISTURBANCE,
} from "../src/viewer/sim/disturbanceConfig";
import {
  digestM3Samples,
  FROZEN_M3_DIGEST,
  M3_SAMPLE_STEPS,
  M3_STEPS,
  runLegacyTrace,
  runRunnerTrace,
} from "./helpers/legacyTrace";

/**
 * Constraint 2: with every disturbance disabled, the M5 code path must be
 * bit-identical to the pre-M5 code path. The digest was captured from the
 * untouched HEAD 15cd29b and hashes Float64LE bytes, not a rounded CSV.
 */
describe("zero-disturbance M3 flight regression", () => {
  it("keeps the pre-M5 helper honest against the frozen digest", () => {
    expect(digestM3Samples(runLegacyTrace())).toBe(FROZEN_M3_DIGEST);
  }, 60_000);

  it("reproduces the frozen flight bit-for-bit through the runner", () => {
    const { samples, state } = runRunnerTrace();
    expect(M3_STEPS % M3_SAMPLE_STEPS).toBe(0);
    expect(samples.length).toBe(M3_STEPS / M3_SAMPLE_STEPS + 1);
    expect(digestM3Samples(samples)).toBe(FROZEN_M3_DIGEST);
    expect(samples).toEqual(runLegacyTrace());
    expect(state.stepCount).toBe(M3_STEPS);
  }, 60_000);

  it("keeps the digest with the viewer's all-off disturbance settings", () => {
    const params = buildDisturbanceParams(DEFAULT_VIEWER_DISTURBANCE);
    expect(isDisturbanceActive(params)).toBe(false);
    const { samples, state } = runRunnerTrace(params);
    expect(digestM3Samples(samples)).toBe(FROZEN_M3_DIGEST);
    expect(samples).toEqual(runLegacyTrace());
    expect(state.disturbance).toBe(null);
    expect(state.diagnostics).toBe(null);
  }, 60_000);
});
