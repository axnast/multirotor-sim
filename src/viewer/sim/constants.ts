/** Shared timing constants for the browser simulation runner. */

/** Fixed simulator step (s). The core is always advanced at this rate. */
export const SIM_DT = 1e-3;

/** Telemetry and trail are sampled together every 20 simulator steps (20 ms). */
export const SAMPLE_INTERVAL_STEPS = 20;

/** 60 s of telemetry at 20 ms cadence. */
export const TELEMETRY_CAPACITY = 60 * 1000 / SAMPLE_INTERVAL_STEPS;

/** Default trail length: 12 s at 20 ms cadence. */
export const TRAIL_CAPACITY = 600;

/** Maximum core steps executed in one animation frame. */
export const DEFAULT_MAX_STEPS_PER_FRAME = 256;

/** Largest real frame delta consumed before the accumulator is allowed to cap. */
export const MAX_FRAME_SECONDS = 0.25;

/** Speed multiplier bounds. */
export const SPEED_MIN = 0.25;
export const SPEED_MAX = 4;
