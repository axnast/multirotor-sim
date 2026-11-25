/**
 * Per-model PRNG streams derived from one scenario seed.
 *
 * Each model has its own stream so enabling or disabling one disturbance does
 * not shift the samples consumed by another. The offsets are distinct
 * splitmix32 constants and are applied to `seed + offset` through the same
 * splitmix32 finaliser used to seed xoshiro128**.
 */

/** Offset for the wind OU stream. */
export const WIND_SEED_OFFSET = 0x9e3779b9;
/** Offset for the sensor-noise stream. */
export const SENSOR_SEED_OFFSET = 0x85ebca6b;

function assertSeed(seed: number): void {
  if (!Number.isInteger(seed) || seed < 0 || seed > 0xffffffff) {
    throw new RangeError("scenario seed must be a 32-bit unsigned integer");
  }
}

/** splitmix32 finaliser applied to `seed + offset` (modulo 2^32). */
export function deriveStreamSeed(scenarioSeed: number, offset: number): number {
  assertSeed(scenarioSeed);
  let z = (scenarioSeed + offset) >>> 0;
  z = Math.imul(z ^ (z >>> 16), 0x21f0aaad) >>> 0;
  z = Math.imul(z ^ (z >>> 15), 0x735a2d97) >>> 0;
  return (z ^ (z >>> 15)) >>> 0;
}

export interface DisturbanceSeeds {
  readonly wind: number;
  readonly sensors: number;
}

export function deriveDisturbanceSeeds(scenarioSeed: number): DisturbanceSeeds {
  return {
    wind: deriveStreamSeed(scenarioSeed, WIND_SEED_OFFSET),
    sensors: deriveStreamSeed(scenarioSeed, SENSOR_SEED_OFFSET),
  };
}
