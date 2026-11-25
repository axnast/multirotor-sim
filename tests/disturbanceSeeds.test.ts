import { describe, expect, it } from "vitest";

import { nextUint32, createPrng } from "../src/core/disturbance/prng";
import {
  deriveDisturbanceSeeds,
  deriveStreamSeed,
  SENSOR_SEED_OFFSET,
  WIND_SEED_OFFSET,
} from "../src/core/disturbance/seeds";
import {
  createDisturbanceState,
} from "../src/core/disturbance/step";
import { disturbanceParamsFor } from "../src/core/disturbance/scenario";

function firstOutputs(seed: number, count: number): number[] {
  let state = createPrng(seed);
  const values: number[] = [];
  for (let i = 0; i < count; i++) {
    const next = nextUint32(state);
    values.push(next.value);
    state = next.state;
  }
  return values;
}

describe("independent PRNG streams for each disturbance model", () => {
  it("derives wind and sensors from distinct splitmix32 offsets", () => {
    expect(WIND_SEED_OFFSET).not.toBe(SENSOR_SEED_OFFSET);
    const seeds = deriveDisturbanceSeeds(12345);
    expect(seeds.wind).not.toBe(seeds.sensors);
    expect(seeds.wind).toBe(deriveStreamSeed(12345, WIND_SEED_OFFSET));
    expect(seeds.sensors).toBe(deriveStreamSeed(12345, SENSOR_SEED_OFFSET));
    // Different scenario seeds give different streams.
    expect(deriveDisturbanceSeeds(12346).wind).not.toBe(seeds.wind);
  });

  it("produces first outputs that differ and are not shifted copies", () => {
    const seeds = deriveDisturbanceSeeds(12345);
    const wind = firstOutputs(seeds.wind, 12);
    const sensors = firstOutputs(seeds.sensors, 12);
    expect(wind[0]).not.toBe(sensors[0]);
    for (let shift = 0; shift < 12; shift++) {
      let shiftedCopy = true;
      for (let i = 0; i < 12 - shift; i++) {
        if (wind[i + shift] !== sensors[i]) {
          shiftedCopy = false;
          break;
        }
      }
      expect(shiftedCopy).toBe(false);
    }
  });

  it("actually initialises the wind and sensor states from those seeds", () => {
    const params = disturbanceParamsFor("gusty", 12345, { sensors: true });
    const state = createDisturbanceState(params);
    const seeds = deriveDisturbanceSeeds(12345);
    expect(state.wind.prng).toEqual(createPrng(seeds.wind));
    expect(state.sensors.prng).toEqual(createPrng(seeds.sensors));
  });
});
