/**
 * Small deterministic PRNG for all Milestone 5 disturbance models.
 *
 * xoshiro128** with splitmix32 seeding: 32-bit unsigned integer arithmetic
 * implemented with `Math.imul` and `>>> 0`, so it is exactly reproducible on
 * every JavaScript engine. No `Math.random` is used anywhere.
 *
 * Gaussians use Box–Muller with a cached spare value, so two uniforms produce
 * two independent samples. `Math.log`, `Math.cos` and `Math.sin` are not
 * required to be correctly rounded across engines: identical bytes are
 * guaranteed for the same engine, seed and input sequence (documented in
 * docs/disturbances.md).
 */

import type { Vec3 } from "../math/vec3";
import type { PrngState } from "./types";

const TWO_32 = 4294967296;

/** Rotate the low 32 bits left by `k`. */
function rotl(value: number, k: number): number {
  return ((value << k) | (value >>> (32 - k))) >>> 0;
}

/**
 * Seed four xoshiro words from one 32-bit integer with splitmix32.
 * Seed 0 is valid and produces a non-degenerate stream.
 */
export function createPrng(seed: number): PrngState {
  if (!Number.isInteger(seed) || seed < 0 || seed > 0xffffffff) {
    throw new RangeError("seed must be a 32-bit unsigned integer");
  }
  let x = seed >>> 0;
  const splitmix32 = (): number => {
    x = (x + 0x9e3779b9) >>> 0;
    let z = x;
    z = Math.imul(z ^ (z >>> 16), 0x21f0aaad) >>> 0;
    z = Math.imul(z ^ (z >>> 15), 0x735a2d97) >>> 0;
    return (z ^ (z >>> 15)) >>> 0;
  };
  return {
    s: [splitmix32(), splitmix32(), splitmix32(), splitmix32()],
    spare: 0,
    hasSpare: false,
  };
}

/** One xoshiro128** step. The input state is never mutated. */
export function nextUint32(
  state: PrngState,
): { readonly state: PrngState; readonly value: number } {
  const s0 = state.s[0];
  const s1 = state.s[1];
  const s2 = state.s[2];
  const s3 = state.s[3];
  const result = Math.imul(rotl(Math.imul(s1, 5) >>> 0, 7), 9) >>> 0;
  const t = (s1 << 9) >>> 0;
  const nextS2 = (s2 ^ s0) >>> 0;
  const nextS3 = (s3 ^ s1) >>> 0;
  const nextS1 = (s1 ^ nextS2) >>> 0;
  const nextS0 = (s0 ^ nextS3) >>> 0;
  return {
    state: {
      s: [nextS0, nextS1, (nextS2 ^ t) >>> 0, rotl(nextS3, 11)],
      spare: state.spare,
      hasSpare: state.hasSpare,
    },
    value: result,
  };
}

/** Uniform in the open interval (0, 1). */
export function nextUniform(
  state: PrngState,
): { readonly state: PrngState; readonly value: number } {
  const next = nextUint32(state);
  return { state: next.state, value: (next.value + 0.5) / TWO_32 };
}

/** Standard normal N(0, 1) via Box–Muller, with one cached spare. */
export function nextGaussian(
  state: PrngState,
): { readonly state: PrngState; readonly value: number } {
  if (state.hasSpare) {
    return { state: { ...state, spare: 0, hasSpare: false }, value: state.spare };
  }
  const u1 = nextUniform(state);
  const u2 = nextUniform(u1.state);
  const radius = Math.sqrt(-2 * Math.log(u1.value));
  const angle = 2 * Math.PI * u2.value;
  return {
    state: {
      ...u2.state,
      spare: radius * Math.sin(angle),
      hasSpare: true,
    },
    value: radius * Math.cos(angle),
  };
}

/** Three independent standard normals, in x/y/z order. */
export function nextGaussian3(
  state: PrngState,
): { readonly state: PrngState; readonly values: Vec3 } {
  const x = nextGaussian(state);
  const y = nextGaussian(x.state);
  const z = nextGaussian(y.state);
  return {
    state: z.state,
    values: [x.value, y.value, z.value],
  };
}
