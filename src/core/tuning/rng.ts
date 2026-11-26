/**
 * Small, dependency-free PRNG for the tuner. xoshiro128** has a 128-bit state
 * stored as four uint32 numbers, so a checkpoint is plain JSON (no bigint) and
 * a resumed stream is bit-identical to an uninterrupted one.
 */

export type RngState = readonly [number, number, number, number];

function rotl(value: number, shift: number): number {
  return ((value << shift) | (value >>> (32 - shift))) >>> 0;
}

/** Expand a numeric seed into a non-zero xoshiro128** state with splitmix32. */
export function seedRng(seed: number): RngState {
  if (!Number.isSafeInteger(seed)) throw new RangeError("seed must be an integer");
  let mix = seed >>> 0;
  const words: number[] = [];
  for (let index = 0; index < 4; index++) {
    mix = (mix + 0x9e3779b9) >>> 0;
    let value = mix;
    value = Math.imul(value ^ (value >>> 16), 0x21f0aaad);
    value = Math.imul(value ^ (value >>> 15), 0x735a2d97);
    words.push((value ^ (value >>> 15)) >>> 0);
  }
  if (words.reduce((acc, word) => acc | word, 0) === 0) words[0] = 1;
  return [words[0]!, words[1]!, words[2]!, words[3]!];
}

/** One xoshiro128** step; neither the input nor the output state is mutated. */
export function nextUint32(state: RngState): { value: number; state: RngState } {
  const [s0, s1, s2, s3] = state;
  const value = (Math.imul(rotl(Math.imul(s1, 5), 7), 9)) >>> 0;
  const t = (s1 << 9) >>> 0;
  const nextS2 = s2 ^ s0;
  const nextS3 = s3 ^ s1;
  const nextS1 = s1 ^ nextS2;
  const nextS0 = s0 ^ nextS3;
  return {
    value,
    state: [
      nextS0 >>> 0,
      nextS1 >>> 0,
      (nextS2 ^ t) >>> 0,
      rotl(nextS3, 11),
    ],
  };
}

/** Standard Box-Muller transform, no cached spare (so the state is minimal). */
export function nextGaussian(state: RngState): { value: number; state: RngState } {
  const first = nextUint32(state);
  const second = nextUint32(first.state);
  const u1 = (first.value + 0.5) / 4_294_967_296;
  const u2 = (second.value + 0.5) / 4_294_967_296;
  return {
    value: Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2),
    state: second.state,
  };
}
