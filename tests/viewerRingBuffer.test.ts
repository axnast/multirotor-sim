import { describe, expect, it } from "vitest";

import {
  createRingBuffer,
  ringClear,
  ringPush,
  ringToArray,
} from "../src/viewer/sim/ringBuffer";

describe("ring buffer", () => {
  it("keeps insertion order before wrapping", () => {
    let buffer = createRingBuffer<number>(3);
    expect(ringToArray(buffer)).toEqual([]);
    buffer = ringPush(buffer, 1);
    buffer = ringPush(buffer, 2);
    expect(buffer.length).toBe(2);
    expect(ringToArray(buffer)).toEqual([1, 2]);
  });

  it("overwrites the oldest item in order after wrapping", () => {
    let buffer = createRingBuffer<number>(3);
    for (const value of [1, 2, 3, 4, 5]) {
      buffer = ringPush(buffer, value);
    }
    expect(buffer.length).toBe(3);
    expect(buffer.capacity).toBe(3);
    expect(ringToArray(buffer)).toEqual([3, 4, 5]);
    buffer = ringPush(buffer, 6);
    expect(ringToArray(buffer)).toEqual([4, 5, 6]);
  });

  it("does not mutate the previous buffer", () => {
    const first = ringPush(createRingBuffer<number>(2), 1);
    const second = ringPush(first, 2);
    expect(ringToArray(first)).toEqual([1]);
    expect(ringToArray(second)).toEqual([1, 2]);
  });

  it("clears to an empty buffer of the same capacity", () => {
    let buffer = createRingBuffer<string>(4);
    buffer = ringPush(buffer, "a");
    buffer = ringPush(buffer, "b");
    const cleared = ringClear(buffer);
    expect(cleared.capacity).toBe(4);
    expect(cleared.length).toBe(0);
    expect(ringToArray(cleared)).toEqual([]);
  });

  it("supports capacity one", () => {
    let buffer = createRingBuffer<number>(1);
    buffer = ringPush(buffer, 7);
    buffer = ringPush(buffer, 8);
    expect(ringToArray(buffer)).toEqual([8]);
  });

  it("rejects non-positive or fractional capacities", () => {
    expect(() => createRingBuffer(0)).toThrow(RangeError);
    expect(() => createRingBuffer(-2)).toThrow(RangeError);
    expect(() => createRingBuffer(2.5)).toThrow(RangeError);
  });
});
