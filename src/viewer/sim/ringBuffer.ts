/**
 * Fixed-capacity ring buffer with explicit, immutable state. No DOM, classes
 * or hidden mutation: every operation returns a new buffer.
 */

export interface RingBufferState<T> {
  readonly items: readonly (T | undefined)[];
  readonly capacity: number;
  /** Index of the oldest element. */
  readonly start: number;
  readonly length: number;
}

function assertCapacity(capacity: number): void {
  if (!Number.isInteger(capacity) || capacity < 1) {
    throw new RangeError("ring buffer capacity must be a positive integer");
  }
}

/** A fresh, empty buffer. */
export function createRingBuffer<T>(capacity: number): RingBufferState<T> {
  assertCapacity(capacity);
  return {
    items: new Array<T | undefined>(capacity),
    capacity,
    start: 0,
    length: 0,
  };
}

/** Push `value`, overwriting the oldest item when full. */
export function ringPush<T>(
  buffer: RingBufferState<T>,
  value: T,
): RingBufferState<T> {
  const { capacity, start, length } = buffer;
  const items = buffer.items.slice();
  const full = length === capacity;
  const index = full ? start : (start + length) % capacity;
  items[index] = value;
  return {
    items,
    capacity,
    start: full ? (start + 1) % capacity : start,
    length: full ? capacity : length + 1,
  };
}

/** Oldest-to-newest copy of the live elements. */
export function ringToArray<T>(buffer: RingBufferState<T>): readonly T[] {
  const result: T[] = [];
  for (let offset = 0; offset < buffer.length; offset++) {
    const value = buffer.items[(buffer.start + offset) % buffer.capacity];
    if (value === undefined) {
      throw new Error("ring buffer contains an undefined element");
    }
    result.push(value);
  }
  return result;
}

/** A fresh empty buffer with the same capacity. */
export function ringClear<T>(buffer: RingBufferState<T>): RingBufferState<T> {
  return createRingBuffer<T>(buffer.capacity);
}
