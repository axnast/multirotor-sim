/** Position trail backed by the runner's deterministic ring buffer. */

import * as THREE from "three";

import type { Vec3 } from "../../core/math/vec3";
import { coreToThree } from "../coords";

export interface TrailVisual {
  readonly line: THREE.Line;
  readonly capacity: number;
  update(positions: readonly Vec3[]): void;
  dispose(): void;
}

export function createTrail(capacity: number): TrailVisual {
  const geometry = new THREE.BufferGeometry();
  const coordinates = new Float32Array(capacity * 3);
  const attribute = new THREE.BufferAttribute(coordinates, 3);
  attribute.setUsage(THREE.DynamicDrawUsage);
  geometry.setAttribute("position", attribute);
  geometry.setDrawRange(0, 0);

  const material = new THREE.LineBasicMaterial({
    color: 0x22d3ee,
    transparent: true,
    opacity: 0.8,
  });
  const line = new THREE.Line(geometry, material);
  line.frustumCulled = false;

  return {
    line,
    capacity,
    update(positions: readonly Vec3[]): void {
      const count = Math.min(positions.length, capacity);
      for (let index = 0; index < count; index++) {
        const point = coreToThree(positions[index]!);
        coordinates[index * 3] = point[0];
        coordinates[index * 3 + 1] = point[1];
        coordinates[index * 3 + 2] = point[2];
      }
      geometry.setDrawRange(0, count);
      attribute.needsUpdate = true;
    },
    dispose(): void {
      geometry.dispose();
      material.dispose();
    },
  };
}
