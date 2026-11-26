/** World-frame wind arrow following the drone, hidden when wind is absent. */

import * as THREE from "three";

import type { Vec3 } from "../../core/math/vec3";
import { coreToThree } from "../coords";

export interface WindArrow {
  readonly group: THREE.Group;
  update(wind: Vec3 | null, dronePosition: THREE.Vector3): void;
  dispose(): void;
}

export function createWindArrow(): WindArrow {
  const arrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(),
    1,
    0x38bdf8,
    0.28,
    0.16,
  );
  const group = new THREE.Group();
  group.add(arrow);
  group.visible = false;
  const direction = new THREE.Vector3();
  const offset = new THREE.Vector3(0, 1.2, 0);

  return {
    group,
    update(wind: Vec3 | null, dronePosition: THREE.Vector3): void {
      if (wind === null) {
        group.visible = false;
        return;
      }
      const three = coreToThree(wind);
      const speed = Math.hypot(three[0], three[1], three[2]);
      if (!Number.isFinite(speed) || speed < 1e-6) {
        group.visible = false;
        return;
      }
      direction.set(three[0] / speed, three[1] / speed, three[2] / speed);
      arrow.setDirection(direction);
      arrow.setLength(Math.min(3, 0.4 + speed * 0.25), 0.28, 0.16);
      group.position.copy(dronePosition).add(offset);
      group.visible = true;
    },
    dispose(): void {
      arrow.dispose();
    },
  };
}
