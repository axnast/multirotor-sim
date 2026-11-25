/** Amber setpoint marker with yaw arrow and altitude guide. */

import * as THREE from "three";

import type { Setpoint } from "../../core/control/types";
import { coreToThree } from "../coords";

export interface SetpointMarkerVisual {
  readonly group: THREE.Group;
  update(setpoint: Setpoint, timeSeconds: number): void;
}

export function createSetpointMarker(): SetpointMarkerVisual {
  const group = new THREE.Group();
  const material = new THREE.MeshBasicMaterial({
    color: 0xfbbf24,
    transparent: true,
    opacity: 0.95,
  });

  const ring = new THREE.Mesh(new THREE.TorusGeometry(0.22, 0.012, 8, 40), material);
  ring.rotation.x = -Math.PI / 2;
  group.add(ring);

  const groundRing = new THREE.Mesh(
    new THREE.TorusGeometry(0.12, 0.008, 8, 32),
    new THREE.MeshBasicMaterial({ color: 0xfbbf24, transparent: true, opacity: 0.35 }),
  );
  groundRing.rotation.x = -Math.PI / 2;
  group.add(groundRing);

  const arrow = new THREE.ArrowHelper(
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(),
    0.45,
    0xfbbf24,
    0.12,
    0.07,
  );
  group.add(arrow);

  const guideGeometry = new THREE.BufferGeometry();
  const guidePositions = new Float32Array(6);
  guideGeometry.setAttribute(
    "position",
    new THREE.BufferAttribute(guidePositions, 3).setUsage(THREE.DynamicDrawUsage),
  );
  const guide = new THREE.Line(
    guideGeometry,
    new THREE.LineBasicMaterial({ color: 0xfbbf24, transparent: true, opacity: 0.3 }),
  );
  guide.frustumCulled = false;
  group.add(guide);

  return {
    group,
    update(setpoint: Setpoint, timeSeconds: number): void {
      const position = coreToThree(setpoint.position);
      const center = new THREE.Vector3(position[0], position[1], position[2]);
      ring.position.copy(center);
      const pulse = 1 + 0.06 * Math.sin(timeSeconds * 3);
      ring.scale.set(pulse, pulse, pulse);
      groundRing.position.set(position[0], 0.006, position[2]);

      arrow.position.copy(center);
      const heading = coreToThree([
        Math.cos(setpoint.yaw),
        Math.sin(setpoint.yaw),
        0,
      ]);
      arrow.setDirection(
        new THREE.Vector3(heading[0], heading[1], heading[2]).normalize(),
      );

      guidePositions[0] = position[0];
      guidePositions[1] = position[1];
      guidePositions[2] = position[2];
      guidePositions[3] = position[0];
      guidePositions[4] = 0;
      guidePositions[5] = position[2];
      guideGeometry.attributes.position!.needsUpdate = true;
    },
  };
}
