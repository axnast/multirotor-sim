/** Orbit controls with an optional smooth follow mode. */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

export interface CameraRig {
  readonly controls: OrbitControls;
  update(target: THREE.Vector3, dt: number, follow: boolean): void;
  reset(target: THREE.Vector3): void;
  dispose(): void;
}

export function createCameraRig(
  camera: THREE.PerspectiveCamera,
  domElement: HTMLElement,
  initialTarget: THREE.Vector3,
): CameraRig {
  const controls = new OrbitControls(camera, domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.minDistance = 1.2;
  controls.maxDistance = 60;
  controls.maxPolarAngle = Math.PI / 2 - 0.02;
  controls.target.copy(initialTarget);
  controls.update();

  const reset = (target: THREE.Vector3): void => {
    controls.target.copy(target);
    camera.position.set(target.x + 3.4, target.y + 1.4, target.z + 4.2);
    controls.update();
  };

  return {
    controls,
    update(target: THREE.Vector3, dt: number, follow: boolean): void {
      if (follow) {
        const desired = new THREE.Vector3(target.x, Math.max(0.1, target.y + 0.15), target.z);
        const delta = desired
          .sub(controls.target)
          .multiplyScalar(1 - Math.exp(-8 * Math.max(0, dt)));
        controls.target.add(delta);
        camera.position.add(delta);
        controls.target.y = Math.max(0.1, controls.target.y);
      }
      controls.update();
    },
    reset,
    dispose(): void {
      controls.dispose();
    },
  };
}
