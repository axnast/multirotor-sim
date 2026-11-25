/** Config-driven drone model. All placement uses src/viewer/coords.ts. */

import * as THREE from "three";

import type { MultirotorConfig, SpinDirection } from "../../core/multirotor/config";
import type { MultirotorState } from "../../core/multirotor/dynamics";
import { coreQuatToThree, coreToThree } from "../coords";

export interface RotorVisual {
  readonly spin: SpinDirection;
  readonly maxOmega: number;
  readonly pivot: THREE.Group;
  readonly discMaterial: THREE.MeshBasicMaterial;
  readonly bladeMaterial: THREE.MeshBasicMaterial;
}

export interface DroneVisual {
  readonly group: THREE.Group;
  readonly rotors: readonly RotorVisual[];
  readonly axes: THREE.Group;
  readonly gearHeight: number;
}

const IDLE_COLOR = new THREE.Color(0x334155);
const CCW_COLOR = new THREE.Color(0x38bdf8);
const CW_COLOR = new THREE.Color(0xfb7185);

function cylinderBetween(
  start: THREE.Vector3,
  end: THREE.Vector3,
  radius: number,
  material: THREE.Material,
): THREE.Mesh {
  const direction = new THREE.Vector3().subVectors(end, start);
  const length = direction.length();
  const mesh = new THREE.Mesh(
    new THREE.CylinderGeometry(radius, radius, length, 8),
    material,
  );
  mesh.position.copy(start).add(end).multiplyScalar(0.5);
  mesh.quaternion.setFromUnitVectors(
    new THREE.Vector3(0, 1, 0),
    direction.normalize(),
  );
  mesh.castShadow = true;
  return mesh;
}

function buildAxes(radius: number, height: number): THREE.Group {
  const axes = new THREE.Group();
  const origin = new THREE.Vector3(0, height, 0);
  const length = radius * 1.5;
  const head = length * 0.28;
  const directions: readonly [readonly number[], number][] = [
    // Body axes in Three-local coordinates (body y maps to Three -z).
    [[1, 0, 0], 0xff5a5a],
    [[0, 0, -1], 0x4ade80],
    [[0, 1, 0], 0x60a5fa],
  ];
  for (const [direction, color] of directions) {
    const arrow = new THREE.ArrowHelper(
      new THREE.Vector3(direction[0]!, direction[1]!, direction[2]!),
      origin,
      length,
      color,
      head,
      head * 0.55,
    );
    axes.add(arrow);
  }
  return axes;
}

/**
 * Build the airframe from config. The visual mesh is offset upward by a small
 * landing-gear height so the body clears the ground; the physics centre of
 * mass remains at world z = 0 and is not modified.
 */
export function buildDroneModel(config: MultirotorConfig): DroneVisual {
  const group = new THREE.Group();
  const radius = Math.max(
    ...config.motors.map((motor) =>
      Math.hypot(motor.position[0], motor.position[1]),
    ),
  );
  const gearHeight = Math.min(0.08, Math.max(0.03, radius * 0.25));
  const rotorRadius = Math.min(0.08, radius * 0.28);

  const bodyMaterial = new THREE.MeshStandardMaterial({
    color: 0xdfe7f5,
    roughness: 0.55,
    metalness: 0.25,
  });
  const armMaterial = new THREE.MeshStandardMaterial({
    color: 0x5b6b8c,
    roughness: 0.7,
    metalness: 0.2,
  });
  const legMaterial = new THREE.MeshStandardMaterial({
    color: 0x2c3550,
    roughness: 0.8,
  });

  const body = new THREE.Mesh(
    new THREE.CylinderGeometry(radius * 0.42, radius * 0.5, gearHeight * 0.6, 16),
    bodyMaterial,
  );
  body.position.y = gearHeight;
  body.castShadow = true;
  group.add(body);

  const rotors: RotorVisual[] = [];
  const hubCenter = new THREE.Vector3(0, gearHeight, 0);
  for (const motor of config.motors) {
    const local = coreToThree(motor.position);
    const hub = new THREE.Vector3(local[0], gearHeight, local[2]);
    group.add(cylinderBetween(hubCenter, hub, radius * 0.035, armMaterial));
    group.add(
      cylinderBetween(
        new THREE.Vector3(hub.x, 0, hub.z),
        hub,
        radius * 0.018,
        legMaterial,
      ),
    );

    const motorHub = new THREE.Mesh(
      new THREE.CylinderGeometry(rotorRadius * 0.18, rotorRadius * 0.18, 0.02, 12),
      armMaterial,
    );
    motorHub.position.set(hub.x, gearHeight + 0.01, hub.z);
    motorHub.castShadow = true;
    group.add(motorHub);

    const active = motor.spin === 1 ? CCW_COLOR : CW_COLOR;
    const discMaterial = new THREE.MeshBasicMaterial({
      color: IDLE_COLOR.clone(),
      transparent: true,
      opacity: 0.35,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    const disc = new THREE.Mesh(
      new THREE.RingGeometry(rotorRadius * 0.18, rotorRadius, 32),
      discMaterial,
    );
    disc.rotation.x = -Math.PI / 2;
    disc.position.set(hub.x, gearHeight + 0.018, hub.z);
    group.add(disc);

    const bladeMaterial = new THREE.MeshBasicMaterial({
      color: active.clone(),
      transparent: true,
      opacity: 0.85,
    });
    const blade = new THREE.Mesh(
      new THREE.BoxGeometry(rotorRadius * 1.9, 0.004, rotorRadius * 0.12),
      bladeMaterial,
    );
    const pivot = new THREE.Group();
    pivot.position.set(hub.x, gearHeight + 0.022, hub.z);
    pivot.add(blade);
    group.add(pivot);

    rotors.push({
      spin: motor.spin,
      maxOmega: motor.maxOmega,
      pivot,
      discMaterial,
      bladeMaterial,
    });
  }

  // Permanent nose marker on the +x arm. It is a child of the airframe (not
  // the axes group), so toggling the axis arrows never hides it.
  const nose = new THREE.Mesh(
    new THREE.ConeGeometry(radius * 0.12, radius * 0.34, 4),
    new THREE.MeshStandardMaterial({
      color: 0xfde68a,
      emissive: 0x4a3200,
      roughness: 0.5,
    }),
  );
  nose.rotation.z = -Math.PI / 2; // cone +Y becomes body +x
  nose.position.set(radius * 0.72, gearHeight + 0.03, 0);
  nose.castShadow = true;
  group.add(nose);

  const axes = buildAxes(radius, gearHeight + 0.035);
  group.add(axes);

  return { group, rotors, axes, gearHeight };
}

/** Apply simulator state to the drone model; `visualDt` drives rotor animation. */
export function updateDrone(
  visual: DroneVisual,
  state: MultirotorState,
  visualDt: number,
): void {
  visual.group.position.set(
    ...coreToThree(state.body.position),
  );
  visual.group.quaternion.set(
    ...coreQuatToThree(state.body.orientation),
  );

  for (const [index, rotor] of visual.rotors.entries()) {
    const omega = Math.max(0, state.motorSpeeds[index] ?? 0);
    const fraction = Math.min(1, omega / rotor.maxOmega);
    const active = rotor.spin === 1 ? CCW_COLOR : CW_COLOR;
    rotor.discMaterial.color.copy(IDLE_COLOR).lerp(active, fraction);
    rotor.discMaterial.opacity = 0.15 + 0.55 * fraction;
    rotor.bladeMaterial.opacity = 0.25 + 0.7 * fraction;
    // Illustrative rotation: real rotor rates alias badly at 60 Hz.
    rotor.pivot.rotation.y +=
      rotor.spin * Math.min(25, omega * 0.02) * visualDt;
  }
}
