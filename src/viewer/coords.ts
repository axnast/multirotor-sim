/**
 * The single coordinate conversion boundary between the simulator and
 * Three.js. Do not put (x,y,z) ↔ (x,z,-y) formulas anywhere else.
 *
 * Core world is right-handed Z-up FLU; Three.js is right-handed Y-up. The
 * rotation C maps core coordinates to Three coordinates:
 *
 *   p_three = C · p_core      C = [ 1  0  0 ]
 *                                 [ 0  0  1 ]
 *                                 [ 0 -1  0 ]
 *
 *   (x, y, z)_core -> (x, z, -y)_three
 *
 * det(C) = +1, so handedness is preserved. It is an active R_x(-90°) rotation:
 * body +x stays Three +x, body +y becomes Three -z, and body +z becomes
 * Three +y.
 *
 * Quaternions transform by conjuction, C q C⁻¹; because C is a proper
 * rotation, the vector part simply maps with C while the scalar part is
 * unchanged:
 *
 *   [w, x, y, z]_core -> [x, z, -y, w]_three (Three stores x,y,z,w)
 */

import type { Quat } from "../core/math/quat";
import type { Vec3 } from "../core/math/vec3";

/** Three.js vector tuple, in its native x,y,z order. */
export type ThreeVec3 = readonly [number, number, number];

/** Three.js quaternion tuple, in its native x,y,z,w order. */
export type ThreeQuat = readonly [number, number, number, number];

export function coreToThree(v: Vec3): ThreeVec3 {
  return [v[0], v[2], -v[1]];
}

export function threeToCore(v: ThreeVec3): Vec3 {
  return [v[0], -v[2], v[1]];
}

export function coreQuatToThree(q: Quat): ThreeQuat {
  return [q[1], q[3], -q[2], q[0]];
}

export function threeQuatToCore(q: ThreeQuat): Quat {
  return [q[3], q[0], -q[2], q[1]];
}
