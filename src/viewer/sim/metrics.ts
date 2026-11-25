/** Shared display metrics used by both the status line and the HUD. */

import { vec3Length, vec3Sub, type Vec3 } from "../../core/math/vec3";

/** 3D distance from the actual position to the active setpoint position. */
export function positionErrorM(position: Vec3, target: Vec3): number {
  return vec3Length(vec3Sub(position, target));
}
