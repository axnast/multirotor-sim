import { describe, expect, it } from "vitest";

import {
  quatFromAxisAngle,
  quatMultiply,
  quatNormalize,
  quatRotateVector,
  type Quat,
} from "../src/core/math/quat";
import type { Vec3 } from "../src/core/math/vec3";
import {
  coreQuatToThree,
  coreToThree,
  threeQuatToCore,
  threeToCore,
  type ThreeQuat,
  type ThreeVec3,
} from "../src/viewer/coords";

function cross(a: ThreeVec3, b: ThreeVec3): ThreeVec3 {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

/** Independent Three-order quaternion rotation: v + 2w(qv×v) + 2qv×(qv×v). */
function rotateThree(q: ThreeQuat, v: ThreeVec3): ThreeVec3 {
  const qv: ThreeVec3 = [q[0], q[1], q[2]];
  const t = cross(qv, v);
  const twiceT: ThreeVec3 = [2 * t[0], 2 * t[1], 2 * t[2]];
  const cross2 = cross(qv, twiceT);
  return [
    v[0] + q[3] * twiceT[0] + cross2[0],
    v[1] + q[3] * twiceT[1] + cross2[1],
    v[2] + q[3] * twiceT[2] + cross2[2],
  ];
}

function expectVec(actual: readonly number[], expected: readonly number[], tol = 1e-12): void {
  for (let index = 0; index < expected.length; index++) {
    expect(actual[index]).toBeCloseTo(expected[index]!, Math.ceil(-Math.log10(tol)));
  }
}

describe("core ↔ Three.js coordinate conversion", () => {
  it("maps Z-up to Y-up and round-trips vectors exactly", () => {
    const v: Vec3 = [1.25, -2.5, 3.75];
    expectVec(coreToThree(v), [1.25, 3.75, 2.5]);
    expectVec(threeToCore(coreToThree(v)), v);
    expectVec(coreToThree([0, 0, 5]), [0, 5, 0]);
  });

  it("preserves vector length", () => {
    const v: Vec3 = [0.7, -1.1, 2.3];
    const coreLength = Math.hypot(...v);
    const three = coreToThree(v);
    expect(Math.hypot(...three)).toBeCloseTo(coreLength, 12);
  });

  it("preserves handedness", () => {
    const ex = coreToThree([1, 0, 0]);
    const ey = coreToThree([0, 1, 0]);
    const ez = coreToThree([0, 0, 1]);
    expectVec(cross(ex, ey), ez);
    expectVec(cross(ey, ez), ex);
    expectVec(cross(ez, ex), ey);
  });

  it("round-trips quaternions", () => {
    const q = quatNormalize([0.7, 0.3, -0.5, 0.2]);
    const three = coreQuatToThree(q);
    expect(three[3]).toBeCloseTo(q[0], 12); // scalar part is unchanged
    expectVec([...three].slice(0, 3), [q[1], q[3], -q[2]]);
    const back = threeQuatToCore(three);
    for (let index = 0; index < 4; index++) {
      expect(back[index]).toBeCloseTo(q[index]!, 12);
    }
  });

  it("maps core yaw to Three +y, roll to +x and pitch to -z", () => {
    const yaw = coreQuatToThree(quatFromAxisAngle([0, 0, 1], Math.PI / 2));
    expectVec(rotateThree(yaw, [1, 0, 0]), [0, 0, -1]);

    const roll = coreQuatToThree(quatFromAxisAngle([1, 0, 0], Math.PI / 2));
    expectVec(rotateThree(roll, [0, 1, 0]), [0, 0, 1]);

    const pitch = coreQuatToThree(quatFromAxisAngle([0, 1, 0], Math.PI / 2));
    expectVec(rotateThree(pitch, [1, 0, 0]), [0, -1, 0]);
  });

  it("commutes rotation and conversion for a general quaternion", () => {
    const q: Quat = quatNormalize([0.82, -0.31, 0.41, 0.26]);
    const v: Vec3 = [0.4, 1.2, -0.9];
    const coreRotated = quatRotateVector(q, v);
    const threeRotated = rotateThree(coreQuatToThree(q), coreToThree(v));
    expectVec(threeRotated, coreToThree(coreRotated));
  });

  it("preserves composition order", () => {
    const a = quatFromAxisAngle([1, 0, 0], 0.4);
    const b = quatFromAxisAngle([0, 1, 0], -0.7);
    // Core applies b first, then a: a ⊗ b. Three must do the same.
    const composed = coreQuatToThree(quatMultiply(a, b));
    const v: ThreeVec3 = [0.3, -0.6, 1.1];
    const applied = rotateThree(
      coreQuatToThree(a),
      rotateThree(coreQuatToThree(b), v),
    );
    expectVec(rotateThree(composed, v), applied);
  });

  it("treats q and -q as the same orientation", () => {
    const q = quatNormalize([0.5, 0.5, 0.5, 0.5]);
    const negative: Quat = [-q[0], -q[1], -q[2], -q[3]];
    const v: ThreeVec3 = [1, 2, 3];
    expectVec(rotateThree(coreQuatToThree(q), v), rotateThree(coreQuatToThree(negative), v));
  });
});
