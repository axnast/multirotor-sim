/** Declarative missions and their pure runtime state. */

import { QUAT_IDENTITY } from "../../core/math/quat";
import type { Vec3 } from "../../core/math/vec3";
import {
  hoverMotorSpeeds,
  type MultirotorConfig,
} from "../../core/multirotor/config";
import type { MultirotorState } from "../../core/multirotor/dynamics";
import type { Setpoint } from "../../core/control/types";

export type MissionId = "hover" | "step" | "square" | "takeoff";

export interface MissionWaypoint {
  readonly position: Vec3;
  readonly yaw: number;
  /** Number of 1 ms steps to hold; null means the final waypoint. */
  readonly holdSteps: number | null;
}

export interface Mission {
  readonly id: MissionId;
  readonly name: string;
  readonly description: string;
  readonly startPosition: Vec3;
  readonly startOnGround: boolean;
  readonly waypoints: readonly MissionWaypoint[];
}

export interface MissionRuntime {
  readonly mission: Mission;
  readonly waypointIndex: number;
  readonly waypointSteps: number;
}

function waypoint(
  position: Vec3,
  yaw: number,
  holdSeconds: number | null,
): MissionWaypoint {
  return {
    position,
    yaw,
    holdSteps: holdSeconds === null ? null : holdSeconds * 1000,
  };
}

const AT_HOVER: Vec3 = [0, 0, 5];
const STEP_TARGET: Vec3 = [5, 0, 5];
const SQUARE_TARGETS: readonly Vec3[] = [
  [5, 0, 5],
  [5, 5, 5],
  [0, 5, 5],
  AT_HOVER,
];

/** Built-in 1 kHz missions. All waypoint timing is in integer steps. */
export const MISSIONS: Record<MissionId, Mission> = {
  hover: {
    id: "hover",
    name: "Hover",
    description: "Hold 5 m altitude at the origin.",
    startPosition: AT_HOVER,
    startOnGround: false,
    waypoints: [waypoint(AT_HOVER, 0, null)],
  },
  step: {
    id: "step",
    name: "5 m step",
    description: "Step 5 m in +x from a 5 m hover.",
    startPosition: AT_HOVER,
    startOnGround: false,
    waypoints: [waypoint(STEP_TARGET, 0, null)],
  },
  square: {
    id: "square",
    name: "Square",
    description: "Fly a 5 m square with 20 s legs.",
    startPosition: AT_HOVER,
    startOnGround: false,
    waypoints: [
      waypoint(SQUARE_TARGETS[0]!, 0, 20),
      waypoint(SQUARE_TARGETS[1]!, 0, 20),
      waypoint(SQUARE_TARGETS[2]!, 0, 20),
      waypoint(SQUARE_TARGETS[3]!, 0, null),
    ],
  },
  takeoff: {
    id: "takeoff",
    name: "Takeoff",
    description: "Take off from the ground and hold 5 m.",
    startPosition: [0, 0, 0],
    startOnGround: true,
    waypoints: [waypoint(AT_HOVER, 0, null)],
  },
};

/** Fresh runtime at the first waypoint. */
export function createMissionRuntime(mission: Mission): MissionRuntime {
  return { mission, waypointIndex: 0, waypointSteps: 0 };
}

/** Current setpoint for a mission runtime. */
export function missionSetpoint(runtime: MissionRuntime): Setpoint {
  const active =
    runtime.mission.waypoints[runtime.waypointIndex] ??
    runtime.mission.waypoints[0];
  if (active === undefined) {
    throw new Error(`mission ${runtime.mission.id} has no waypoints`);
  }
  return { position: active.position, yaw: active.yaw };
}

/**
 * Advance mission timing by one core step. The setpoint seen by that step is
 * the one active at its start; timing changes apply from the following step.
 */
export function advanceMission(runtime: MissionRuntime): MissionRuntime {
  const active = runtime.mission.waypoints[runtime.waypointIndex];
  if (active === undefined || active.holdSteps === null) return runtime;
  if (runtime.waypointSteps + 1 >= active.holdSteps) {
    return {
      mission: runtime.mission,
      waypointIndex: Math.min(
        runtime.waypointIndex + 1,
        runtime.mission.waypoints.length - 1,
      ),
      waypointSteps: 0,
    };
  }
  return { ...runtime, waypointSteps: runtime.waypointSteps + 1 };
}

/** Aircraft state at the beginning of a mission. */
export function missionInitialMultirotor(
  config: MultirotorConfig,
  mission: Mission,
): MultirotorState {
  const motorSpeeds = mission.startOnGround
    ? config.motors.map(() => 0)
    : hoverMotorSpeeds(config);
  return {
    body: {
      position: [
        mission.startPosition[0],
        mission.startPosition[1],
        mission.startPosition[2],
      ],
      velocity: [0, 0, 0],
      orientation: QUAT_IDENTITY,
      angularVelocity: [0, 0, 0],
    },
    motorSpeeds,
  };
}
