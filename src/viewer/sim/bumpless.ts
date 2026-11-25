/**
 * Bumpless live gain changes.
 *
 * The core stores the raw integral ∫e dt and multiplies it by `ki` when it
 * forms the output. Changing `ki` in the viewer would therefore cause an
 * instantaneous output step of (ki_new - ki_old)·∫e dt. Rather than change core
 * behaviour, the viewer rescales the stored integrals so `ki·∫e dt` is
 * preserved. Turning the integrator off (`ki = 0`) zeroes it, as expected.
 */

import type {
  AxisPidState,
  ControllerParams,
  ControllerState,
  PidState,
} from "../../core/control/types";

function rescaleAxis(state: PidState, factor: number): PidState {
  return { ...state, integral: state.integral * factor };
}

function rescaleAxes(axes: AxisPidState, factor: number): AxisPidState {
  return [
    rescaleAxis(axes[0], factor),
    rescaleAxis(axes[1], factor),
    rescaleAxis(axes[2], factor),
  ];
}

/** Return a new controller state whose integral contributions are unchanged. */
export function bumplessKiChange(
  state: ControllerState,
  oldParams: ControllerParams,
  newParams: ControllerParams,
): ControllerState {
  const oldPositionKi = oldParams.position.gains.ki;
  const newPositionKi = newParams.position.gains.ki;
  const oldRateKi = oldParams.rate.gains.ki;
  const newRateKi = newParams.rate.gains.ki;
  const positionFactor = newPositionKi === 0 ? 0 : oldPositionKi / newPositionKi;
  const rateFactor = newRateKi === 0 ? 0 : oldRateKi / newRateKi;

  return {
    position: rescaleAxes(state.position, positionFactor),
    rate: rescaleAxes(state.rate, rateFactor),
  };
}
