from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class PIDState:
    integral_pos_e: np.ndarray
    last_rpy: np.ndarray
    integral_rpy_e: np.ndarray


def capture_pid_state(ctrl):
    return PIDState(
        integral_pos_e=np.asarray(
            ctrl.integral_pos_e
        ).copy(),

        last_rpy=np.asarray(
            ctrl.last_rpy
        ).copy(),

        integral_rpy_e=np.asarray(
            ctrl.integral_rpy_e
        ).copy(),
    )


def restore_pid_state(
    ctrl,
    pid_state,
):
    ctrl.integral_pos_e = np.asarray(
        pid_state.integral_pos_e
    ).copy()

    ctrl.last_rpy = np.asarray(
        pid_state.last_rpy
    ).copy()

    ctrl.integral_rpy_e = np.asarray(
        pid_state.integral_rpy_e
    ).copy()