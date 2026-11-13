from dataclasses import dataclass
import numpy as np

# State layout:
# [x, vx, y, vy, z, vz, roll, pitch, yaw, p, q, r]
STATE_X = 0
STATE_VX = 1
STATE_Y = 2
STATE_VY = 3
STATE_Z = 4
STATE_VZ = 5
STATE_ROLL = 6
STATE_PITCH = 7
STATE_YAW = 8
STATE_P = 9
STATE_Q = 10
STATE_R = 11
STATE_SIZE = 12


@dataclass(frozen=True)
class QuadParams:
    mass: float
    inertia: np.ndarray
    inertia_inv: np.ndarray
    arm_length: float
    kf: float
    km: float
    gravity: float

    @property
    def gamma(self):
        return self.km / self.kf


@dataclass(frozen=True)
class PIDParams:
    control_dt: float
    kf: float
    gravity: float

    p_for: np.ndarray
    i_for: np.ndarray
    d_for: np.ndarray
    p_tor: np.ndarray
    i_tor: np.ndarray
    d_tor: np.ndarray

    pwm2rpm_scale: float
    pwm2rpm_const: float
    min_pwm: float
    max_pwm: float

    pos_integral_limit: float
    z_integral_limit: float
    attitude_integral_limit: float
    roll_pitch_integral_limit: float
    torque_limit: float

    mixer_matrix: np.ndarray
    target_yaw: float


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