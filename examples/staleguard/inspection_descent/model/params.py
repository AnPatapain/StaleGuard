from dataclasses import dataclass
import numpy as np


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
    # Controller timing
    control_dt: float

    # Controller's thrust coefficient and gravity.
    # These belong to the controller model,
    # not the rigid-body physics model.
    kf: float
    gravity: float

    # Position PID
    p_for: np.ndarray
    i_for: np.ndarray
    d_for: np.ndarray

    # Attitude PID
    p_tor: np.ndarray
    i_tor: np.ndarray
    d_tor: np.ndarray

    # Motor command conversion
    pwm2rpm_scale: float
    pwm2rpm_const: float
    min_pwm: float
    max_pwm: float

    # SCG PID limits
    pos_integral_limit: float
    z_integral_limit: float

    attitude_integral_limit: float
    roll_pitch_integral_limit: float

    torque_limit: float

    # 4 motors x 3 torques
    mixer_matrix: np.ndarray

    # Current SCG mission uses yaw target = 0.
    target_yaw: float