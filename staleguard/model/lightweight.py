from numba import njit
from copy import deepcopy
from dataclasses import dataclass
import numpy as np
from staleguard.model import CTRL_FREQ_HZ, MODEL_FREQ_HZ
from staleguard.model.common import (
    PIDState,
    STATE_P,
    STATE_PITCH,
    STATE_Q,
    STATE_R,
    STATE_ROLL,
    STATE_VX,
    STATE_VY,
    STATE_VZ,
    STATE_X,
    STATE_Y,
    STATE_YAW,
    STATE_Z,
    capture_pid_state,
    restore_pid_state,
)


@njit(cache=True)
def rotate_matrix_body_to_world(phi, theta, psi):
    """
    (Linear transformation)
    Rotate vector expressed in body frame to inertial frame (world frame).
    """
    cphi = np.cos(phi)
    sphi = np.sin(phi)

    ctheta = np.cos(theta)
    stheta = np.sin(theta)

    cpsi = np.cos(psi)
    spsi = np.sin(psi)

    return np.array([
        [
            cpsi * ctheta,
            cpsi * stheta * sphi - spsi * cphi,
            cpsi * stheta * cphi + spsi * sphi,
        ],
        [
            spsi * ctheta,
            spsi * stheta * sphi + cpsi * cphi,
            spsi * stheta * cphi - cpsi * sphi,
        ],
        [
            -stheta,
            ctheta * sphi,
            ctheta * cphi,
        ],
    ])


@njit(cache=True)
def pid_step(
        obs,  # current 12-D states vector.
        target,  # target 12-D states vector
        integral_pos_e,  # accumulated position error
        last_rpy,  # roll-pitch-yaw in the previous step
        integral_rpy_e,  # accumulated roll-pitch-yaw error
        mass,

        control_dt,
        kf,
        gravity,

        p_for,
        i_for,
        d_for,

        p_tor,
        i_tor,
        d_tor,

        pwm2rpm_scale,
        pwm2rpm_const,
        min_pwm,
        max_pwm,

        pos_integral_limit,
        z_integral_limit,
        attitude_integral_limit,
        roll_pitch_integral_limit,
        torque_limit,

        mixer_matrix,
        target_yaw,
):
    # --------------------------------------------------
    # Current Quadrotor state
    # --------------------------------------------------
    cur_pos = np.array([
        obs[STATE_X],
        obs[STATE_Y],
        obs[STATE_Z],
    ])
    cur_vel = np.array([
        obs[STATE_VX],
        obs[STATE_VY],
        obs[STATE_VZ],
    ])
    cur_rpy = np.array([
        obs[STATE_ROLL],
        obs[STATE_PITCH],
        obs[STATE_YAW],
    ])
    cur_rotation = rotate_matrix_body_to_world(
        cur_rpy[0],
        cur_rpy[1],
        cur_rpy[2],
    )

    # --------------------------------------------------
    # Reference
    # --------------------------------------------------
    target_pos = np.array([
        target[STATE_X],
        target[STATE_Y],
        target[STATE_Z],
    ])
    target_vel = np.array([
        target[STATE_VX],
        target[STATE_VY],
        target[STATE_VZ],
    ])

    # --------------------------------------------------
    # Position PID
    # Same equations as SCG
    # --------------------------------------------------
    pos_e = target_pos - cur_pos
    vel_e = target_vel - cur_vel

    integral_pos_e[:] += pos_e * control_dt
    integral_pos_e[:] = np.clip(
        integral_pos_e,
        -pos_integral_limit,
        pos_integral_limit,
    )
    integral_pos_e[2] = min(
        z_integral_limit,
        max(-z_integral_limit, integral_pos_e[2]),
    )

    # Target thrust vector [Fx, Fy, Fz] in Newton (inertial world frame)
    target_thrust = (
            p_for * pos_e  # P
            + i_for * integral_pos_e  # I
            + d_for * vel_e  # D
            + np.array([0.0, 0.0, gravity * mass])
    )

    # target_thrust: [Fx, Fy, Fz] in world frame
    # cur_rotation: body x-axis, y-axis, z-axis vectors expressed in world frame
    # scalar_thrust is projection of [Fx, Fy, Fz] to k_b
    scalar_thrust = np.dot(target_thrust, cur_rotation[:, 2])
    if scalar_thrust < 0.0:
        scalar_thrust = 0.0

    # Convert scalar thrust to PWM (pulse-width modulation)
    thrust_pwm = (
                         np.sqrt(scalar_thrust / (4.0 * kf))
                         - pwm2rpm_const
                 ) / pwm2rpm_scale

    # Desired body Z direction
    target_zb_axis = target_thrust / np.linalg.norm(target_thrust)

    target_x_c = np.array([
        np.cos(target_yaw),
        np.sin(target_yaw),
        0.0,
    ])

    target_yb_axis = np.cross(
        target_zb_axis,
        target_x_c,
    )
    target_yb_axis /= np.linalg.norm(
        target_yb_axis
    )

    target_xb_axis = np.cross(
        target_yb_axis,
        target_zb_axis,
    )

    target_rotation = np.empty((3, 3))
    target_rotation[:, 0] = target_xb_axis
    target_rotation[:, 1] = target_yb_axis
    target_rotation[:, 2] = target_zb_axis

    # --------------------------------------------------
    # Attitude PID
    # Same equations as SCG
    # --------------------------------------------------
    rot_matrix_e = (
            target_rotation.T @ cur_rotation
            -
            cur_rotation.T @ target_rotation
    )
    rot_e = np.array([
        rot_matrix_e[2, 1],
        rot_matrix_e[0, 2],
        rot_matrix_e[1, 0],
    ])

    rpy_rates_e = -(
            cur_rpy - last_rpy
    ) / control_dt

    last_rpy[:] = cur_rpy

    integral_rpy_e[:] -= (
            rot_e * control_dt
    )
    integral_rpy_e[:] = np.clip(
        integral_rpy_e,
        -attitude_integral_limit,
        attitude_integral_limit,
    )
    integral_rpy_e[0] = min(
        roll_pitch_integral_limit,
        max(
            -roll_pitch_integral_limit,
            integral_rpy_e[0],
        ),
    )
    integral_rpy_e[1] = min(
        roll_pitch_integral_limit,
        max(
            -roll_pitch_integral_limit,
            integral_rpy_e[1],
        ),
    )

    target_torques = (
            -p_tor * rot_e
            + d_tor * rpy_rates_e
            + i_tor * integral_rpy_e
    )

    target_torques = np.clip(
        target_torques,
        -torque_limit,
        torque_limit,
    )

    pwm = (
            thrust_pwm
            + mixer_matrix @ target_torques
    )

    pwm = np.clip(
        pwm,
        min_pwm,
        max_pwm,
    )

    rpm = (
            pwm2rpm_scale * pwm
            + pwm2rpm_const
    )

    # SCG PID returns motor thrust in Newtons.
    return kf * rpm ** 2


class LightweightPID:
    def __init__(
            self,
            reference,
            quad_params,
            pid_params,
    ):
        self.reference = reference
        self.mass = quad_params.mass
        self.params = pid_params

        self.integral_pos_e = np.zeros(3)
        self.last_rpy = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)

    def select_action(
            self,
            obs,
            info,
    ):
        step = info["current_step"]
        p = self.params

        return pid_step(
            obs,
            self.reference[step],
            self.integral_pos_e,
            self.last_rpy,
            self.integral_rpy_e,
            self.mass,

            p.control_dt,
            p.kf,
            p.gravity,

            p.p_for,
            p.i_for,
            p.d_for,

            p.p_tor,
            p.i_tor,
            p.d_tor,

            p.pwm2rpm_scale,
            p.pwm2rpm_const,
            p.min_pwm,
            p.max_pwm,

            p.pos_integral_limit,
            p.z_integral_limit,
            p.attitude_integral_limit,
            p.roll_pitch_integral_limit,
            p.torque_limit,

            p.mixer_matrix,
            p.target_yaw,
        )


@dataclass(frozen=True)
class LightweightCheckpoint:
    obs: np.ndarray
    info: dict
    pid_state: PIDState


@njit(cache=True)
def dynamics(state, thrust, m, J, J_inv, L, gamma, g):
    """
    State: [x, vx, y, vy, z, vz, roll, pitch, yaw, p, q, r]
    Thrust: [f1, f2, f3, f4]
    Return rate of state change.
    """
    x = state
    phi, theta, psi = x[STATE_ROLL], x[STATE_PITCH], x[STATE_YAW]  # roll pitch yaw.
    p, q, r = x[STATE_P], x[STATE_Q], x[STATE_R]  # angular velocity of Quadrotor expressed in body-frame.
    f1, f2, f3, f4 = thrust  # thrust created by 4 propellers.

    # Rotation linear transformations.
    rotation = rotate_matrix_body_to_world(
        phi, theta, psi,
    )

    total_thrust = f1 + f2 + f3 + f4
    acceleration = (
            rotation
            @ np.array([
        0.0,
        0.0,
        total_thrust / m,
    ])
            - np.array([
        0.0,
        0.0,
        g,
    ])
    )

    torque = np.array([
        (L / np.sqrt(2.0)) * (f1 + f2 - f3 - f4),
        (L / np.sqrt(2.0)) * (-f1 + f2 + f3 - f4),
        gamma * (-f1 + f2 - f3 + f4),
    ])
    omega = np.array([
        p,
        q,
        r,
    ])
    omega_dot = (
            J_inv
            @ (
                    torque
                    - np.cross(
                omega,
                J @ omega,
            )
            )
    )

    euler_rate_matrix = np.array([
        [1.0, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
        [0.0, np.cos(phi), -np.sin(phi)],
        [0.0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)],
    ])
    euler_dot = euler_rate_matrix @ omega

    return np.array([
        # v_x, a_x
        x[STATE_VX],
        acceleration[0],
        # v_y, a_y
        x[STATE_VY],
        acceleration[1],
        # v_z, a_z
        x[STATE_VZ],
        acceleration[2],
        # rate of change roll, pitch, yaw over time (d_roll/dt, d_pitch/dt, d_yaw/dt)
        euler_dot[0],
        euler_dot[1],
        euler_dot[2],
        # rate of change p, q, r over time (dp/dt, dq/dt, dr/dt)
        omega_dot[0],
        omega_dot[1],
        omega_dot[2],
    ])


@njit(cache=True)
def rk4(state, thrust, dt, m, J, J_inv, L, gamma, g):
    """
    This function uses Runge-Kutta (RK4) method for numerical integration.

    It receives current state and the thrust (produced by PID), then computes
    the current rate of change of the current state given that thrust.
    Finally, with the current state + current rate of change of state, this function
    returns the next state after a tiny fraction of time -- which is 1/model_hz
    """
    k1 = dynamics(state, thrust, m, J, J_inv, L, gamma, g)
    k2 = dynamics(state + 0.5 * dt * k1, thrust, m, J, J_inv, L, gamma, g)
    k3 = dynamics(state + 0.5 * dt * k2, thrust, m, J, J_inv, L, gamma, g)
    k4 = dynamics(state + dt * k3, thrust, m, J, J_inv, L, gamma, g)

    avg_k = (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
    return state + avg_k * dt


@njit(cache=True)
def control_step(state, thrust, dt, substeps, m, J, J_inv, L, gamma, g):
    for _ in range(substeps):
        state = rk4(state, thrust, dt, m, J, J_inv, L, gamma, g)
    return state


class LightweightPredictiveModel:
    def __init__(self, quad_params, pid_params):
        if MODEL_FREQ_HZ % CTRL_FREQ_HZ != 0:
            raise ValueError(
                "model frequency must be divisible by controller frequency"
            )

        self.quad_params = quad_params
        self.pid_params = pid_params
        self.dt = 1.0 / MODEL_FREQ_HZ
        self.substeps = MODEL_FREQ_HZ // CTRL_FREQ_HZ
        self.ctrl = None

    def start(self, snapshot):
        info = {"current_step": 0}
        obs = np.asarray(snapshot.state, dtype=float).copy()
        reference = np.asarray(
            snapshot.remaining_reference,
            dtype=float,
        ).copy()

        self.ctrl = LightweightPID(
            reference=reference.copy(),
            quad_params=self.quad_params,
            pid_params=self.pid_params,
        )
        restore_pid_state(self.ctrl, snapshot.pid_state)
        return self.ctrl, obs, info, reference

    def step(self, action, obs, info):
        p = self.quad_params
        obs = control_step(
            obs,
            action,
            self.dt,
            self.substeps,
            p.mass,
            p.inertia,
            p.inertia_inv,
            p.arm_length,
            p.gamma,
            p.gravity,
        )
        info = deepcopy(info)
        info["current_step"] += 1
        return obs, False, info

    def save_checkpoint(self, ctrl, obs, info):
        return LightweightCheckpoint(
            obs=np.asarray(obs, dtype=float).copy(),
            info=deepcopy(info),
            pid_state=capture_pid_state(ctrl),
        )

    def restore_checkpoint(self, ctrl, checkpoint):
        restore_pid_state(ctrl, checkpoint.pid_state)
        return (
            checkpoint.obs.copy(),
            deepcopy(checkpoint.info),
        )

    def set_reference(self, ctrl, reference):
        ctrl.reference = np.asarray(
            reference,
            dtype=float,
        ).copy()

    def close(self, checkpoints):
        pass
