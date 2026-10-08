from numba import njit
from copy import deepcopy
from dataclasses import dataclass
from functools import partial
import numpy as np
from safe_control_gym.utils.registration import (
    get_config,
    make,
)
from model.common import (
    FREQ,
    EPISODE_SECONDS,
    PIDState,
    capture_pid_state,
    restore_pid_state,
)

MODEL_HZ = 100

# Same constants as SCG pid.py
KF = 3.16e-10

P_FOR = np.array([0.4, 0.4, 1.25])
I_FOR = np.array([0.05, 0.05, 0.05])
D_FOR = np.array([0.2, 0.2, 0.5])

P_TOR = np.array([70000.0, 70000.0, 60000.0])
I_TOR = np.array([0.0, 0.0, 500.0])
D_TOR = np.array([20000.0, 20000.0, 12000.0])

PWM2RPM_SCALE = 0.2685
PWM2RPM_CONST = 4070.3
MIN_PWM = 20000.0
MAX_PWM = 65535.0

DT_CTRL = 1.0 / FREQ


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
    last_rpy,  # roll-pitch-yaw in the previous step (to compute the roll-pitch-yaw rate in inertial frame)
    integral_rpy_e,  # accumulated roll-pitch-yaw error (altitude error)
    mass,  # Quadrotor weight
):
    # --------------------------------------------------
    # Current Quadrotor state
    # --------------------------------------------------
    cur_pos = np.array([obs[0], obs[2], obs[4]])  # x, y, z
    cur_vel = np.array([obs[1], obs[3], obs[5]])  # v_x, v_y, v_z
    cur_rpy = np.array([obs[6], obs[7], obs[8]])  # roll, pitch, yaw
    cur_rotation = rotate_matrix_body_to_world(
        cur_rpy[0],
        cur_rpy[1],
        cur_rpy[2],
    )

    # --------------------------------------------------
    # Reference
    # --------------------------------------------------
    target_pos = np.array([target[0], target[2], target[4]])  # target x, y, z
    target_vel = np.array([target[1], target[3], target[5]])  # target v_x, v_y, v_z

    # --------------------------------------------------
    # Position PID
    # Same equations as SCG
    # --------------------------------------------------
    pos_e = target_pos - cur_pos
    vel_e = target_vel - cur_vel

    integral_pos_e[:] += pos_e * DT_CTRL
    integral_pos_e[:] = np.clip(integral_pos_e, -2.0, 2.0)
    integral_pos_e[2] = min(0.15, max(-0.15, integral_pos_e[2]))

    # Target thrust vector [Fx, Fy, Fz] in Newton (inertial world frame)
    target_thrust = (
        P_FOR * pos_e  # P
        + I_FOR * integral_pos_e  # I
        + D_FOR * vel_e  # D
        + np.array([0.0, 0.0, 9.8 * mass])
    )

    # target_thrust: [Fx, Fy, Fz] in world frame
    # cur_rotation: body x-axis, y-axis, z-axis vectors expressed in world frame
    # scalar_thrust is projection of [Fx, Fy, Fz] to k_b
    # Summary: project the force to current body z-axis first, tilt the body z-axis to the target_force direction later.
    scalar_thrust = np.dot(target_thrust, cur_rotation[:, 2])
    if scalar_thrust < 0.0:
        scalar_thrust = 0.0

    # Convert scalar thrust to PWM (pulse-width modulation)
    thrust_pwm = (np.sqrt(scalar_thrust / (4.0 * KF)) - PWM2RPM_CONST) / PWM2RPM_SCALE

    # Desired body Z direction. We will titl current body z-axis to this target_zb_axis
    target_zb_axis = target_thrust / np.linalg.norm(target_thrust)
    # SCG target yaw = 0.
    target_x_c = np.array([
        1.0,
        0.0,
        0.0,
    ])
    # Right-hand rule, cross product of target body z-axis and x_c produce target body y-axis
    target_yb_axis = np.cross(
        target_zb_axis,
        target_x_c,
    )
    target_yb_axis /= np.linalg.norm(
        target_yb_axis
    )
    # Right-hand rule, cross product of target y-axis and z-axis produce target x-asxis
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
    ) / DT_CTRL

    last_rpy[:] = cur_rpy

    integral_rpy_e[:] -= (
        rot_e * DT_CTRL
    )
    integral_rpy_e[:] = np.clip(
        integral_rpy_e,
        -1500.0,
        1500.0,
    )
    integral_rpy_e[0] = min(
        1.0,
        max(-1.0, integral_rpy_e[0]),
    )
    integral_rpy_e[1] = min(
        1.0,
        max(-1.0, integral_rpy_e[1]),
    )

    target_torques = (
        -P_TOR * rot_e
        + D_TOR * rpy_rates_e
        + I_TOR * integral_rpy_e
    )

    target_torques = np.clip(
        target_torques,
        -3200.0,
        3200.0,
    )

    tx = target_torques[0]
    ty = target_torques[1]
    tz = target_torques[2]

    # Same mixer matrix as SCG pid.py
    pwm = np.array([
        thrust_pwm + 0.5 * tx - 0.5 * ty - tz,
        thrust_pwm + 0.5 * tx + 0.5 * ty + tz,
        thrust_pwm - 0.5 * tx + 0.5 * ty - tz,
        thrust_pwm - 0.5 * tx - 0.5 * ty + tz,
    ])
    pwm = np.clip(
        pwm,
        MIN_PWM,
        MAX_PWM,
    )
    rpm = (
        PWM2RPM_SCALE * pwm
        + PWM2RPM_CONST
    )
    # SCG PID returns motor thrust in Newtons.
    return KF * rpm ** 2


class LightweightPID:
    def __init__(
        self,
        reference,
        mass,
    ):
        self.reference = reference
        self.mass = mass
        self.integral_pos_e = np.zeros(3)
        self.last_rpy = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)

    def select_action(
        self,
        obs,
        info,
    ):
        step = info["current_step"]
        return pid_step(
            obs,
            self.reference[step],
            self.integral_pos_e,
            self.last_rpy,
            self.integral_rpy_e,
            self.mass,
        )


@dataclass(frozen=True)
class LightweightCheckpoint:
    obs: np.ndarray
    info: dict
    pid_state: PIDState


def controller_config(state):
    config = get_config(
        "quadrotor"
    )
    config.update({
        "quad_type": 3,
        "task": "traj_tracking",
        "cost": "quadratic",
        "ctrl_freq": FREQ,
        # Environment exists only for PID creation
        # and physical parameters.
        "pyb_freq": FREQ,
        "episode_len_sec":
            EPISODE_SECONDS,
        "init_state": np.asarray(
            state,
            dtype=float,
        ).copy(),
        "randomized_init": False,
        "randomized_inertial_prop": False,
        "disturbances": None,
        "adversary_disturbance": None,
        "done_on_out_of_bound": False,
        "gui": False,
    })
    return config


@njit(cache=True)
def dynamics(state, thrust, m, J, J_inv, L, gamma, g):
    """
    State: [x, vx, y, vy, z, vz, roll, pitch, yaw, p, q, r]
    Thrust: [f1, f2, f3, f4]
    Return rate of state change.
    """
    x = state

    phi, theta, psi = x[6], x[7], x[8]  # roll pitch yaw.
    p, q, r = x[9], x[10], x[11]  # angular velocity of Quadrotor expressed in body-frame.
    f1, f2, f3, f4 = thrust  # thrust created by 4 propellers.

    cphi, sphi = np.cos(phi), np.sin(phi)  # cos, sin of roll.
    ctheta, stheta = np.cos(theta), np.sin(theta)  # cos, sin of pitch.
    cpsi, spsi = np.cos(psi), np.sin(psi)  # cos, sin of yaw.

    # Rotation linear transformations.
    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cphi, -sphi],
        [0.0, sphi, cphi],
    ])
    ry = np.array([
        [ctheta, 0.0, stheta],
        [0.0, 1.0, 0.0],
        [-stheta, 0.0, ctheta],
    ])
    rz = np.array([
        [cpsi, -spsi, 0.0],
        [spsi, cpsi, 0.0],
        [0.0, 0.0, 1.0],
    ])
    rotation = rz @ ry @ rx

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
        [
            1.0,
            sphi * np.tan(theta),
            cphi * np.tan(theta),
        ],
        [
            0.0,
            cphi,
            -sphi,
        ],
        [
            0.0,
            sphi / ctheta,
            cphi / ctheta,
        ],
    ])

    euler_dot = euler_rate_matrix @ omega

    return np.array([
        # v_x, a_x
        x[1],
        acceleration[0],
        # v_y, a_y
        x[3],
        acceleration[1],
        # v_z, a_z
        x[5],
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


class LightweightQuadrotor:
    def __init__(self, env, model_hz=MODEL_HZ):
        if model_hz % FREQ != 0:
            raise ValueError(
                "model_hz must be divisible by controller frequency"
            )
        self.dt = 1.0 / model_hz
        self.substeps = model_hz // FREQ

        self.m = env.MASS
        self.J = np.asarray(env.J, dtype=np.float64)
        self.J_inv = np.linalg.inv(self.J)

        self.L = env.L
        self.gamma = env.KM / env.KF
        self.g = env.GRAVITY_ACC

    def control_step(self, state, thrust):
        return control_step(
            state,
            thrust,
            self.dt,
            self.substeps,
            self.m,
            self.J,
            self.J_inv,
            self.L,
            self.gamma,
            self.g,
        )


class LightweightPredictiveModel:
    def __init__(self):
        self.env = None
        self.ctrl = None
        self.model = None

    def start(self, snapshot):
        config = controller_config(snapshot.state)
        env_factory = partial(make, "quadrotor", **config)

        self.env = env_factory()
        self.model = LightweightQuadrotor(
            self.env,
            model_hz=MODEL_HZ,
        )
        info = {
            "current_step": 0,
        }

        # 12d state vector of Quadrotor snap-shotted at approval-time.
        obs = np.asarray(snapshot.state, dtype=float).copy()
        # Remaining reference from approval-time to scenario end.
        reference = np.asarray(
            snapshot.remaining_reference,
            dtype=float,
        ).copy()

        # Restore the state of Quad env, PID state to the approval snapshot.
        self.ctrl = LightweightPID(
            reference=reference.copy(),
            mass=self.env.MASS,
        )
        restore_pid_state(
            self.ctrl,
            snapshot.pid_state,
        )

        return (self.ctrl, obs, info, reference)

    def step(self, action, obs, info):
        obs = self.model.control_step(obs, action)
        info = deepcopy(info)
        info["current_step"] += 1

        return (obs, False, info)

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
        if self.env is not None:
            self.env.close()
