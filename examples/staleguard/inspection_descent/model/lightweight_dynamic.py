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


class LightweightQuadrotor:

    def __init__(
        self,
        env,
        model_hz=MODEL_HZ,
    ):
        if model_hz % FREQ != 0:
            raise ValueError(
                "model_hz must be divisible "
                "by controller frequency"
            )

        self.dt = 1.0 / model_hz

        self.substeps = (
            model_hz // FREQ
        )

        self.m = env.MASS

        self.J = np.asarray(
            env.J,
            dtype=float,
        )

        self.J_inv = np.linalg.inv(
            self.J
        )

        self.L = env.L
        self.gamma = env.KM / env.KF
        self.g = env.GRAVITY_ACC

    def dynamics(
        self,
        state,
        thrust,
    ):
        """
        State:
        [x, vx,
         y, vy,
         z, vz,
         roll, pitch, yaw,
         p, q, r]

        Thrust:
        [f1, f2, f3, f4]
        """

        x = np.asarray(
            state,
            dtype=float,
        )

        phi = x[6]
        theta = x[7]
        psi = x[8]

        p = x[9]
        q = x[10]
        r = x[11]

        f1, f2, f3, f4 = thrust

        cphi = np.cos(phi)
        sphi = np.sin(phi)

        ctheta = np.cos(theta)
        stheta = np.sin(theta)

        cpsi = np.cos(psi)
        spsi = np.sin(psi)

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

        total_thrust = (
            f1 + f2 + f3 + f4
        )

        acceleration = (
            rotation
            @ np.array([
                0.0,
                0.0,
                total_thrust,
            ])
            / self.m
            - np.array([
                0.0,
                0.0,
                self.g,
            ])
        )

        torque = np.array([
            self.L
            / np.sqrt(2.0)
            * (
                f1
                + f2
                - f3
                - f4
            ),

            self.L
            / np.sqrt(2.0)
            * (
                -f1
                + f2
                + f3
                - f4
            ),

            self.gamma
            * (
                -f1
                + f2
                - f3
                + f4
            ),
        ])

        omega = np.array([
            p,
            q,
            r,
        ])

        omega_dot = (
            self.J_inv
            @ (
                torque
                - np.cross(
                    omega,
                    self.J @ omega,
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

        euler_dot = (
            euler_rate_matrix
            @ omega
        )

        return np.array([
            x[1],
            acceleration[0],

            x[3],
            acceleration[1],

            x[5],
            acceleration[2],

            euler_dot[0],
            euler_dot[1],
            euler_dot[2],

            omega_dot[0],
            omega_dot[1],
            omega_dot[2],
        ])

    def rk4(
        self,
        state,
        thrust,
    ):
        dt = self.dt

        k1 = self.dynamics(
            state,
            thrust,
        )

        k2 = self.dynamics(
            state + 0.5 * dt * k1,
            thrust,
        )

        k3 = self.dynamics(
            state + 0.5 * dt * k2,
            thrust,
        )

        k4 = self.dynamics(
            state + dt * k3,
            thrust,
        )

        return (
            state
            + dt
            / 6.0
            * (
                k1
                + 2.0 * k2
                + 2.0 * k3
                + k4
            )
        )

    def control_step(
        self,
        state,
        thrust,
    ):
        for _ in range(
            self.substeps
        ):
            state = self.rk4(
                state,
                thrust,
            )

        return state


class LightweightPredictiveModel:

    def __init__(self):
        self.env = None
        self.ctrl = None
        self.model = None

    def start(
        self,
        snapshot,
    ):
        config = controller_config(
            snapshot.state
        )

        env_factory = partial(
            make,
            "quadrotor",
            **config,
        )

        self.ctrl = make(
            "pid",
            env_factory,
        )

        self.env = self.ctrl.env

        _, info = self.env.reset(
            seed=0
        )

        self.model = LightweightQuadrotor(
            self.env,
            model_hz=MODEL_HZ,
        )

        obs = np.asarray(
            snapshot.state,
            dtype=float,
        ).copy()

        reference = np.asarray(
            snapshot.remaining_reference,
            dtype=float,
        ).copy()

        self.env.X_GOAL = (
            reference.copy()
        )

        self.ctrl.reference = (
            reference.copy()
        )

        self.ctrl.reset_before_run(
            obs=obs,
            info=info,
            env=self.env,
        )

        restore_pid_state(
            self.ctrl,
            snapshot.pid_state,
        )

        return (
            self.ctrl,
            obs,
            info,
            reference,
        )

    def step(
        self,
        action,
        obs,
        info,
    ):
        obs = self.model.control_step(
            obs,
            action,
        )

        info = deepcopy(
            info
        )

        info["current_step"] += 1

        return (
            obs,
            False,
            info,
        )

    def save_checkpoint(
        self,
        ctrl,
        obs,
        info,
    ):
        return LightweightCheckpoint(
            obs=np.asarray(
                obs,
                dtype=float,
            ).copy(),

            info=deepcopy(
                info
            ),

            pid_state=capture_pid_state(
                ctrl
            ),
        )

    def restore_checkpoint(
        self,
        ctrl,
        checkpoint,
    ):
        restore_pid_state(
            ctrl,
            checkpoint.pid_state,
        )

        return (
            checkpoint.obs.copy(),
            deepcopy(
                checkpoint.info
            ),
        )

    def set_reference(
        self,
        ctrl,
        reference,
    ):
        reference = np.asarray(
            reference,
            dtype=float,
        ).copy()

        self.env.X_GOAL = reference
        ctrl.reference = reference

    def close(
        self,
        checkpoints,
    ):
        if self.ctrl is not None:
            self.ctrl.close()