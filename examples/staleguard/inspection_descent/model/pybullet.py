from copy import deepcopy
from dataclasses import dataclass
import numpy as np
import pybullet as p
from config import PYB_FREQ_HZ
from model.scg import make_scg_env_factory
from safe_control_gym.utils.registration import (
    make,
)
from model.common import (
    PIDState,
    capture_pid_state,
    restore_pid_state,
)
from model.state import (
    STATE_X,
    STATE_VX,
    STATE_Y,
    STATE_VY,
    STATE_Z,
    STATE_VZ,
    STATE_ROLL,
    STATE_YAW,
    STATE_P,
    STATE_R,
)


@dataclass(frozen=True)
class RolloutCheckpoint:
    pyb_state_id: int
    obs: np.ndarray
    info: dict
    pid_state: PIDState
    ctrl_step_counter: int
    pyb_step_counter: int


def restore_exact_state(
    env,
    state,
):
    state = np.asarray(
        state,
        dtype=float,
    )

    position = np.array([
        state[STATE_X],
        state[STATE_Y],
        state[STATE_Z],
    ])

    velocity_world = np.array([
        state[STATE_VX],
        state[STATE_VY],
        state[STATE_VZ],
    ])

    rpy = state[
        STATE_ROLL:
        STATE_YAW + 1
    ]

    body_rates = state[
        STATE_P:
        STATE_R + 1
    ]

    quat = p.getQuaternionFromEuler(
        rpy
    )

    rotation_world_from_body = np.asarray(
        p.getMatrixFromQuaternion(
            quat
        )
    ).reshape(3, 3)

    angular_velocity_world = (
        rotation_world_from_body
        @ body_rates
    )

    p.resetBasePositionAndOrientation(
        env.DRONE_ID,
        position,
        quat,
        physicsClientId=env.PYB_CLIENT,
    )

    p.resetBaseVelocity(
        env.DRONE_ID,
        linearVelocity=velocity_world,
        angularVelocity=angular_velocity_world,
        physicsClientId=env.PYB_CLIENT,
    )

    env._update_and_store_kinematic_information()

    obs = env._get_observation()

    return np.asarray(
        obs
    ).copy()


def save_checkpoint(
    env,
    ctrl,
    obs,
    info,
):
    return RolloutCheckpoint(
        pyb_state_id=p.saveState(
            physicsClientId=env.PYB_CLIENT
        ),

        obs=np.asarray(
            obs
        ).copy(),

        info=deepcopy(
            info
        ),

        pid_state=capture_pid_state(
            ctrl
        ),

        ctrl_step_counter=(
            env.ctrl_step_counter
        ),

        pyb_step_counter=(
            env.pyb_step_counter
        ),
    )


def restore_checkpoint(
    env,
    ctrl,
    checkpoint,
):
    p.restoreState(
        stateId=checkpoint.pyb_state_id,
        physicsClientId=env.PYB_CLIENT,
    )

    env.ctrl_step_counter = (
        checkpoint.ctrl_step_counter
    )

    env.pyb_step_counter = (
        checkpoint.pyb_step_counter
    )

    env._update_and_store_kinematic_information()

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


def start_from_approval(
    snapshot,
):
    env_factory = (
        make_scg_env_factory(
            init_state=(
                snapshot.state
            ),
            pyb_freq=PYB_FREQ_HZ,
        )
    )
    ctrl = None

    try:
        ctrl = make(
            "pid",
            env_factory,
        )

        env = ctrl.env

        obs, info = env.reset(
            seed=0
        )

        obs = restore_exact_state(
            env,
            snapshot.state,
        )

        reference = np.asarray(
            snapshot.remaining_reference
        ).copy()

        env.X_GOAL = reference.copy()
        ctrl.reference = reference.copy()

        ctrl.reset_before_run(
            obs=obs,
            info=info,
            env=env,
        )

        restore_pid_state(
            ctrl,
            snapshot.pid_state,
        )

        if not np.allclose(
            obs,
            snapshot.state,
            rtol=1e-7,
            atol=1e-8,
        ):
            raise RuntimeError(
                "Could not reconstruct "
                "approval-time state"
            )

        return (
            env,
            ctrl,
            obs,
            info,
            reference,
        )

    except Exception:
        if ctrl is not None:
            ctrl.close()

        raise


class PyBullet1000HzPredictiveModel:
    """
    Current StaleGuard predictive model.

    Physics:
        SCG + PyBullet @ 1000 Hz

    Controller:
        SCG PID @ 50 Hz
    """

    def __init__(self):
        self.env = None
        self.ctrl = None

    def start(
        self,
        snapshot,
    ):
        (
            self.env,
            self.ctrl,
            obs,
            info,
            reference,
        ) = start_from_approval(
            snapshot
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
        obs, _, done, info = (
            self.env.step(
                action
            )
        )

        return (
            obs,
            done,
            info,
        )

    def save_checkpoint(
        self,
        ctrl,
        obs,
        info,
    ):
        return save_checkpoint(
            self.env,
            ctrl,
            obs,
            info,
        )

    def restore_checkpoint(
        self,
        ctrl,
        checkpoint,
    ):
        return restore_checkpoint(
            self.env,
            ctrl,
            checkpoint,
        )

    def set_reference(
        self,
        ctrl,
        reference,
    ):
        reference = np.asarray(
            reference
        ).copy()

        self.env.X_GOAL = reference
        ctrl.reference = reference

    def close(
        self,
        checkpoints,
    ):
        if self.env is not None:
            for checkpoint in checkpoints.values():
                p.removeState(
                    checkpoint.pyb_state_id,
                    physicsClientId=(
                        self.env.PYB_CLIENT
                    ),
                )

        if self.ctrl is not None:
            self.ctrl.close()