from functools import partial
import numpy as np
from safe_control_gym.utils.registration import (
    get_config,
    make,
)
from config import (
    CTRL_FREQ_HZ,
    PYB_FREQ_HZ,
    EPISODE_SECONDS,
)
from model.params import (
    QuadParams,
    PIDParams,
)


def make_scg_config(
    init_state,
    pyb_freq=PYB_FREQ_HZ,
):
    config = get_config(
        "quadrotor"
    )

    if isinstance(init_state, np.ndarray):
        init_state = np.asarray(
            init_state,
            dtype=float,
        ).copy()

    config.update({
        "quad_type": 3,
        "task": "traj_tracking",
        "cost": "quadratic",

        "ctrl_freq": CTRL_FREQ_HZ,
        "pyb_freq": pyb_freq,

        "episode_len_sec":
            EPISODE_SECONDS,

        "init_state": init_state,

        "randomized_init": False,
        "randomized_inertial_prop": False,

        "disturbances": None,
        "adversary_disturbance": None,

        "done_on_out_of_bound": False,
        "gui": False,
    })

    return config


def make_scg_env_factory(
    init_state,
    pyb_freq=PYB_FREQ_HZ,
):
    config = make_scg_config(
        init_state=init_state,
        pyb_freq=pyb_freq,
    )

    return partial(
        make,
        "quadrotor",
        **config,
    )


def quad_params_from_env(env):
    inertia = np.asarray(
        env.J,
        dtype=np.float64,
    ).copy()

    return QuadParams(
        mass=float(env.MASS),

        inertia=inertia,
        inertia_inv=np.linalg.inv(
            inertia
        ),

        arm_length=float(env.L),

        kf=float(env.KF),
        km=float(env.KM),

        gravity=float(
            env.GRAVITY_ACC
        ),
    )


def _scalar(value):
    return float(
        np.asarray(value).item()
    )


def load_scg_platform_params():
    """
    Load the physical vehicle parameters and PID parameters
    from Safe-Control-Gym once at application startup.
    """

    env_factory = make_scg_env_factory(
        init_state={
            "init_x": 0.0,
            "init_y": 0.0,
            "init_z": 1.0,
        },
    )

    ctrl = make(
        "pid",
        env_factory,
    )

    try:
        env = ctrl.env

        quad_params = (
            quad_params_from_env(env)
        )

        pid_params = PIDParams(
            control_dt=float(
                ctrl.control_timestep
            ),

            kf=float(ctrl.KF),
            gravity=float(ctrl.g),

            p_for=np.asarray(
                ctrl.P_COEFF_FOR,
                dtype=np.float64,
            ).copy(),

            i_for=np.asarray(
                ctrl.I_COEFF_FOR,
                dtype=np.float64,
            ).copy(),

            d_for=np.asarray(
                ctrl.D_COEFF_FOR,
                dtype=np.float64,
            ).copy(),

            p_tor=np.asarray(
                ctrl.P_COEFF_TOR,
                dtype=np.float64,
            ).copy(),

            i_tor=np.asarray(
                ctrl.I_COEFF_TOR,
                dtype=np.float64,
            ).copy(),

            d_tor=np.asarray(
                ctrl.D_COEFF_TOR,
                dtype=np.float64,
            ).copy(),

            pwm2rpm_scale=_scalar(
                ctrl.PWM2RPM_SCALE
            ),

            pwm2rpm_const=_scalar(
                ctrl.PWM2RPM_CONST
            ),

            min_pwm=_scalar(
                ctrl.MIN_PWM
            ),

            max_pwm=_scalar(
                ctrl.MAX_PWM
            ),

            # These are literals inside
            # SCG's pid.py implementation.
            pos_integral_limit=2.0,
            z_integral_limit=0.15,

            attitude_integral_limit=1500.0,
            roll_pitch_integral_limit=1.0,

            torque_limit=3200.0,

            mixer_matrix=np.asarray(
                ctrl.MIXER_MATRIX,
                dtype=np.float64,
            ).copy(),

            target_yaw=0.0,
        )

        return (
            quad_params,
            pid_params,
        )

    finally:
        ctrl.close()