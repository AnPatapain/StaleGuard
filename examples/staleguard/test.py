from functools import partial

from safe_control_gym.utils.registration import get_config, make

FREQ = 50 # 50 control_step/s
EPISODE_SECONDS = 5
USUAL_HEIGHT = 1.20

config = get_config("quadrotor")
config.update({
        "quad_type": 3,
        "task": "traj_tracking",
        "cost": "quadratic",
        "ctrl_freq": FREQ,
        "pyb_freq": 1000,
        "episode_len_sec": EPISODE_SECONDS,
        "init_state": {"init_x": 0.0, "init_y": 0.0, "init_z": USUAL_HEIGHT},
        "randomized_init": False,
        "randomized_inertial_prop": False,
        "disturbances": None,
        "adversary_disturbance": None,
        "done_on_out_of_bound": False,
        "gui": False,
    })

env_factory = partial(make, "quadrotor", **config)
env = env_factory()
ctrl = make("pid", env_factory)

print(env.X_GOAL.shape)