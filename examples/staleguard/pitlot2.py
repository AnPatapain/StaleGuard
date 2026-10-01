from functools import partial

import numpy as np

from safe_control_gym.utils.registration import get_config, make


FREQ = 50
EPISODE_SECONDS = 5
MEASURE_SECONDS = 4.5
APPROVAL_SECONDS = 0.2

FORWARD_SPEED = 0.6       # metres per second
USUAL_HEIGHT = 1.20       # metres
LOW_HEIGHT = 0.65         # metres
LOW_SECONDS = 0.6

# Fixed safety rule: structure footprint plus a 0.10 m height margin.
BOX_X_MIN = 0.85
BOX_X_MAX = 1.25
BOX_Y_LIMIT = 0.35
UNSAFE_BELOW_Z = 1.00

DELAYS = (0.0, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.5)


def make_forward_trajectory(env):
    """A forward trajectory at constant height, with a gentle start."""
    n = env.X_GOAL.shape[0]
    dt = 1.0 / env.CTRL_FREQ
    times = np.arange(n) * dt

    speeds = FORWARD_SPEED * np.minimum(times / 0.4, 1.0)

    traj = np.zeros_like(env.X_GOAL)
    traj[:, 0] = np.cumsum(speeds) * dt  # requested x
    traj[:, 1] = speeds                 # requested forward speed
    traj[:, 4] = USUAL_HEIGHT           # requested z
    return traj


def unsafe(state):
    """Check the drone's observed position, not its requested position."""
    x, y, z = state[0], state[2], state[4]
    return (
        BOX_X_MIN <= x <= BOX_X_MAX
        and abs(y) <= BOX_Y_LIMIT
        and z <= UNSAFE_BELOW_Z
    )


def first_unsafe_time(states, first_step, last_step):
    for step in range(first_step, last_step):
        if unsafe(states[step]):
            return step / FREQ
    return None


def run_one(delay_seconds):
    """None means: keep flying forward; never issue the lower-flight command."""
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

    env_func = partial(make, "quadrotor", **config)
    env = env_func()
    ctrl = None

    try:
        ctrl = make("pid", env_func)

        obs, info = env.reset(seed=42)
        ctrl.reset_before_run(obs=obs, info=info, env=env)

        traj = make_forward_trajectory(env)
        env.X_GOAL = traj.copy()
        ctrl.reference = traj.copy()

        approval_step = round(APPROVAL_SECONDS * FREQ)
        dispatch_step = (
            None if delay_seconds is None
            else approval_step + round(delay_seconds * FREQ)
        )
        low_steps = round(LOW_SECONDS * FREQ)
        total_steps = round(MEASURE_SECONDS * FREQ)

        states = [obs.copy()]

        for step in range(total_steps):
            # At this instant, replace only the requested height for 0.6 s.
            # All runs use the same height and duration.
            if step == dispatch_step:
                changed_traj = traj.copy()
                changed_traj[step:step + low_steps, 4] = LOW_HEIGHT
                env.X_GOAL = changed_traj
                ctrl.reference = changed_traj

            motor_action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(motor_action)
            states.append(obs.copy())

            if done:
                break

        states = np.asarray(states)

        if len(states) <= approval_step:
            raise RuntimeError("Flight ended before the approval time.")
        if dispatch_step is not None and len(states) <= dispatch_step:
            raise RuntimeError("Flight ended before the command could start.")

        wait_end = (
            len(states) if dispatch_step is None else dispatch_step + 1
        )
        hit_while_waiting = first_unsafe_time(
            states, approval_step, wait_end
        )
        hit_after_command = (
            None if dispatch_step is None
            else first_unsafe_time(states, dispatch_step + 1, len(states))
        )

        over_structure = (
            (states[:, 0] >= BOX_X_MIN)
            & (states[:, 0] <= BOX_X_MAX)
            & (np.abs(states[:, 2]) <= BOX_Y_LIMIT)
        )
        lowest_over_structure = (
            float(np.min(states[over_structure, 4]))
            if np.any(over_structure) else None
        )

        return {
            "approval": states[approval_step],
            "dispatch": (
                None if dispatch_step is None else states[dispatch_step]
            ),
            "max_x_while_waiting": float(
                np.max(states[approval_step:wait_end, 0])
            ),
            "hit_while_waiting": hit_while_waiting,
            "hit_after_command": hit_after_command,
            "lowest_over_structure": lowest_over_structure,
            "ended_early": len(states) != total_steps + 1,
        }
    finally:
        if ctrl is not None:
            ctrl.close()  # Closes the PID controller's own environment.
        env.close()


def show_time(value):
    return "none" if value is None else f"{value:.2f} s"


def show_height(value):
    return "never passed over it" if value is None else f"{value:.3f} m"


def show_state(state):
    return f"x={state[0]:.3f}, vx={state[1]:.3f}, z={state[4]:.3f}"


if __name__ == "__main__":
    print("Unsafe zone: 0.85 <= x <= 1.25, |y| <= 0.35, z <= 1.00")
    print("Command approved at 0.20 s; Command: descent to 0.65 m for 0.60s")
    print()

    for delay in (None, *DELAYS):
        result = run_one(delay)
        name = "NO COMMAND" if delay is None else f"DELAY {delay:.1f} s"

        print(name)
        print("  state at approval:       ", show_state(result["approval"]))
        if result["dispatch"] is not None:
            print("  state when command starts:", show_state(result["dispatch"]))
        print("  furthest x while waiting: ",
              f'{result["max_x_while_waiting"]:.3f} m')
        print("  unsafe while waiting:     ",
              show_time(result["hit_while_waiting"]))
        print("  unsafe after command:     ",
              show_time(result["hit_after_command"]))
        print("  lowest z over structure:  ",
              show_height(result["lowest_over_structure"]))
        if result["ended_early"]:
            print("  NOTE: simulation ended before 4.5 s")
        print()