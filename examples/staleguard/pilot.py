from functools import partial
import numpy as np
from safe_control_gym.utils.configuration import ConfigFactory
from safe_control_gym.utils.registration import make


cfg = ConfigFactory().merge()
env_factory = partial(make, cfg.task, **cfg.task_config)
env = env_factory(gui=True)
ctrl = make(cfg.algo, env_factory)

OLD_WAYPOINT = (0.70, 0.0, 1.0)
APPROVED_WAYPOINT = (0.85, 0.0, 1.0)
X_LIMIT = 1.0                    # Virtual inspection boundary, metres
DT = 1.0 / env.CTRL_FREQ


def set_waypoint(point):
    """Change the supervisory command read by PID."""
    env.X_GOAL[[0, 2, 4]] = point       # x, y, z targets
    env.X_GOAL[[1, 3, 5]] = 0.0         # target velocities
    ctrl.reference = env.X_GOAL.copy()


def advance(obs, info, force_x=0.0):
    """One PID + drone-physics step."""
    # Input is normalized; 0.5 becomes 0.005 N with the YAML scale.
    env.set_adversary_control(np.array([force_x, 0.0, 0.0]))
    motor_action = ctrl.select_action(obs, info)
    return env.step(motor_action)


def run_trial(delay_seconds):
    obs, info = env.reset(seed=42)
    ctrl.reset_before_run()
    set_waypoint(OLD_WAYPOINT)

    # Fly for 0.5 s before the approval moment t1.
    for _ in range(round(0.5 / DT)):
        obs, _, done, info = advance(obs, info)
        if done:
            raise RuntimeError("Drone stopped before t1; adjust setup.")

    state_at_approval = (float(obs[0]), float(obs[1]))
    max_x_waiting = float(obs[0])

    # t1: the 0.85 m waypoint is proposed and approved, but NOT applied.
    pending_waypoint = APPROVED_WAYPOINT

    # Queue wait: PID still follows the OLD waypoint.
    for i in range(round(delay_seconds / DT)):
        elapsed = i * DT
        gust = 0.5 if 0.1 <= elapsed < 0.4 else 0.0
        obs, _, done, info = advance(obs, info, force_x=gust)
        max_x_waiting = max(max_x_waiting, float(obs[0]))
        if done:
            return delay_seconds, state_at_approval, max_x_waiting, None

    state_at_dispatch = (float(obs[0]), float(obs[1]))

    # t2: release the pending command. Only NOW does PID change target.
    set_waypoint(pending_waypoint)
    max_x_after_dispatch = float(obs[0])

    for _ in range(round(1.5 / DT)):
        obs, _, done, info = advance(obs, info)
        max_x_after_dispatch = max(max_x_after_dispatch, float(obs[0]))
        if done:
            break

    return (
        delay_seconds,
        state_at_approval,
        max_x_waiting,
        (state_at_dispatch, max_x_after_dispatch),
    )


try:
    for delay in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
        delay, at_approval, max_wait, dispatch = run_trial(delay)
        print(f"\ndelay={delay:.1f}s; state at approval (x, vx)={at_approval}")
        print(f"maximum x while waiting={max_wait:.3f}")

        if dispatch is None:
            print("Episode ended before dispatch.")
            continue

        at_dispatch, max_after = dispatch
        print(f"state at dispatch (x, vx)={at_dispatch}")
        print(f"maximum x after command={max_after:.3f}")
        print(f"boundary crossed after command={max_after > X_LIMIT}")
finally:
    ctrl.close()
    env.close()