from dataclasses import dataclass
import numpy as np

from model.state import (
    STATE_X,
    STATE_Y,
    STATE_Z,
    STATE_VX
)


@dataclass
class Scenario:
    # Nominal flight
    speed: float
    usual_height: float
    # Structure
    box_x_min: float
    box_x_max: float
    box_y_limit: float
    unsafe_below_z: float
    # Approved command
    approval_time: float
    low_height: float
    low_seconds: float

def print_scenario(s):
    print(
        f"speed={s.speed:.2f} m/s, "
        f"height={s.usual_height:.2f} m"
    )
    print(
        f"structure x=[{s.box_x_min:.2f}, "
        f"{s.box_x_max:.2f}], "
        f"unsafe z<={s.unsafe_below_z:.2f}"
    )
    print(
        f"approval={s.approval_time:.2f}s, "
        f"command: z={s.low_height:.2f}m "
        f"for {s.low_seconds:.2f}s"
    )


def generate_scenario(seed):
    rng = np.random.default_rng(seed)

    speed = rng.uniform(0.4, 0.8)
    usual_height = rng.uniform(1.15, 1.40)

    structure_start = rng.uniform(0.8, 1.5)
    structure_length = rng.uniform(0.3, 0.6)

    unsafe_below_z = rng.uniform(0.90, 1.05)  # Drone will be unsafe under this z altitude.

    # Nominal requested height must be above the unsafe threshold.
    if usual_height <= unsafe_below_z + 0.10:
        usual_height = unsafe_below_z + 0.10

    # The dip command goes below the unsafe threshold,
    # so timing can potentially matter.
    low_height = rng.uniform(0.60, unsafe_below_z - 0.10)

    return Scenario(
        speed=speed,
        usual_height=usual_height,

        box_x_min=structure_start,
        box_x_max=structure_start + structure_length,
        box_y_limit=0.35,
        unsafe_below_z=unsafe_below_z,

        approval_time=rng.uniform(0.1, 0.5),
        low_height=low_height,
        low_seconds=rng.uniform(0.4, 1.0),
    )

ACCELERATION_SECONDS = 0.4
def make_forward_trajectory(env, s):
    n = env.X_GOAL.shape[0]
    dt = 1.0 / env.CTRL_FREQ
    times = np.arange(n) * dt
    # Gentle acceleration during first 0.4 s.
    speeds = s.speed * np.minimum(times/ACCELERATION_SECONDS, 1.0)
    traj = np.zeros_like(env.X_GOAL)
    traj[:, STATE_X] = np.cumsum(speeds) * dt  # x
    traj[:, STATE_VX] = speeds  # v_x
    traj[:, STATE_Z] = s.usual_height  # z
    return traj


def unsafe(state, s):
    x, y, z = state[STATE_X], state[STATE_Y], state[STATE_Z]
    return (
            s.box_x_min <= x <= s.box_x_max
            and abs(y) <= s.box_y_limit
            and z <= s.unsafe_below_z
    )

def apply_descent_command(
    reference,
    start_step,
    low_steps,
    low_height,
):
    """
    Command Quadrotor to descent to low_height (world frame z-axis) from [start_step, start_step + low_steps].
    Return: edited trajectory.
    """
    changed = reference.copy()
    end = min(
        start_step + low_steps,
        len(changed),
    )
    changed[start_step:end, STATE_Z] = low_height
    return changed
