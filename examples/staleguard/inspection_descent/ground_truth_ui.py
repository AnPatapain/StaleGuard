from dataclasses import dataclass
from functools import partial
import time

import numpy as np
import pybullet as p

from safe_control_gym.utils.registration import get_config, make


FREQ = 50
EPISODE_SECONDS = 6
MEASURE_SECONDS = 5.5

# 1.0 = approximately real time.
# 2.0 = twice as slow, easier to watch.
GUI_SLOWDOWN = 1.5


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


def generate_scenario(seed):
    rng = np.random.default_rng(seed)

    speed = rng.uniform(0.4, 0.8)
    usual_height = rng.uniform(1.15, 1.40)

    structure_start = rng.uniform(0.8, 1.5)
    structure_length = rng.uniform(0.3, 0.6)

    unsafe_below_z = rng.uniform(0.90, 1.05)

    if usual_height <= unsafe_below_z + 0.10:
        usual_height = unsafe_below_z + 0.10

    low_height = rng.uniform(
        0.60,
        unsafe_below_z - 0.10,
    )

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


def make_forward_trajectory(env, s):
    n = env.X_GOAL.shape[0]
    dt = 1.0 / env.CTRL_FREQ
    times = np.arange(n) * dt

    # Gentle acceleration during first 0.4 s.
    speeds = s.speed * np.minimum(
        times / 0.4,
        1.0,
    )

    traj = np.zeros_like(env.X_GOAL)

    traj[:, 0] = np.cumsum(speeds) * dt  # x
    traj[:, 1] = speeds                  # x velocity
    traj[:, 4] = s.usual_height          # z

    return traj


def unsafe(state, s):
    x = state[0]
    y = state[2]
    z = state[4]

    return (
        s.box_x_min <= x <= s.box_x_max
        and abs(y) <= s.box_y_limit
        and z <= s.unsafe_below_z
    )


def add_gui_scene(env, s):
    """
    Add the visual representation of the unsafe region.

    IMPORTANT:
    This is VISUAL ONLY.
    It has no collision geometry and does not change the physics.
    """
    client = env.PYB_CLIENT

    x_size = s.box_x_max - s.box_x_min
    y_size = 2.0 * s.box_y_limit
    z_size = s.unsafe_below_z

    center_x = (
        s.box_x_min + s.box_x_max
    ) / 2.0

    center_z = z_size / 2.0

    # --------------------------------------------------
    # 1. Translucent unsafe volume
    # --------------------------------------------------

    visual_shape = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=[
            x_size / 2.0,
            y_size / 2.0,
            z_size / 2.0,
        ],
        # More transparent than before so we can see
        # the drone clearly.
        rgbaColor=[1.0, 0.0, 0.0, 0.12],
        physicsClientId=client,
    )

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=visual_shape,
        basePosition=[
            center_x,
            0.0,
            center_z,
        ],
        physicsClientId=client,
    )

    # --------------------------------------------------
    # 3. Labels
    # --------------------------------------------------

    p.addUserDebugText(
        text=(
            "UNSAFE VOLUME\n"
            f"z <= {s.unsafe_below_z:.2f} m"
        ),
        textPosition=[
            s.box_x_max + 0.05,
            0.0,
            s.unsafe_below_z + 0.05,
        ],
        textColorRGB=[1, 0, 0],
        textSize=1.2,
        physicsClientId=client,
    )

    # --------------------------------------------------
    # 4. Side camera
    #
    # Look approximately along the y axis.
    #
    # Screen:
    #       z ↑
    #
    #         |
    #         |
    #         +----------→ x
    # --------------------------------------------------


    p.resetDebugVisualizerCamera(
      cameraDistance=2.2,
      cameraYaw=20,
      cameraPitch=-12,
      cameraTargetPosition=[
          center_x,
          0.0,
          0.75,
      ],
      physicsClientId=client,
    )


def create_state_marker(env, obs):
    """
    Small sphere at the exact (x, y, z) state point used
    by unsafe().

    This removes ambiguity caused by the visible dimensions
    of the quadrotor model.
    """
    client = env.PYB_CLIENT

    marker_visual = p.createVisualShape(
        shapeType=p.GEOM_SPHERE,
        radius=0.011,
        rgbaColor=[0.0, 1.0, 0.0, 1.0],
        physicsClientId=client,
    )

    marker_id = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=-1,
        baseVisualShapeIndex=marker_visual,
        basePosition=[
            float(obs[0]),
            float(obs[2]),
            float(obs[4]),
        ],
        physicsClientId=client,
    )

    return marker_id


def update_state_marker(
    env,
    marker_id,
    obs,
    is_unsafe,
):
    """
    Move the state marker with the actual drone state.

    Green = state point is safe.
    Red   = state point is unsafe.
    """
    client = env.PYB_CLIENT

    p.resetBasePositionAndOrientation(
        marker_id,
        [
            float(obs[0]),
            float(obs[2]),
            float(obs[4]),
        ],
        [0, 0, 0, 1],
        physicsClientId=client,
    )

    if is_unsafe:
        color = [1.0, 0.0, 0.0, 1.0]
    else:
        color = [0.0, 1.0, 0.0, 1.0]

    p.changeVisualShape(
        marker_id,
        -1,
        rgbaColor=color,
        physicsClientId=client,
    )


def set_drone_label(
    env,
    old_label_id,
    text,
    color,
):
  return p.addUserDebugText(
    text=text,
    textPosition=[0, 0, 0.12],
    textColorRGB=color,
    textSize=1.0,
    parentObjectUniqueId=env.DRONE_ID,
    parentLinkIndex=-1,
    replaceItemUniqueId=old_label_id,
    physicsClientId=env.PYB_CLIENT,
  )


def update_clearance_line(
    env,
    old_line_id,
    obs,
    s,
    is_unsafe,
):
    """
    Draw the vertical distance between the drone state point
    and the unsafe altitude threshold.

    Positive clearance -> safe altitude.
    Negative clearance -> below unsafe altitude.
    """
    x = float(obs[0])
    z = float(obs[4])

    if is_unsafe:
        color = [1, 0, 0]
    else:
        color = [0, 1, 0]

    return p.addUserDebugLine(
        lineFromXYZ=[
            x,
            0.0,
            s.unsafe_below_z,
        ],
        lineToXYZ=[
            x,
            0.0,
            z,
        ],
        lineColorRGB=color,
        lineWidth=3,
        replaceItemUniqueId=old_line_id,
        physicsClientId=env.PYB_CLIENT,
    )


def get_phase(
    step,
    approval_step,
    dispatch_step,
    low_steps,
):
    if step < approval_step:
        return "NOMINAL"

    if step < dispatch_step:
        return "APPROVED / WAITING"

    if step < dispatch_step + low_steps:
        return "DIP ACTIVE"

    return "RECOVERY"


def run_gui(s, delay):
    # --------------------------------------------------
    # Environment configuration
    # --------------------------------------------------

    config = get_config("quadrotor")

    config.update({
        "quad_type": 3,
        "task": "traj_tracking",
        "cost": "quadratic",
        "ctrl_freq": FREQ,
        "pyb_freq": 1000,
        "episode_len_sec": EPISODE_SECONDS,

        "init_state": {
            "init_x": 0.0,
            "init_y": 0.0,
            "init_z": s.usual_height,
        },

        "randomized_init": False,
        "randomized_inertial_prop": False,
        "disturbances": None,
        "adversary_disturbance": None,
        "done_on_out_of_bound": False,
    })

    # Actual flight environment -> GUI.
    gui_config = config.copy()
    gui_config["gui"] = True

    # PID internally creates another environment.
    # Keep that environment hidden.
    ctrl_config = config.copy()
    ctrl_config["gui"] = False

    env_generator = partial(
        make,
        "quadrotor",
        **gui_config,
    )

    ctrl_env_generator = partial(
        make,
        "quadrotor",
        **ctrl_config,
    )

    env = env_generator()
    ctrl = None

    try:
        ctrl = make(
            "pid",
            ctrl_env_generator,
        )

        obs, info = env.reset(seed=42)

        ctrl.reset_before_run(
            obs=obs,
            info=info,
            env=env,
        )

        # --------------------------------------------------
        # Reference trajectory
        # --------------------------------------------------

        traj = make_forward_trajectory(
            env,
            s,
        )

        env.X_GOAL = traj.copy()
        ctrl.reference = traj.copy()

        # --------------------------------------------------
        # Timing
        # --------------------------------------------------

        total_steps = round(
            MEASURE_SECONDS * FREQ
        )

        approval_step = round(
            s.approval_time * FREQ
        )

        dispatch_step = (
            approval_step
            + round(delay * FREQ)
        )

        low_steps = round(
            s.low_seconds * FREQ
        )

        # --------------------------------------------------
        # GUI
        # --------------------------------------------------

        add_gui_scene(
            env,
            s,
        )

        marker_id = create_state_marker(
            env,
            obs,
        )

        label_id = -1
        clearance_line_id = -1

        # --------------------------------------------------
        # Console information
        # --------------------------------------------------

        print()
        print("=" * 70)
        print("GUI DEMO")

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
            f"approval={s.approval_time:.2f}s"
        )

        print(
            f"delay={delay:.2f}s"
        )

        print(
            f"dispatch="
            f"{s.approval_time + delay:.2f}s"
        )

        print(
            f"command: z={s.low_height:.2f}m "
            f"for {s.low_seconds:.2f}s"
        )

        print("=" * 70)

        # --------------------------------------------------
        # Simulation
        # --------------------------------------------------

        ever_unsafe = False

        for step in range(total_steps):

            t = step / FREQ

            # ----------------------------------------------
            # Approval
            # ----------------------------------------------

            if step == approval_step:
                print(
                    f"[t={t:.2f}] COMMAND APPROVED"
                )

            # ----------------------------------------------
            # Dispatch
            # ----------------------------------------------

            if step == dispatch_step:

                changed_traj = traj.copy()

                changed_traj[
                    step:step + low_steps,
                    4
                ] = s.low_height

                env.X_GOAL = changed_traj
                ctrl.reference = changed_traj

                print(
                    f"[t={t:.2f}] COMMAND DISPATCHED"
                )

            # ----------------------------------------------
            # PID control + physics
            # ----------------------------------------------

            motor_action = ctrl.select_action(
                obs,
                info,
            )

            obs, _, done, info = env.step(
                motor_action
            )

            # ----------------------------------------------
            # Current physical state
            # ----------------------------------------------

            x = float(obs[0])
            y = float(obs[2])
            z = float(obs[4])

            clearance = (
                z - s.unsafe_below_z
            )

            inside_x = (
                s.box_x_min
                <= x
                <= s.box_x_max
            )

            inside_y = (
                abs(y)
                <= s.box_y_limit
            )

            is_unsafe = unsafe(
                obs,
                s,
            )

            phase = get_phase(
                step,
                approval_step,
                dispatch_step,
                low_steps,
            )

            # ----------------------------------------------
            # Update exact state-point marker
            # ----------------------------------------------

            update_state_marker(
                env,
                marker_id,
                obs,
                is_unsafe,
            )

            # ----------------------------------------------
            # Update clearance line
            # ----------------------------------------------

            clearance_line_id = (
                update_clearance_line(
                    env,
                    clearance_line_id,
                    obs,
                    s,
                    is_unsafe,
                )
            )

            # ----------------------------------------------
            # Live status label
            # ----------------------------------------------

            if is_unsafe:
                safety_text = "UNSAFE"
                label_color = [1.0, 0.0, 0.0]
            else:
                safety_text = "SAFE"
                label_color = [0.0, 1.0, 0.0]

            label_id = set_drone_label(
              env,
              label_id,
              (
                  f"{phase}\n"
                  f"{safety_text}\n"
                  f"x={x:.2f}  z={z:.2f}\n"
                  f"clearance={clearance:+.2f} m"
              ),
              label_color,
            )

            # ----------------------------------------------
            # Print first unsafe event
            # ----------------------------------------------

            if is_unsafe and not ever_unsafe:

                ever_unsafe = True

                print(
                    f"[t={(step + 1) / FREQ:.2f}] "
                    f"UNSAFE: "
                    f"x={x:.3f}, "
                    f"y={y:.3f}, "
                    f"z={z:.3f}"
                )

            # ----------------------------------------------
            # Optional useful transition logging
            # ----------------------------------------------

            if inside_x and inside_y:
                # You can uncomment this if you want to inspect
                # every state while horizontally inside the volume.
                #
                # print(
                #     f"t={(step + 1)/FREQ:.2f} "
                #     f"x={x:.3f} "
                #     f"z={z:.3f} "
                #     f"clearance={clearance:+.3f}"
                # )
                pass

            # Slow wall-clock execution for visualization.
            time.sleep(
                GUI_SLOWDOWN / FREQ
            )

            if done:
                break

        print()
        print(
            "FINAL RESULT:",
            "UNSAFE occurred"
            if ever_unsafe
            else "SAFE",
        )

        # Keep final frame visible.
        time.sleep(2.0)

    finally:

        if ctrl is not None:
            ctrl.close()

        env.close()


if __name__ == "__main__":

    # --------------------------------------------------
    # Select scenario + delay to visualize.
    # --------------------------------------------------

    SCENARIO_ID = 1
    DELAY = 1.5

    scenario = generate_scenario(
        seed=SCENARIO_ID
    )

    run_gui(
        scenario,
        delay=DELAY,
    )