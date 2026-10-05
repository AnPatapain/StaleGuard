"""T0 mission runner.
Runs the real mission to approval once, captures the approval-time snapshot,
then asks StaleGuard to predict from that snapshot only.

Usage:
    python mission.py --scenario 0
    python mission.py --count 10
"""
import argparse
from functools import partial
import numpy as np
from safe_control_gym.utils.registration import get_config, make
import ground_truth
from staleguard import (
    FREQ,
    EPISODE_SECONDS,
    MEASURE_SECONDS,
    DELAYS,
    PIDState,
    ObstacleMap,
    ApprovedCommand,
    ApprovalSnapshot,
    predict,
)

SEED = 42


def forward_trajectory(env, s):
    # Intentionally identical to frozen ground_truth.py.
    n = env.X_GOAL.shape[0]
    dt = 1.0 / env.CTRL_FREQ
    times = np.arange(n) * dt

    speeds = s.speed * np.minimum(times / 0.4, 1.0)

    traj = np.zeros_like(env.X_GOAL)
    traj[:, 0] = np.cumsum(speeds) * dt
    traj[:, 1] = speeds
    traj[:, 4] = s.usual_height

    return traj


def start_mission_simulator(s, seed=SEED):
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
        "gui": False,
    })

    env_factory = partial(make, "quadrotor", **config)
    env = env_factory()
    ctrl = None

    try:
        ctrl = make("pid", env_factory)
        obs, info = env.reset(seed=seed)
        ctrl.reset_before_run(
            obs=obs,
            info=info,
            env=env,
        )

        traj = forward_trajectory(env, s)

        env.X_GOAL = traj.copy()
        ctrl.reference = traj.copy()

        return env, ctrl, obs, info, traj

    except Exception:
        if ctrl is not None:
            ctrl.close()
        env.close()
        raise


def capture_approval(s, seed=SEED):
    """Actually fly to approval and capture the state at that instant."""
    env, ctrl, obs, info, traj = start_mission_simulator(
        s,
        seed,
    )
    approval_step = round(
        s.approval_time * FREQ
    )
    total_steps = round(
        MEASURE_SECONDS * FREQ
    )

    try:
        # This is REAL mission execution, not StaleGuard replay.
        for _ in range(approval_step):
            action = ctrl.select_action(
                obs,
                info,
            )
            obs, _, done, info = env.step(
                action
            )
            if done:
                raise RuntimeError(
                    "Mission terminated before approval"
                )

        snapshot = ApprovalSnapshot(
            state=np.asarray(
                obs
            ).copy(),
            pid_state=PIDState(
                integral_pos_e=np.asarray(
                    ctrl.integral_pos_e
                ).copy(),
                last_rpy=np.asarray(
                    ctrl.last_rpy
                ).copy(),
                integral_rpy_e=np.asarray(
                    ctrl.integral_rpy_e
                ).copy(),
            ),

            # Rebase reference:
            # index 0 now means approval time.
            remaining_reference=np.asarray(
                traj[approval_step:]
            ).copy(),

            command=ApprovedCommand(
                low_height=float(
                    s.low_height
                ),
                low_steps=round(
                    s.low_seconds * FREQ
                ),
            ),

            obstacle=ObstacleMap(
                x_min=float(
                    s.box_x_min
                ),
                x_max=float(
                    s.box_x_max
                ),
                y_limit=float(
                    s.box_y_limit
                ),
                unsafe_below_z=float(
                    s.unsafe_below_z
                ),
            ),

            approval_step=approval_step,
            horizon_steps=(
                    total_steps
                    - approval_step
            ),
        )

        return snapshot

    finally:
        ctrl.close()
        env.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--count",
        type=int,
        default=None,
    )
    args = parser.parse_args()

    ids = (
        range(args.count)
        if args.count is not None
        else [args.scenario]
    )
    for scenario_id in ids:
        s = ground_truth.generate_scenario(
            scenario_id
        )
        # ----------------------------
        # ONLINE / StaleGuard
        # ----------------------------
        snapshot = capture_approval(s)
        predicted = predict(
            snapshot,
            DELAYS,
        )
        # ----------------------------
        # OFFLINE ground truth
        # ----------------------------
        actual = (
            ground_truth.classify_scenario(s)
        )

        print(
            f"\nSCENARIO {scenario_id}"
        )
        print(
            "delay   predicted      actual"
        )
        if (
                predicted["results"] is not None
                and actual["results"] is not None
        ):
            for d, pred, gt in zip(
                    DELAYS,
                    predicted["results"],
                    actual["results"],
            ):
                print(
                    f"{d:>4.1f}s   "
                    f"{pred:<14} "
                    f"{gt}"
                )
        print(
            f"class: "
            f"{predicted['classification']} / "
            f"{actual['classification']}"
        )
        print(
            f"first bad: "
            f"{predicted['first_bad_delay']} / "
            f"{actual['first_bad_delay']}"
        )
        print(
            f"last safe: "
            f"{predicted['last_safe_delay']} / "
            f"{actual['last_safe_delay']}"
        )
        match = (
                predicted["classification"]
                == actual["classification"]

                and predicted["results"]
                == actual["results"]

                and predicted["first_bad_delay"]
                == actual["first_bad_delay"]

                and predicted["last_safe_delay"]
                == actual["last_safe_delay"]
        )
        print(
            "MATCH"
            if match
            else "MISMATCH"
        )

if __name__ == "__main__":
    main()