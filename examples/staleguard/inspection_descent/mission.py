"""T0 mission runner.
Runs the real mission to approval once, captures the approval-time snapshot,
then asks StaleGuard to predict from that snapshot only.

Usage:
    python mission.py --scenario 0
    python mission.py --count 10
"""
import time
import argparse
import numpy as np
from config import CTRL_FREQ_HZ, MEASURE_SECONDS, DELAY_GRID, PYB_FREQ_HZ, SIM_SEED
from model.common import capture_pid_state
from model.scg import make_scg_env_factory, load_scg_platform_params
from safe_control_gym.utils.registration import make
import ground_truth
from scenario import generate_scenario, make_forward_trajectory, print_scenario
from staleguard import (
    ObstacleMap,
    ApprovedCommand,
    ApprovalSnapshot,
    predict,
)
from model.lightweight_dynamic import LightweightPredictiveModel


def start_mission_simulator(
    s,
    seed=SIM_SEED,
):
    env_factory = (
        make_scg_env_factory(
            init_state={
                "init_x": 0.0,
                "init_y": 0.0,
                "init_z":
                    s.usual_height,
            },
            pyb_freq=PYB_FREQ_HZ,
        )
    )
    env = env_factory()
    ctrl = None
    try:
        ctrl = make(
            "pid",
            env_factory,
        )
        obs, info = env.reset(
            seed=seed
        )
        ctrl.reset_before_run(
            obs=obs,
            info=info,
            env=env,
        )
        traj = (
            make_forward_trajectory(
                env,
                s,
            )
        )
        env.X_GOAL = traj.copy()
        ctrl.reference = traj.copy()
        return (
            env,
            ctrl,
            obs,
            info,
            traj,
        )

    except Exception:
        if ctrl is not None:
            ctrl.close()
        env.close()
        raise


def capture_approval(s, seed=SIM_SEED):
    """Actually fly to approval and capture the state at that instant."""
    env, ctrl, obs, info, traj = start_mission_simulator(
        s,
        seed,
    )
    approval_step = round(
        s.approval_time * CTRL_FREQ_HZ
    )
    total_steps = round(
        MEASURE_SECONDS * CTRL_FREQ_HZ
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
            pid_state=capture_pid_state(ctrl),

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
                    s.low_seconds * CTRL_FREQ_HZ
                ),
            ),

            obstacle=ObstacleMap(
                box_x_min=float(
                    s.box_x_min
                ),
                box_x_max=float(
                    s.box_x_max
                ),
                box_y_limit=float(
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
        default=None,
    )
    parser.add_argument(
        "--count",
        type=int,
        default=100,  # run 100 configurations by default
    )
    args = parser.parse_args()
    quad_params, pid_params = (
        load_scg_platform_params()
    )

    ids = (
        [args.scenario]
        if args.scenario is not None
        else range(args.count)
    )

    # Delay-level classification metrics.
    false_safe = 0  # predicted SAFE, actually unsafe
    false_reject = 0  # predicted unsafe, actually SAFE
    total_delay_tests = 0
    actual_unsafe = 0
    actual_safe = 0

    # Scenario-level metrics.
    scenarios_with_false_safe = 0
    class_correct = 0
    class_total = 0

    # TTL error only makes sense when BOTH have a finite monotonic TTL.
    ttl_errors = []

    # Warm up Numba JIT once.
    warmup_s = generate_scenario(0)
    warmup_snapshot = capture_approval(warmup_s)
    predict(
        snapshot=warmup_snapshot,
        predictive_model=LightweightPredictiveModel(quad_params, pid_params),
        delays=DELAY_GRID,
    )

    # StaleGuard computation time only:
    # excludes snapshot capture and ground-truth evaluation.
    prediction_times = []

    for scenario_id in ids:
        s = generate_scenario(scenario_id)
        snapshot = capture_approval(s)

        # ----------------------------
        # ONLINE / StaleGuard
        # ----------------------------
        t0 = time.perf_counter()
        predicted = predict(
            snapshot,
            predictive_model=LightweightPredictiveModel(quad_params, pid_params),
            delays=DELAY_GRID,
        )
        elapsed = time.perf_counter() - t0
        prediction_times.append(elapsed)

        # ----------------------------
        # OFFLINE ground truth
        # ----------------------------
        actual = ground_truth.classify_scenario(s, DELAY_GRID)

        scenario_has_false_safe = False

        class_total += 1
        if (
                predicted["classification"]
                == actual["classification"]
        ):
            class_correct += 1

        print(f"\nSCENARIO {scenario_id}")
        print_scenario(s)
        print("delay   predicted      actual")

        if (
                predicted["results"] is not None
                and actual["results"] is not None
        ):
            for d, pred, gt in zip(
                    DELAY_GRID,
                    predicted["results"],
                    actual["results"],
            ):
                print(
                    f"{d:>4.1f}s   "
                    f"{pred:<14} "
                    f"{gt}"
                )

                # Binary safety evaluation.
                pred_safe = pred == "SAFE"
                gt_safe = gt == "SAFE"
                total_delay_tests += 1

                if gt_safe:
                    actual_safe += 1
                else:
                    actual_unsafe += 1

                if pred_safe and not gt_safe:
                    false_safe += 1
                    scenario_has_false_safe = True

                if not pred_safe and gt_safe:
                    false_reject += 1

        if scenario_has_false_safe:
            scenarios_with_false_safe += 1

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

        # TTL error: first delay at which safety validity is lost.
        if (
                predicted["first_bad_delay"] is not None
                and actual["first_bad_delay"] is not None
                and predicted["classification"] not in {"INVALID_NOMINAL", "INVALID_AT_APPROVAL"}
                and actual["classification"] not in {"INVALID_NOMINAL", "INVALID_AT_APPROVAL"}
        ):
            ttl_error = (
                    predicted["first_bad_delay"]
                    - actual["first_bad_delay"]
            )
            ttl_errors.append(ttl_error)
            print(
                f"TTL error: "
                f"{ttl_error:+.3f}s"
            )
        print(
            f"StaleGuard prediction time: "
            f"{1000 * elapsed:.1f} ms"
        )

    # ============================================================
    # SUMMARY
    # ============================================================
    false_safe_rate = (
        false_safe / actual_unsafe
        if actual_unsafe > 0
        else 0.0
    )
    false_rejection_rate = (
        false_reject / actual_safe
        if actual_safe > 0
        else 0.0
    )
    scenario_false_safe_rate = (
        scenarios_with_false_safe / class_total
        if class_total > 0
        else 0.0
    )
    classification_accuracy = (
        class_correct / class_total
        if class_total > 0
        else 0.0
    )
    prediction_times = np.asarray(
        prediction_times
    )
    print("\n" + "=" * 70)
    print("EVALUATION SUMMARY")
    print("=" * 70)

    # ============================================================
    # PREDICTION CORRECTNESS
    # ============================================================
    print("\nPrediction correctness")
    print("-" * 30)
    print(
        f"scenarios: "
        f"{len(prediction_times)}"
    )
    print(
        f"delay tests: "
        f"{total_delay_tests}"
    )
    print(
        f"false-safe: "
        f"{false_safe}/{actual_unsafe} "
        f"= {100 * false_safe_rate:.2f}%"
    )
    print(
        f"false-rejection: "
        f"{false_reject}/{actual_safe} "
        f"= {100 * false_rejection_rate:.2f}%"
    )
    print(
        f"scenarios with any false-safe: "
        f"{scenarios_with_false_safe}/{class_total} "
        f"= {100 * scenario_false_safe_rate:.2f}%"
    )
    print(
        f"scenario classification accuracy: "
        f"{class_correct}/{class_total} "
        f"= {100 * classification_accuracy:.2f}%"
    )

    # ============================================================
    # TTL ACCURACY
    # ============================================================

    print("\nTTL accuracy")
    print("-" * 30)

    if ttl_errors:
        ttl_errors = np.asarray(
            ttl_errors
        )
        print(
            f"TTL comparable cases: "
            f"{len(ttl_errors)}"
        )
        print(
            f"TTL MAE: "
            f"{np.mean(np.abs(ttl_errors)):.3f}s"
        )
        print(
            f"TTL mean signed error: "
            f"{np.mean(ttl_errors):+.3f}s"
        )
        print(
            f"TTL max abs error: "
            f"{np.max(np.abs(ttl_errors)):.3f}s"
        )
    else:
        print(
            "TTL comparable cases: 0"
        )

    # ============================================================
    # RUNTIME
    # ============================================================

    print("\nRuntime")
    print("-" * 30)
    print(
        f"StaleGuard compute mean: "
        f"{1000 * prediction_times.mean():.1f} "
        f"ms/scenario"
    )
    print(
        f"StaleGuard compute median: "
        f"{1000 * np.median(prediction_times):.1f} "
        f"ms/scenario"
    )
    print(
        f"StaleGuard compute p95: "
        f"{1000 * np.percentile(prediction_times, 95):.1f} "
        f"ms/scenario"
    )
    print(
        f"StaleGuard compute p99: "
        f"{1000 * np.percentile(prediction_times, 99):.1f} "
        f"ms/scenario"
    )
    print(
        f"StaleGuard compute max: "
        f"{1000 * prediction_times.max():.1f} "
        f"ms/scenario"
    )

if __name__ == "__main__":
    main()
