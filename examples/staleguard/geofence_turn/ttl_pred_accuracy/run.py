"""Compare StaleGuard predictions against actual No-Recovery PyBullet safety."""

from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import geofence_turn  # Set CPU/thread limits before numerical imports.

import argparse
from dataclasses import asdict
import fcntl
import json
import traceback
import numpy as np

from model.scg import load_scg_platform_params
from geofence_turn.scenario import Config, HZ
from geofence_turn.simulation import execute, check_memory
from geofence_turn.storage import (
    ROOT, load_development, load_ground_truth,
    validate_flight, check_approval_snapshot,
    file_hash, write_json,
)
from geofence_turn.baselines.staleguard import (
    predict_delays, staleguard_status, LightweightPredictiveModel,
)
from geofence_turn.ttl_pred_accuracy.metrics import summarize


# Same dispatch-delay grid for StaleGuard and PyBullet.
DELAYS = (
    0., .02, .04, .06, .08, .10, .12, .14, .16, .18, .20,
    .30, .40, .50, .60, .70, .80, .90, 1.,
)


def first_bad(rows):
    return next((d for d, safe in rows if not safe), None)


def safety_class(first):
    if first is None:
        return "VALID_THROUGH_HORIZON"
    return "INVALID_AT_APPROVAL" if first == 0 else "FINITE_TTL"


def nonmonotonic(rows):
    seen_bad = False
    for _, safe in rows:
        if seen_bad and safe:
            return True
        seen_bad |= not safe
    return False


def compare(s, actual, predicted, forecast):
    """Compare direct prediction and operational TTL decision separately."""
    delay = actual["delay_s"]

    # Ignore computation readiness for this accuracy experiment.
    ready, expired = staleguard_status(
        dict(forecast, computation_s=0.),
        round(delay * HZ),
    )
    assert ready

    first = actual["first_violation_s"]

    return dict(
        seed=s.seed,
        delay_s=delay,
        safe=actual["safe"],
        completed=actual["completed"],
        safe_completion=actual["safe_completion"],
        min_clearance_m=actual["min_clearance_m"],
        first_violation_s=None if first is None else first - s.approval_s,
        first_violation_flight_s=first,
        approval_s=s.approval_s,
        completion_s=actual["completion_s"],
        predicted_safe_raw=predicted["safe"],
        predicted_safe_ttl=not expired,
        prediction_grid_point=True,
        predicted_min_clearance_m=predicted["min_clearance_m"],
        predicted_first_unsafe_delay_s=(
            None if forecast["expiry_steps"] is None
            else forecast["expiry_steps"] / HZ
        ),
        prediction_computation_s=forecast["computation_s"],
    )


def run_accuracy(calibration, out, ground_truth, log):
    config = Config()

    calibrated_scenarios = load_development(calibration, config)

    if not calibrated_scenarios:
        raise ValueError("No calibrated scenarios")

    check_memory(config)

    # Load existing PyBullet flights when scientifically compatible.
    try:
        saved_ground_truth = load_ground_truth(
            calibration, calibrated_scenarios, config, ground_truth
        )
    except ValueError as error:
        if ground_truth:
            raise
        log(f"Ground truth incompatible: {error}")
        log("Running fresh No-Recovery PyBullet flights.")
        saved_ground_truth = {}

    platform = load_scg_platform_params()

    # Warm up numerical model (not an additional approval forecast).
    model = LightweightPredictiveModel(*platform)
    try:
        controller, obs, info, _ = model.start(calibrated_scenarios[0][1])
        model.step(controller.select_action(obs, info), obs, info)
    finally:
        model.close({})

    runtimes = []
    ttl_errors = []
    forecasts = []

    false_safe_scenarios = 0
    class_correct = 0
    reused = 0
    simulated = 0

    executions = out / "executions.jsonl"

    with executions.open("w") as journal:

        for scenario, snapshot, _ in calibrated_scenarios:

            check_memory(config)

            # -------------------------------------------
            # 1. ONLINE: StaleGuard predicts ONCE
            # -------------------------------------------
            forecast = predict_delays(
                snapshot,
                platform,
                delays=DELAYS,
            )

            points = {
                round(row["delay_s"] * HZ): row
                for row in forecast["results"]
            }

            steps = [round(d * HZ) for d in DELAYS]

            if (
                [round(d * HZ) for d in forecast["grid_s"]] != steps
                or len(points) != len(DELAYS)
                or set(points) != set(steps)
            ):
                raise ValueError("StaleGuard prediction grid mismatch")

            runtimes.append(forecast["computation_s"])

            forecasts.append(dict(
                seed=scenario.seed,
                computation_s=forecast["computation_s"],
                expiry_steps=forecast["expiry_steps"],
            ))

            predicted_safety = []
            actual_safety = []
            has_false_safe = False

            # -------------------------------------------
            # 2. SCENARIO LOG
            # -------------------------------------------
            log(f"\nSCENARIO {scenario.seed}")

            log(
                f"speed={scenario.speed:.2f} m/s, "
                f"height={scenario.height:.2f} m, "
                f"fence x={scenario.fence_x:.2f} m"
            )

            log(
                f"approval={scenario.approval_s:.2f}s, "
                f"turn radius={scenario.command.radius:.2f}m, "
                f"deadline={scenario.deadline_s:.2f}s"
            )

            log(
                f"checkpoint: "
                f"{scenario.command.checkpoint_lower} → "
                f"{scenario.command.checkpoint_upper}"
            )

            log("delay   predicted      actual")

            for delay in DELAYS:

                # ---------------------------------------
                # 3. OFFLINE: PyBullet ground truth
                # ---------------------------------------
                actual = saved_ground_truth.get((scenario.seed, delay))

                if actual is None:
                    actual, replayed, _ = execute(
                        scenario,
                        delay,
                        policy="no_recovery",
                        config=config,
                    )

                    check_approval_snapshot(replayed, snapshot)
                    validate_flight(actual, scenario, config)

                    simulated += 1
                else:
                    reused += 1

                predicted = points[round(delay * HZ)]

                predicted_safety.append((delay, predicted["safe"]))
                actual_safety.append((delay, actual["safe"]))

                if predicted["safe"] and not actual["safe"]:
                    has_false_safe = True

                pred_text = "SAFE" if predicted["safe"] else "UNSAFE"
                actual_text = "SAFE" if actual["safe"] else "UNSAFE"

                log(
                    f"{delay:>4.2f}s   "
                    f"{pred_text:<14} "
                    f"{actual_text}"
                )

                # ---------------------------------------
                # 4. SAVE COMPARISON
                # ---------------------------------------
                result = compare(
                    scenario, actual, predicted, forecast
                )

                journal.write(json.dumps(result, allow_nan=False) + "\n")

            # -------------------------------------------
            # 5. SCENARIO METRICS
            # -------------------------------------------
            first_pred = first_bad(predicted_safety)
            first_actual = first_bad(actual_safety)

            pred_class = safety_class(first_pred)
            actual_class = safety_class(first_actual)

            class_correct += pred_class == actual_class
            false_safe_scenarios += has_false_safe

            last_pred = next(
                (d for d, safe in reversed(predicted_safety) if safe),
                None,
            )
            last_actual = next(
                (d for d, safe in reversed(actual_safety) if safe),
                None,
            )

            log(f"class: {pred_class} / {actual_class}")
            log(f"first bad: {first_pred} / {first_actual}")
            log(f"last safe: {last_pred} / {last_actual}")

            if (
                first_pred is not None
                and first_actual is not None
                and not nonmonotonic(predicted_safety)
                and not nonmonotonic(actual_safety)
            ):
                error = first_pred - first_actual
                ttl_errors.append(error)
                log(f"TTL error: {error:+.3f}s")

            log(
                f"StaleGuard prediction time: "
                f"{forecast['computation_s'] * 1000:.1f} ms"
            )

    # -----------------------------------------------
    # 6. FINAL METRICS
    # -----------------------------------------------
    source = Path(calibration)

    if source.is_dir():
        source /= "calibrated_scenarios.json"

    protocol = dict(
        seeds=[s.seed for s, _, _ in calibrated_scenarios],
        delays_s=list(DELAYS),
        prediction_grid_s=list(DELAYS),
        control_hz=HZ,
        config=asdict(config),
        dataset_kind=(
            source.parent.parent.name
            if source.parent.parent.name in ("development", "evaluation")
            else "calibrated"
        ),
    )

    metrics = summarize(executions, protocol)

    if metrics["missing_comparisons"]:
        raise ValueError("Incomplete accuracy comparisons")

    write_json(out / "metrics.json", metrics)

    write_json(
        out / "run.json",
        dict(
            variant="fine_grid_v2",
            input_dataset=dict(
                path=str(source.resolve()),
                sha256=file_hash(source),
            ),
            protocol=protocol,
            forecasts=forecasts,
            ground_truth=dict(
                reused_flights=reused,
                new_pybullet_flights=simulated,
            ),
        ),
    )

    # -----------------------------------------------
    # 7. TERMINAL SUMMARY
    # -----------------------------------------------
    def rate(n, total):
        return f"{100 * n / total:.2f}%" if total else "N/A"

    def print_accuracy(title, result):
        unsafe = result["false_safe"] + result["correct_unsafe"]
        safe = result["false_rejection"] + result["correct_safe"]

        log(f"\n{title}")
        log("-" * 30)
        log(f"delay tests: {result['total_tests']}")
        log(
            f"false-safe: {result['false_safe']}/{unsafe} "
            f"= {rate(result['false_safe'], unsafe)}"
        )
        log(
            f"false-rejection: {result['false_rejection']}/{safe} "
            f"= {rate(result['false_rejection'], safe)}"
        )

    log("\n" + "=" * 70)
    log("EVALUATION SUMMARY")
    log("=" * 70)

    log(f"scenarios: {len(calibrated_scenarios)}")

    print_accuracy(
        "Prediction correctness",
        metrics["raw_prediction"],
    )

    log(
        f"scenarios with any false-safe: "
        f"{false_safe_scenarios}/{len(calibrated_scenarios)} "
        f"= {rate(false_safe_scenarios, len(calibrated_scenarios))}"
    )

    log(
        f"scenario classification accuracy: "
        f"{class_correct}/{len(calibrated_scenarios)} "
        f"= {rate(class_correct, len(calibrated_scenarios))}"
    )

    print_accuracy(
        "TTL dispatch decisions (assuming prediction ready)",
        metrics["ttl_dispatch_decision"],
    )

    log("\nTTL accuracy")
    log("-" * 30)
    log(f"TTL comparable cases: {len(ttl_errors)}")

    if ttl_errors:
        errors = np.asarray(ttl_errors)

        log(f"TTL MAE: {np.abs(errors).mean():.3f}s")
        log(f"TTL mean signed error: {errors.mean():+.3f}s")
        log(f"TTL max abs error: {np.abs(errors).max():.3f}s")

    times = 1000 * np.asarray(runtimes)

    log("\nRuntime")
    log("-" * 30)

    for name, value in (
        ("mean", times.mean()),
        ("median", np.median(times)),
        ("p95", np.percentile(times, 95)),
        ("p99", np.percentile(times, 99)),
        ("max", times.max()),
    ):
        log(f"StaleGuard compute {name}: {value:.1f} ms/scenario")

    log(f"\nPyBullet ground truth: {reused} reused, {simulated} new flights")
    log("Accuracy applies to tested delays, not unsampled dispatch times.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument(
        "--calibration", "--development",
        dest="calibration",
        type=Path,
        default=ROOT / "results/development/calibration",
    )
    parser.add_argument(
        "--ground-truth",
        type=Path,
        action="append",
        default=[],
    )
    parser.add_argument("--out", type=Path)

    args = parser.parse_args()

    folder = (
        args.calibration
        if args.calibration.is_dir()
        else args.calibration.parent
    )

    out = args.out or folder.parent / "ttl_pred_accuracy"
    out.mkdir(parents=True, exist_ok=True)

    # One simple logger: print now, save complete text on termination.
    lines = []

    def log(message):
        print(message, flush=True)
        lines.append(message)

    try:
        with (ROOT / "support/logs/.simulation.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)

            run_accuracy(
                args.calibration,
                out,
                args.ground_truth,
                log,
            )

    except BaseException:
        log(traceback.format_exc().rstrip())
        raise

    finally:
        (out / "accuracy.log").write_text("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()