import argparse
from dataclasses import replace
import json
from pathlib import Path
import time
import pybullet as p
from experiment import (
    Scenario,
    generate_scenario,
    apply_descent_command,
    unsafe,
    DELAY_GRID,
    CTRL_FREQ_HZ,
    PYB_FREQ_HZ,
    MEASURE_SECONDS,
    SIM_SEED,
    make_forward_trajectory,
    inspection_x,
)
from mission import capture_approval
from safe_control_gym.utils.registration import make
from model.common import STATE_VX, STATE_X, STATE_Y, STATE_Z, capture_pid_state
from model.lightweight import LightweightPredictiveModel
from model.scg import load_scg_platform_params, make_scg_env_factory
from physics_monitor import monitor_physics_steps
from staleguard import predict
from use_time_recheck import predict_immediate, first_step_after_check

MISSION_DEADLINE_S = 4.2  # Fixed deadline, measured from command receipt.
SCENARIOS = 10  # Run 10 scenarios for each pending delay.
SCAN_DWELL_S = 0.18  # Quadrotor must at least in the "inspection region" at least 0.18 seconds.
CALIBRATION_FILE = Path(__file__).with_name("calibrated_scenarios.json")


def should_recover(result, pending):
    # Should recover if command is unsafe at t_cmd_recv.
    if result["classification"] in {"INVALID_AT_APPROVAL", "INVALID_NOMINAL"}:
        return True
    # Should also recover if command is unsafe before it is applied.
    first_unsafe = result["first_bad_delay"]
    return first_unsafe is not None and first_unsafe <= pending


def predict_at_reception(snapshot, quad_params, pid_params):
    """Experiment B receives no actual dispatch delay or dispatch timestamp."""
    started = time.perf_counter()
    prediction = predict(snapshot, LightweightPredictiveModel(quad_params, pid_params),
                         delays=DELAY_GRID)
    return prediction, time.perf_counter() - started


def ttl_expired_at_dispatch(prediction, waited_steps):
    """Apply the existing first-unsafe cutoff on the scheduler's control grid."""
    if prediction["classification"] in {"INVALID_AT_APPROVAL", "INVALID_NOMINAL"}:
        return True
    first_unsafe = prediction["first_bad_delay"]
    # Integer steps preserve equality at e.g. 0.6 versus np.arange's 0.600...1.
    return first_unsafe is not None and waited_steps >= round(float(first_unsafe) * CTRL_FREQ_HZ)


def in_inspection_region(state, scenario, scan_x):
    return (
        abs(state[STATE_X] - scan_x) < 0.12
        and abs(state[STATE_Y]) < 0.20
        and abs(state[STATE_Z] - scenario.low_height) < 0.08
    )


def format_count(count, total):
    return f"{count}/{total} ({count / total:.1%})"


def run_actual(
    s,
    snapshot,
    dispatch_step,
    recover_step,
    deadline_step,
    recover,
    gui=False,
    trace=None,
    physics_trace=None,
    recovery_mode="safety-only",
    ttl_prediction=None,
    use_time_check=None,
    recheck_log=None,
):
    """Execute one policy; mission-retry retains the historical recovery variant."""
    if ttl_prediction is not None and recovery_mode != "safety-only":
        raise ValueError("Experiment B requires safety-only recovery")
    if use_time_check is not None and (recovery_mode != "safety-only" or ttl_prediction is not None or recover):
        raise ValueError("Use-Time Recheck requires its own safety-only dispatch decision")
    env_factory = make_scg_env_factory(
        init_state={
            "init_x": 0.0,
            "init_y": 0.0,
            "init_z": s.usual_height,
        },
        pyb_freq=PYB_FREQ_HZ,
    )
    env = env_factory(gui=gui)
    ctrl = make("pid", env_factory)

    obs, info = env.reset(seed=SIM_SEED)
    ctrl.reset_before_run(obs=obs, info=info, env=env)
    traj = make_forward_trajectory(env, s)
    assert deadline_step <= len(traj)

    env.X_GOAL = traj.copy()
    ctrl.reference = traj.copy()

    if trace is not None:
        trace.append((0, obs.copy(), traj[0].copy()))

    scan_x = inspection_x(s)
    recovering = False
    descending = False
    scan_start_tick = None
    completed = False
    collision = False
    recovery_started = False
    command_step = dispatch_step
    check_result = None

    def monitor(state, tick):
        nonlocal scan_start_tick, completed, collision
        collision = collision or unsafe(state, snapshot)
        if in_inspection_region(state, s, scan_x):
            if scan_start_tick is None:
                scan_start_tick = tick
            elapsed_ticks = tick - scan_start_tick
            if elapsed_ticks >= round(SCAN_DWELL_S * PYB_FREQ_HZ):
                completed = True
        else:
            scan_start_tick = None

    def monitor_position(position, tick):
        state = obs.copy()
        state[[STATE_X, STATE_Y, STATE_Z]] = position
        monitor(state, tick)
        if physics_trace is not None:
            physics_trace.append((tick, position))

    sample_physics = monitor_physics_steps(env, monitor_position)

    if gui:
        p.addUserDebugText(
            "SCAN TARGET",
            [scan_x, 0, s.low_height],
            textColorRGB=[0, 1, 0],
            physicsClientId=env.PYB_CLIENT
        )

        p.addUserDebugText(
            "UNSAFE START",
            [s.box_x_min, 0, s.unsafe_below_z],
            textColorRGB=[1, 0, 0],
            physicsClientId=env.PYB_CLIENT
        )

    try:
        for step in range(deadline_step):
            if use_time_check is not None and step == dispatch_step:
                started = time.perf_counter()
                current = replace(
                    snapshot, state=obs.copy(), pid_state=capture_pid_state(ctrl),
                    remaining_reference=traj[step:].copy(), approval_time=step / CTRL_FREQ_HZ,
                    horizon_steps=round(MEASURE_SECONDS * CTRL_FREQ_HZ) - step,
                )
                check_result = use_time_check(current)
                elapsed = time.perf_counter() - started
                command_step = first_step_after_check(dispatch_step, elapsed)
                if recheck_log is not None:
                    recheck_log.update(snapshot=current, prediction=check_result, computation_s=elapsed,
                                       scheduled_dispatch_step=dispatch_step, decision_step=command_step,
                                       decision_applied_step=None, application_step=None, fallback_step=None)

            if use_time_check is not None and step == command_step and check_result is not None:
                # The original snapshot's result is used once, despite state motion.
                recover = check_result != "SAFE"
                recover_step = step
                if recheck_log is not None:
                    recheck_log.update(decision_applied_step=step, state_at_decision=obs.copy(),
                                       application_step=None if recover else step,
                                       fallback_step=step if recover else None)

            # Experiment B: scheduler checks TTL before applying the command.
            # Nothing is cancelled earlier using knowledge of future dispatch.
            if step == dispatch_step and ttl_prediction is not None:
                prediction, computation_s = ttl_prediction
                waited_steps = step - round(snapshot.approval_time * CTRL_FREQ_HZ)
                ready = computation_s <= waited_steps / CTRL_FREQ_HZ
                # No result yet: execute on schedule, without later cancellation.
                recover = ready and ttl_expired_at_dispatch(prediction, waited_steps)
                recover_step = step

            # StaleGuard cancels pending command and starts recovery.
            if recover and step == recover_step:
                recovering = recovery_mode == "mission-retry"
                recovery_started = True
                traj = traj.copy()
                if recovering:
                    # Historical variant: stop and retry the inspection.
                    traj[step:, STATE_X] = scan_x
                    traj[step:, STATE_VX] = 0.0
                # Safety-only fallback preserves nominal forward references.
                traj[step:, STATE_Z] = s.usual_height
                env.X_GOAL = traj
                ctrl.reference = traj

            # Recovery: descend after reaching inspection position.
            if recovering and not descending:
                if (
                    abs(obs[STATE_X] - scan_x) < 0.08
                    and abs(obs[STATE_VX]) < 0.15
                ):
                    descending = True
                    end_step = step + snapshot.low_steps
                    traj[step:end_step, STATE_Z] = s.low_height
                    env.X_GOAL = traj
                    ctrl.reference = traj

            # Otherwise, execute approved command at scheduled time.
            if not recover and step == command_step:
                traj = apply_descent_command(
                    traj, step,
                    snapshot.low_steps,
                    snapshot.low_height
                )
                env.X_GOAL = traj
                ctrl.reference = traj

            # Actual PyBullet execution.
            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)

            if trace is not None:
                # State is at step + 1; reference was applied at step.
                trace.append((step + 1, obs.copy(), traj[step].copy()))
            sample_physics()

            if gui:
                time.sleep(1.0 / CTRL_FREQ_HZ)

            # Actual safety.
            if collision:
                break

            if done:
                break

        if gui:
            print(f"completed={completed}, collision={collision}")
            input("Press Enter for next scenario...")

        return (
            bool(completed),  # Inspection objective achieved, even if unsafe
            bool(collision),  # Entered the unsafe volume
            recovery_started,
            bool(completed and not collision),  # Safely complete mission
        )

    finally:
        ctrl.close()
        env.close()


def plan_feasible_command(s):
    """Manually calibrate one scenario using maximum D=0 inspection dwell."""
    # Local import: calibration uses this module's execution and predicates.
    from max_dwell_calibration import plan_max_dwell_command
    return plan_max_dwell_command(s)


def print_report(metrics, paired, pending, evaluated, safety_only, predictions_not_ready=None):
    """Print one delay's policy metrics and paired safe-completion comparisons."""
    policies = ("No Recovery", "Always-Recover", "StaleGuard")
    if "Use-Time Recheck" in metrics:
        policies += ("Use-Time Recheck",)
    staleguard = metrics["StaleGuard"]
    no_recovery = metrics["No Recovery"]
    always_recover = metrics["Always-Recover"]

    print("=" * 78)
    print(f"D = {pending:.1f}s | T = {MISSION_DEADLINE_S:.1f}s | N = {evaluated}")
    print("=" * 78)
    print("\nMISSION OUTCOMES")
    print(f"{'':27}" + "".join(f"{policy:>17}" for policy in policies))
    print("-" * 78)
    rows = (
        ("Safe mission completion", "successes"),
        ("Safe mission non-completion", "safe_non_completions"),
        ("Unsafe execution", "collisions"),
    ) if safety_only else (
        ("Inspection completed", "inspection_completions"),
        ("Unsafe entry", "collisions"),
        ("Safe mission completion", "successes"),
    )
    for label, key in rows:
        print(f"{label:<27}" + "".join(
            f"{format_count(metrics[policy][key], evaluated):>17}" for policy in policies
        ))

    print("\nSAFETY FALLBACK" if safety_only else "\nRECOVERY DECISIONS")
    print("-" * 78)
    label = "Fallback triggered" if safety_only else "Recovery triggered"
    print(f"{label:<27}" + "".join(
        f"{format_count(metrics[policy]['recoveries'], evaluated):>17}" for policy in policies
    ))
    recovery_rates = []
    for policy in policies:
        counts = metrics[policy]
        rate = (format_count(counts["recovery_successes"], counts["recoveries"])
                if counts["recoveries"] else "N/A (0/0)")
        recovery_rates.append(rate)
    label = "Fallback safe (no entry)" if safety_only else "Successful recoveries"
    print(f"{label:<27}" + "".join(f"{rate:>17}" for rate in recovery_rates))
    if predictions_not_ready is not None:
        print(f"Prediction ready by dispatch: {format_count(evaluated - predictions_not_ready, evaluated)}")
        print(f"Prediction not ready:         {format_count(predictions_not_ready, evaluated)}")

    print("\nSTALEGUARD vs NO RECOVERY")
    if safety_only:
        print("Paired safe mission completion")
    print("-" * 78)
    print(
        f"{('Completion benefit' if safety_only else 'Recovery benefit'):<27}"
        f"{format_count(paired['recovery_benefits'], evaluated)}"
    )
    print(
        f"{('Completion harm' if safety_only else 'Recovery harm'):<27}"
        f"{format_count(paired['recovery_harms'], evaluated)}"
    )
    print(
        f"{('Completion difference' if safety_only else 'Net benefit'):<27}"
        f"{100 * (staleguard['successes'] - no_recovery['successes']) / evaluated:+.1f} percentage points"
    )
    print("\nSTALEGUARD vs ALWAYS-RECOVER (StaleGuard minus Always-Recover)")
    print("-" * 78)
    print(
        f"{'Safe mission completion difference':<36}"
        f"{100 * (staleguard['successes'] - always_recover['successes']) / evaluated:+.1f} percentage points"
    )
    print(f"{'StaleGuard-only successes':<36}{format_count(paired['staleguard_only_successes'], evaluated)}")
    print(f"{'Always-Recover-only successes':<36}{format_count(paired['always_only_successes'], evaluated)}")
    print(
        f"{('Fallback frequency difference' if safety_only else 'Recovery frequency difference'):<36}"
        f"{100 * (staleguard['recoveries'] - always_recover['recoveries']) / evaluated:+.1f} percentage points"
    )
    print("=" * 78)
    print()


def main(count=SCENARIOS, recovery_mode="safety-only", experiment="A", use_time_recheck=False):
    if experiment == "B" and recovery_mode != "safety-only":
        raise ValueError("Experiment B requires safety-only recovery")
    if use_time_recheck and (experiment != "B" or recovery_mode != "safety-only"):
        raise ValueError("Use-Time Recheck is an additional safety-only Experiment B policy")
    safety_only = recovery_mode == "safety-only"
    calibrated = json.loads(CALIBRATION_FILE.read_text())
    quad_params, pid_params = load_scg_platform_params()

    # Warmup: compile Numba JIT function once.
    warmup_s = generate_scenario(0)
    warmup_snapshot = capture_approval(warmup_s)
    predict(
        snapshot=warmup_snapshot,
        predictive_model=LightweightPredictiveModel(quad_params, pid_params),
        delays=DELAY_GRID
    )
    def check_now(current):
        return predict_immediate(current, LightweightPredictiveModel(quad_params, pid_params))
    if use_time_recheck:
        check_now(warmup_snapshot)  # Warm the same CPU model kernels before measuring.

    planned_scenarios = []
    infeasible_ids = []
    for scenario_id in range(count):
        command = calibrated[str(scenario_id)]["command"]
        if command is None:
            infeasible_ids.append(scenario_id)
        else:
            planned_scenarios.append(Scenario(**command))

    evaluated = len(planned_scenarios)
    if experiment == "B":
        print("Experiment B: dispatch delay unknown at reception; TTL checked at dispatch.")
        print("Prediction not ready: execute on schedule; no later cancellation.")
    print(f"Recovery mode: {recovery_mode}")
    print(f"Calibration feasible: {evaluated}/{count}")
    print(f"Infeasible seeds: {infeasible_ids or 'none'}")
    print("Policy rates use feasible scenarios; calibration failures are excluded.")
    if not evaluated:
        print("No policy executions; all mission and recovery rates are N/A.")
        return

    receptions = []
    if experiment == "B":
        # Predict once per scenario before the scheduler enumerates actual D.
        for s in planned_scenarios:
            snapshot = capture_approval(s)
            receptions.append((snapshot, predict_at_reception(snapshot, quad_params, pid_params)))
        durations = [reception[1][1] for reception in receptions]
        print(f"Prediction computation: min={1000 * min(durations):.1f} ms, "
              f"mean={1000 * sum(durations) / len(durations):.1f} ms, "
              f"max={1000 * max(durations):.1f} ms")

    for queue_pending in DELAY_GRID:
        predictions_not_ready = 0
        recheck_events = []
        policies = ("StaleGuard", "No Recovery", "Always-Recover")
        if use_time_recheck:
            policies += ("Use-Time Recheck",)
        metrics = {
            policy: {
                "inspection_completions": 0,
                "successes": 0,
                "safe_non_completions": 0,
                "collisions": 0,
                "recoveries": 0,
                "recovery_successes": 0,
            }
            for policy in policies
        }
        # Paired counts compare outcomes of the same scenario and command.
        paired = {
            "recovery_benefits": 0,
            "recovery_harms": 0,
            "staleguard_only_successes": 0,
            "always_only_successes": 0,
        }

        for scenario_index, s in enumerate(planned_scenarios):
            if experiment == "B":
                snapshot, reception_prediction = receptions[scenario_index]
                predicted, elapsed = reception_prediction
            else:
                snapshot = capture_approval(s)  # Quadrotor flies up to approval-time.

                # Experiment A retains its known-delay prediction/timing behavior.
                t0 = time.perf_counter()
                predicted = predict(
                    snapshot,
                    predictive_model=LightweightPredictiveModel(quad_params, pid_params),
                    delays=DELAY_GRID
                )
                elapsed = time.perf_counter() - t0  # seconds

            t_cmd_recv = snapshot.approval_time

            # Decision available after prediction finishes.
            recover_step = round((t_cmd_recv + elapsed) * CTRL_FREQ_HZ)
            # Original scheduled dispatch time.
            dispatch_step = round((t_cmd_recv + queue_pending) * CTRL_FREQ_HZ)
            # Mission deadline.
            deadline_step = round((t_cmd_recv + MISSION_DEADLINE_S) * CTRL_FREQ_HZ)
            ttl_kwargs = {}
            if experiment == "B":
                # The execution loop will decide at the actual dispatch event.
                recover = False
                recover_step = dispatch_step
                ttl_kwargs = {"ttl_prediction": reception_prediction}
                waited_steps = dispatch_step - round(t_cmd_recv * CTRL_FREQ_HZ)
                predictions_not_ready += int(elapsed > waited_steps / CTRL_FREQ_HZ)
            else:
                # Experiment A: recovery is possible only before dispatch.
                recover = should_recover(predicted, queue_pending)
                recover = recover and recover_step < dispatch_step

            # Run all three policies with the same command, snapshot, and deadline.
            outcomes = {}
            outcomes["StaleGuard"] = run_actual(
                s, snapshot,
                dispatch_step,
                recover_step,
                deadline_step,
                recover,
                recovery_mode=recovery_mode,
                **ttl_kwargs,
            )

            outcomes["No Recovery"] = run_actual(
                s, snapshot,
                dispatch_step,
                recover_step,
                deadline_step,
                False,
                recovery_mode=recovery_mode,
            )

            # Always-Recover has zero decision time and must precede dispatch.
            always_recover_step = round(t_cmd_recv * CTRL_FREQ_HZ)
            outcomes["Always-Recover"] = run_actual(
                s, snapshot,
                dispatch_step,
                always_recover_step,
                deadline_step,
                always_recover_step < dispatch_step,
                recovery_mode=recovery_mode,
            )
            if use_time_recheck:
                event = {}
                outcomes["Use-Time Recheck"] = run_actual(
                    s, snapshot, dispatch_step, dispatch_step, deadline_step, False,
                    recovery_mode=recovery_mode, use_time_check=check_now, recheck_log=event,
                )
                recheck_events.append(event)

            # Validate and collect each policy's outcomes using the same definitions.
            for policy, (inspected, unsafe_entry, started, safely_completed) in outcomes.items():
                assert safely_completed == (inspected and not unsafe_entry)
                assert sum((safely_completed, not inspected and not unsafe_entry, unsafe_entry)) == 1
                counts = metrics[policy]
                counts["inspection_completions"] += int(inspected)
                counts["successes"] += int(safely_completed)
                counts["safe_non_completions"] += int(not inspected and not unsafe_entry)
                counts["collisions"] += int(unsafe_entry)
                counts["recoveries"] += int(started)
                # Safety-only fallback success does not require inspection completion.
                counts["recovery_successes"] += int(
                    started and (not unsafe_entry if safety_only else safely_completed)
                )

            # Compare safe completion against the two paired alternatives.
            _, _, recovery_started, success = outcomes["StaleGuard"]
            counterfactual_success = outcomes["No Recovery"][3]
            always_success = outcomes["Always-Recover"][3]
            paired["recovery_benefits"] += int(
                recovery_started and success and not counterfactual_success
            )
            paired["recovery_harms"] += int(
                recovery_started and not success and counterfactual_success
            )
            paired["staleguard_only_successes"] += int(success and not always_success)
            paired["always_only_successes"] += int(always_success and not success)

        staleguard = metrics["StaleGuard"]
        assert (
            paired["recovery_benefits"] - paired["recovery_harms"]
            == staleguard["successes"] - metrics["No Recovery"]["successes"]
        )
        assert (
            paired["staleguard_only_successes"] - paired["always_only_successes"]
            == staleguard["successes"] - metrics["Always-Recover"]["successes"]
        )
        for counts in metrics.values():
            assert counts["successes"] + counts["safe_non_completions"] + counts["collisions"] == evaluated

        print_report(
            metrics, paired, queue_pending, evaluated, safety_only,
            predictions_not_ready=predictions_not_ready if experiment == "B" else None,
        )
        if use_time_recheck:
            durations = [r["computation_s"] for r in recheck_events]
            delays = [(r["decision_step"] - r["scheduled_dispatch_step"]) / CTRL_FREQ_HZ for r in recheck_events]
            print(f"Use-Time Recheck: check mean/max={1000*sum(durations)/evaluated:.2f}/"
                  f"{1000*max(durations):.2f} ms; decision delay mean/max="
                  f"{1000*sum(delays)/evaluated:.2f}/{1000*max(delays):.2f} ms")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="No Recovery / Always-Recover / StaleGuard experiment.")
    parser.add_argument("--count", type=int, default=SCENARIOS, help="Scenario seeds 0 through count-1.")
    parser.add_argument("--recovery", choices=("safety-only", "mission-retry"), default="safety-only",
                        help="Safety fallback (primary), or historical stop-and-retry inspection variant.")
    parser.add_argument("--experiment", choices=("A", "B"), default="A",
                        help="A: known scheduled delay; B: TTL enforcement with unknown dispatch delay.")
    parser.add_argument("--use-time-recheck", action="store_true",
                        help="Add dispatch-time prediction as a fourth safety-only Experiment B policy.")
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count must be positive")
    if args.experiment == "B" and args.recovery != "safety-only":
        parser.error("Experiment B requires --recovery safety-only")
    if args.use_time_recheck and (args.experiment != "B" or args.recovery != "safety-only"):
        parser.error("--use-time-recheck requires --experiment B --recovery safety-only")
    main(count=args.count, recovery_mode=args.recovery, experiment=args.experiment,
         use_time_recheck=args.use_time_recheck)
