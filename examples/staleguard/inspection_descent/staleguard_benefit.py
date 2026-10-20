import argparse
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
    SIM_SEED,
    make_forward_trajectory,
    inspection_x,
)
from mission import capture_approval
from safe_control_gym.utils.registration import make
from model.common import STATE_VX, STATE_X, STATE_Y, STATE_Z
from model.lightweight import LightweightPredictiveModel
from model.scg import load_scg_platform_params, make_scg_env_factory
from staleguard import predict


MISSION_DEADLINE_S = 4.2 # Fixed deadline, measured from command receipt.
SCENARIOS = 10 # Run 10 scenarios for each pending delay.
SCAN_DWELL_S = 0.18 # Quadrotor must at least in the "inspection region" at least 0.18 seconds.
CALIBRATION_FILE = Path(__file__).with_name("calibrated_scenarios.json")


def should_recover(result, pending):
    # Should recover if command is unsafe at t_cmd_recv.
    if result["classification"] in {"INVALID_AT_APPROVAL", "INVALID_NOMINAL"}:
        return True
    # Should also recover if command is unsafe before it is applied.
    first_unsafe = result["first_bad_delay"]
    return first_unsafe is not None and first_unsafe <= pending


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
):
    """Execute one policy; mission-retry retains the historical recovery variant."""
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
    physics_tick = 0
    completed = False
    collision = False
    recovery_started = False

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

    # Observe every existing physics step without altering its action/dynamics.
    # Counting N control samples as N*dt overstates elapsed dwell by one dt.
    original_physics = env._physics

    def monitored_physics(rpm):
        nonlocal physics_tick
        position, _ = p.getBasePositionAndOrientation(
            env.DRONE_ID, physicsClientId=env.PYB_CLIENT,
        )
        state = obs.copy()
        state[[STATE_X, STATE_Y, STATE_Z]] = position
        monitor(state, physics_tick)
        if physics_trace is not None:
            physics_trace.append((physics_tick, position))
        original_physics(rpm)
        physics_tick += 1

    env._physics = monitored_physics

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
            if not recover and step == dispatch_step:
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
            monitor(obs, physics_tick)

            if gui:
                time.sleep(1.0 / CTRL_FREQ_HZ)

            # Actual safety.
            if collision:
                break

            if done:
                break

        if physics_trace is not None:
            physics_trace.append((physics_tick, obs[[STATE_X, STATE_Y, STATE_Z]].copy()))

        if gui:
            print(f"completed={completed}, collision={collision}")
            input("Press Enter for next scenario...")

        return (
            bool(completed), # Inspection objective achieved, even if unsafe
            bool(collision), # Entered the unsafe volume
            recovery_started,
            bool(completed and not collision), # Safely complete mission
        )

    finally:
        ctrl.close()
        env.close()


def plan_feasible_command(s):
    """Manually calibrate one scenario using maximum D=0 inspection dwell."""
    # Local import: calibration uses this module's execution and predicates.
    from max_dwell_calibration import plan_max_dwell_command
    return plan_max_dwell_command(s)


def main(count=SCENARIOS, recovery_mode="safety-only"):
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

    planned_scenarios = []
    infeasible_ids = []
    for scenario_id in range(count):
        command = calibrated[str(scenario_id)]["command"]
        if command is None:
            infeasible_ids.append(scenario_id)
        else:
            planned_scenarios.append(Scenario(**command))

    evaluated = len(planned_scenarios)
    print(f"Recovery mode: {recovery_mode}")
    print(f"Calibration feasible: {evaluated}/{count}")
    print(f"Infeasible seeds: {infeasible_ids or 'none'}")
    print("Policy rates use feasible scenarios; calibration failures are excluded.")
    if not evaluated:
        print("No policy executions; all mission and recovery rates are N/A.")
        return

    for queue_pending in DELAY_GRID:
        inspection_completions = 0
        successes = 0
        safe_non_completions = 0
        collisions = 0
        recoveries = 0
        recovery_successes = 0
        recovery_benefits = 0
        recovery_harms = 0
        counterfactual_inspection_completions = 0
        counterfactual_collisions = 0
        counterfactual_successes = 0
        counterfactual_non_completions = 0
        counterfactual_recoveries = 0
        counterfactual_recovery_successes = 0
        always_inspection_completions = 0
        always_collisions = 0
        always_successes = 0
        always_non_completions = 0
        always_recoveries = 0
        always_recovery_successes = 0
        staleguard_only_successes = 0
        always_only_successes = 0

        for s in planned_scenarios:
            snapshot = capture_approval(s) # Quadrotor flies up to approval-time.

            t_cmd_recv = snapshot.approval_time

            # Staleguard starts predicting command TTL after UAV receives approved-command.
            t0 = time.perf_counter()
            predicted = predict(
                snapshot,
                predictive_model=LightweightPredictiveModel(quad_params, pid_params),
                delays=DELAY_GRID
            )
            elapsed = time.perf_counter() - t0 # seconds

            # Decision available after prediction finishes.
            recover_step = round((t_cmd_recv + elapsed) * CTRL_FREQ_HZ)
            # Original scheduled dispatch time.
            dispatch_step = round((t_cmd_recv + queue_pending) * CTRL_FREQ_HZ)
            # Mission deadline.
            deadline_step = round((t_cmd_recv + MISSION_DEADLINE_S) * CTRL_FREQ_HZ)
            # Recovery is possible only before dispatch.
            recover = should_recover(predicted, queue_pending)
            recover = recover and recover_step < dispatch_step

            inspection_completed, collision, recovery_started, success = run_actual(
                s, snapshot,
                dispatch_step,
                recover_step,
                deadline_step,
                recover,
                recovery_mode=recovery_mode,
            )

            counterfactual_inspection, counterfactual_collision, counterfactual_started, counterfactual_success = run_actual(
                s, snapshot,
                dispatch_step,
                recover_step,
                deadline_step,
                False,
                recovery_mode=recovery_mode,
            )

            # Always-Recover has zero decision time and must precede dispatch.
            always_recover_step = round(t_cmd_recv * CTRL_FREQ_HZ)
            always_inspection, always_collision, always_started, always_success = run_actual(
                s, snapshot,
                dispatch_step,
                always_recover_step,
                deadline_step,
                always_recover_step < dispatch_step,
                recovery_mode=recovery_mode,
            )

            assert success == (inspection_completed and not collision)
            assert counterfactual_success == (
                counterfactual_inspection and not counterfactual_collision
            )
            assert always_success == (always_inspection and not always_collision)
            for inspected, unsafe_entry, _, safely_completed in (
                (inspection_completed, collision, recovery_started, success),
                (counterfactual_inspection, counterfactual_collision, counterfactual_started, counterfactual_success),
                (always_inspection, always_collision, always_started, always_success),
            ):
                assert sum((safely_completed, not inspected and not unsafe_entry, unsafe_entry)) == 1

            inspection_completions += int(inspection_completed)
            successes += int(success)
            safe_non_completions += int(not inspection_completed and not collision)
            collisions += int(collision)
            recoveries += int(recovery_started)
            recovery_successes += int(recovery_started and (not collision if safety_only else success))
            recovery_benefits += int(
                recovery_started and success and not counterfactual_success
            )
            recovery_harms += int(
                recovery_started and not success and counterfactual_success
            )
            counterfactual_inspection_completions += int(counterfactual_inspection)
            counterfactual_collisions += int(counterfactual_collision)
            counterfactual_successes += int(counterfactual_success)
            counterfactual_non_completions += int(not counterfactual_inspection and not counterfactual_collision)
            counterfactual_recoveries += int(counterfactual_started)
            counterfactual_recovery_successes += int(counterfactual_started and (not counterfactual_collision if safety_only else counterfactual_success))
            always_inspection_completions += int(always_inspection)
            always_collisions += int(always_collision)
            always_successes += int(always_success)
            always_non_completions += int(not always_inspection and not always_collision)
            always_recoveries += int(always_started)
            always_recovery_successes += int(always_started and (not always_collision if safety_only else always_success))
            staleguard_only_successes += int(success and not always_success)
            always_only_successes += int(always_success and not success)

        net_recovery_benefit_count = recovery_benefits - recovery_harms
        assert (
            net_recovery_benefit_count
            == successes - counterfactual_successes
        )
        assert (
            staleguard_only_successes - always_only_successes
            == successes - always_successes
        )
        assert successes + safe_non_completions + collisions == evaluated
        assert counterfactual_successes + counterfactual_non_completions + counterfactual_collisions == evaluated
        assert always_successes + always_non_completions + always_collisions == evaluated

        print("=" * 78)
        print(f"D = {queue_pending:.1f}s | T = {MISSION_DEADLINE_S:.1f}s | N = {evaluated}")
        print("=" * 78)
        print("\nMISSION OUTCOMES")
        print(f"{'':27}{'No Recovery':>17}{'Always-Recover':>17}{'StaleGuard':>17}")
        print("-" * 78)
        outcomes = (
            ("Safe mission completion", counterfactual_successes, always_successes, successes),
            ("Safe mission non-completion", counterfactual_non_completions, always_non_completions, safe_non_completions),
            ("Unsafe execution", counterfactual_collisions, always_collisions, collisions),
        ) if safety_only else (
            ("Inspection completed", counterfactual_inspection_completions, always_inspection_completions, inspection_completions),
            ("Unsafe entry", counterfactual_collisions, always_collisions, collisions),
            ("Safe mission completion", counterfactual_successes, always_successes, successes),
        )
        for label, no_recovery, always, staleguard in outcomes:
            print(
                f"{label:<27}{format_count(no_recovery, evaluated):>17}"
                f"{format_count(always, evaluated):>17}"
                f"{format_count(staleguard, evaluated):>17}"
            )

        print("\nSAFETY FALLBACK" if safety_only else "\nRECOVERY DECISIONS")
        print("-" * 78)
        print(
            f"{('Fallback triggered' if safety_only else 'Recovery triggered'):<27}{format_count(counterfactual_recoveries, evaluated):>17}"
            f"{format_count(always_recoveries, evaluated):>17}"
            f"{format_count(recoveries, evaluated):>17}"
        )
        counterfactual_recovery_rate = (
            format_count(counterfactual_recovery_successes, counterfactual_recoveries)
            if counterfactual_recoveries else "N/A (0/0)"
        )
        always_recovery_rate = (
            format_count(always_recovery_successes, always_recoveries)
            if always_recoveries else "N/A (0/0)"
        )
        recovery_rate = (
            format_count(recovery_successes, recoveries)
            if recoveries else "N/A (0/0)"
        )
        print(
            f"{('Fallback safe (no entry)' if safety_only else 'Successful recoveries'):<27}{counterfactual_recovery_rate:>17}"
            f"{always_recovery_rate:>17}{recovery_rate:>17}"
        )

        print("\nSTALEGUARD vs NO RECOVERY")
        if safety_only:
            print("Paired safe mission completion")
        print("-" * 78)
        print(
            f"{('Completion benefit' if safety_only else 'Recovery benefit'):<27}{format_count(recovery_benefits, evaluated)}"
        )
        print(
            f"{('Completion harm' if safety_only else 'Recovery harm'):<27}{format_count(recovery_harms, evaluated)}"
        )
        print(
            f"{('Completion difference' if safety_only else 'Net benefit'):<27}"
            f"{100 * (successes - counterfactual_successes) / evaluated:+.1f} percentage points"
        )
        print("\nSTALEGUARD vs ALWAYS-RECOVER (StaleGuard minus Always-Recover)")
        print("-" * 78)
        print(
            f"{'Safe mission completion difference':<36}"
            f"{100 * (successes - always_successes) / evaluated:+.1f} percentage points"
        )
        print(f"{'StaleGuard-only successes':<36}{format_count(staleguard_only_successes, evaluated)}")
        print(f"{'Always-Recover-only successes':<36}{format_count(always_only_successes, evaluated)}")
        print(
            f"{('Fallback frequency difference' if safety_only else 'Recovery frequency difference'):<36}"
            f"{100 * (recoveries - always_recoveries) / evaluated:+.1f} percentage points"
        )
        print("=" * 78)
        print()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="No Recovery / Always-Recover / StaleGuard experiment.")
    parser.add_argument("--count", type=int, default=SCENARIOS, help="Scenario seeds 0 through count-1.")
    parser.add_argument("--recovery", choices=("safety-only", "mission-retry"), default="safety-only",
                        help="Safety fallback (primary), or historical stop-and-retry inspection variant.")
    args = parser.parse_args()
    if args.count < 1:
        parser.error("--count must be positive")
    main(count=args.count, recovery_mode=args.recovery)
