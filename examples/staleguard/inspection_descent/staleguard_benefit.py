import time
from dataclasses import replace
import pybullet as p
from experiment import (
    generate_scenario,
    apply_descent_command,
    unsafe,
    DELAY_GRID,
    CTRL_FREQ_HZ,
    PYB_FREQ_HZ,
    SIM_SEED,
    make_forward_trajectory,
    inspection_x, ACCELERATION_SECONDS
)
from mission import capture_approval
from safe_control_gym.utils.registration import make
from model.common import STATE_VX, STATE_X, STATE_Y, STATE_Z
from model.lightweight import LightweightPredictiveModel
from model.scg import load_scg_platform_params, make_scg_env_factory
from staleguard import predict


PENDINGS = [0.0, 0.5, 1.0] # Pending delay for command received but not yet applied. (from t_cmd_recv)
DEADLINES = [4.2, 5.0] # Deadline for mission (from t_cmd_recv)
SCENARIOS = 10 # Run 10 scenarios for each PENDING x DEADLINE
SCAN_DWELL_S = 0.18


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
    debug=False,
    trace=None,
    physics_trace=None,
):
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
    best_scan_ticks = 0
    physics_tick = 0
    completed = False
    collision = False
    recovery_started = False

    def monitor(state, tick):
        nonlocal scan_start_tick, best_scan_ticks, completed, collision
        collision = collision or unsafe(state, snapshot)
        if in_inspection_region(state, s, scan_x):
            if scan_start_tick is None:
                scan_start_tick = tick
            elapsed_ticks = tick - scan_start_tick
            best_scan_ticks = max(best_scan_ticks, elapsed_ticks)
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
        best_z_error_at_scan = float("inf")
        scan_crossed = False
        first_height_step = None
        min_z = float("inf")

        for step in range(deadline_step):
            # StaleGuard cancels pending command and starts recovery.
            if recover and step == recover_step:
                recovering = True
                recovery_started = True
                traj = traj.copy()
                traj[step:, STATE_X] = scan_x
                traj[step:, STATE_VX] = 0.0
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

            min_z = min(min_z, obs[STATE_Z])

            if debug and first_height_step is None:
                if abs(obs[STATE_Z] - s.low_height) < 0.08:
                    first_height_step = step + 1

                    print(
                        f"first_target_height_time={first_height_step / CTRL_FREQ_HZ:.2f}s "
                        f"x_at_target={obs[STATE_X]:.3f}m "
                        f"scan_x={scan_x:.3f}m "
                        f"x_offset={obs[STATE_X] - scan_x:+.3f}m"
                    )

            if debug and not scan_crossed and obs[STATE_X] >= scan_x:
                scan_crossed = True
                print(
                    f"scan_time={(step + 1) / CTRL_FREQ_HZ:.2f}s "
                    f"actual_z={obs[STATE_Z]:.3f}m "
                    f"reference_z={traj[step, STATE_Z]:.3f}m "
                    f"target_z={s.low_height:.3f}m "
                    f"vx={obs[STATE_VX]:.3f}m/s"
                )

            if gui:
                time.sleep(1.0 / CTRL_FREQ_HZ)

            # Actual safety.
            if collision:
                break

            if abs(obs[STATE_X] - scan_x) < 0.12:
                z_error = abs(obs[STATE_Z] - s.low_height)
                best_z_error_at_scan = min(best_z_error_at_scan, z_error)

            if done:
                break

        if physics_trace is not None:
            physics_trace.append((physics_tick, obs[[STATE_X, STATE_Y, STATE_Z]].copy()))

        if debug:
            print(
                f"min_z={min_z:.3f}m "
                f"target_z={s.low_height:.3f}m "
                f"low_seconds={s.low_seconds:.2f}s"
            )

        if debug and first_height_step is None:
            print("Target altitude never reached")

        if debug:
            print(
                f"collision={collision}, "
                f"completed={completed}, "
                f"best_inspection_dwell={best_scan_ticks / PYB_FREQ_HZ:.3f}s, "
                f"best_z_error_at_scan={best_z_error_at_scan:.3f}m"
            )

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
    """
    Find an original command that safely completes inspection at D=0.

    Offline mission design only.
    No StaleGuard predictions are used.
    """

    # Search resolution: existing DELAY_GRID resolution (0.1 s).
    grid_step = round(
        (DELAY_GRID[1] - DELAY_GRID[0]) * CTRL_FREQ_HZ
    )

    # Approval may occur at the initial state, before the first control action.
    # The former 0.1 s floor excluded physically feasible early descents.
    earliest_approval_step = 0

    original_approval_step = round(
        s.approval_time * CTRL_FREQ_HZ
    )

    original_low_steps = round(
        s.low_seconds * CTRL_FREQ_HZ
    )

    # Expected time at which nominal flight reaches the structure.
    structure_time = (
        s.box_x_min / s.speed
        + ACCELERATION_SECONDS / 2
    )
    structure_step = round(structure_time * CTRL_FREQ_HZ)

    # Prefer commands close to the original timing.
    approval_steps = list(range(
        original_approval_step,
        earliest_approval_step - 1,
        -grid_step
    ))
    if earliest_approval_step not in approval_steps:
        approval_steps.append(earliest_approval_step)

    for approval_step in approval_steps:
        # Do not plan a low-altitude command extending beyond
        # nominal arrival at the structure.
        max_low_steps = structure_step - approval_step

        for low_steps in range(
            original_low_steps,
            max_low_steps + 1,
            grid_step
        ):
            candidate = replace(
                s,
                approval_time=approval_step / CTRL_FREQ_HZ,
                low_seconds=low_steps / CTRL_FREQ_HZ
            )

            snapshot = capture_approval(candidate)

            deadline_step = round(
                (candidate.approval_time + min(DEADLINES))
                * CTRL_FREQ_HZ
            )

            _, _, _, success = run_actual(
                candidate,
                snapshot,
                dispatch_step=approval_step,
                recover_step=0,
                deadline_step=deadline_step,
                recover=False,
                gui=False,
            )

            if success:
                return candidate

    return None

def main():
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
    for scenario_id in range(SCENARIOS):
        original = generate_scenario(scenario_id)
        planned = plan_feasible_command(original)
        if planned is None:
            raise RuntimeError(
                f"Scenario {scenario_id}: "
                "No feasible original command found."
            )

        planned_scenarios.append(planned)
        # print(
        #     f"Scenario={scenario_id} "
        #     f"approval={planned.approval_time:.2f}s "
        #     f"low_seconds={planned.low_seconds:.2f}s"
        # )


    for queue_pending in PENDINGS:
        for mission_deadline in DEADLINES:
            inspection_completions = 0
            successes = 0
            collisions = 0
            recoveries = 0
            recovery_successes = 0
            recovery_benefits = 0
            recovery_harms = 0
            counterfactual_inspection_completions = 0
            counterfactual_collisions = 0
            counterfactual_successes = 0

            for scenario_id in range(SCENARIOS):
                s = planned_scenarios[scenario_id]
                snapshot = capture_approval(s) # Quadrotor flies up to approval-time.

                # Assume: t_cmd_recv = t_approved
                t_cmd_approved = snapshot.approval_time
                t_cmd_recv = t_cmd_approved

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
                deadline_step = round((t_cmd_recv + mission_deadline) * CTRL_FREQ_HZ)
                # Recovery is possible only before dispatch.
                recover = should_recover(predicted, queue_pending)
                recover = recover and recover_step < dispatch_step

                inspection_completed, collision, recovery_started, success = run_actual(
                    s, snapshot,
                    dispatch_step,
                    recover_step,
                    deadline_step,
                    recover,
                    # gui=(queue_pending == 0.0 and mission_deadline == 4.2)
                    # debug=(queue_pending == 0.0 and mission_deadline == 4.2)
                )

                counterfactual_inspection, counterfactual_collision, _, counterfactual_success = run_actual(
                    s, snapshot,
                    dispatch_step,
                    recover_step,
                    deadline_step,
                    False
                )

                assert success == (inspection_completed and not collision)
                assert counterfactual_success == (
                    counterfactual_inspection and not counterfactual_collision
                )

                inspection_completions += int(inspection_completed)
                successes += int(success)
                collisions += int(collision)
                recoveries += int(recovery_started)
                recovery_successes += int(recovery_started and success)
                recovery_benefits += int(
                    recovery_started and success and not counterfactual_success
                )
                recovery_harms += int(
                    recovery_started and not success and counterfactual_success
                )
                counterfactual_inspection_completions += int(counterfactual_inspection)
                counterfactual_collisions += int(counterfactual_collision)
                counterfactual_successes += int(counterfactual_success)

            net_recovery_benefit_count = recovery_benefits - recovery_harms
            assert (
                net_recovery_benefit_count
                == successes - counterfactual_successes
            )

            print("=" * 58)
            print(f"D = {queue_pending:.1f}s | T = {mission_deadline:.1f}s | N = {SCENARIOS}")
            print("=" * 58)
            print("\nMISSION OUTCOMES")
            print(f"{'':27}{'No Recovery':>15}{'StaleGuard':>15}")
            print("-" * 58)
            for label, no_recovery, staleguard in (
                ("Inspection completed", counterfactual_inspection_completions, inspection_completions),
                ("Unsafe entry", counterfactual_collisions, collisions),
                ("Safe mission completion", counterfactual_successes, successes),
            ):
                print(
                    f"{label:<27}{format_count(no_recovery, SCENARIOS):>15}"
                    f"{format_count(staleguard, SCENARIOS):>15}"
                )

            print("\nSTALEGUARD DECISIONS")
            print("-" * 58)
            print(f"{'Recovery triggered':<42}{format_count(recoveries, SCENARIOS):>15}")
            if recoveries == 0:
                print("Successful recoveries: N/A (0/0)")
            else:
                print(
                    f"{'Successful recoveries':<42}"
                    f"{format_count(recovery_successes, recoveries):>15}"
                )

            print("\nPAIRED COMPARISON")
            print("-" * 58)
            print(
                f"{'Recovery benefit':<27}{format_count(recovery_benefits, SCENARIOS)}"
            )
            print(
                f"{'Recovery harm':<27}{format_count(recovery_harms, SCENARIOS)}"
            )
            print(
                f"{'Net benefit':<27}"
                f"{100 * (successes - counterfactual_successes) / SCENARIOS:+.1f} percentage points"
            )
            print("=" * 58)
            print()


if __name__ == "__main__":
    main()