import time
from experiment import (
    generate_scenario,
    apply_descent_command,
    unsafe,
    DELAY_GRID,
    CTRL_FREQ_HZ,
    PYB_FREQ_HZ,
    SIM_SEED,
    make_forward_trajectory,
    inspection_x
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


def run_actual(
    s,
    snapshot,
    dispatch_step,
    recover_step,
    deadline_step,
    recover,
    gui=False,
    debug=False,
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


    scan_x = inspection_x(s)
    recovering = False
    descending = False
    scan_steps = 0
    completed = False
    collision = False
    recovery_started = False

    if gui:
        import pybullet as p

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
        best_scan_steps = 0
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
                    f"scan_time={step / CTRL_FREQ_HZ:.2f}s "
                    f"actual_z={obs[STATE_Z]:.3f}m "
                    f"reference_z={traj[step, STATE_Z]:.3f}m "
                    f"target_z={s.low_height:.3f}m "
                    f"vx={obs[STATE_VX]:.3f}m/s"
                )

            if gui:
                time.sleep(1.0 / CTRL_FREQ_HZ)

            # Actual safety.
            if unsafe(obs, snapshot):
                collision = True
                break

            if abs(obs[STATE_X] - scan_x) < 0.12:
                z_error = abs(obs[STATE_Z] - s.low_height)
                best_z_error_at_scan = min(best_z_error_at_scan, z_error)

            # Actual inspection completion.
            scanning = in_inspection_region(obs, s, scan_x)
            scan_steps = scan_steps + 1 if scanning else 0

            best_scan_steps = max(best_scan_steps, scan_steps)

            if scan_steps >= round(SCAN_DWELL_S * CTRL_FREQ_HZ):
                completed = True

            if done:
                break

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
                f"best_inspection_dwell={best_scan_steps / CTRL_FREQ_HZ:.2f}s, "
                f"best_z_error_at_scan={best_z_error_at_scan:.3f}m"
            )

        if gui:
            print(f"completed={completed}, collision={collision}")
            input("Press Enter for next scenario...")

        return (
            completed and not collision, # Safely complete mission
            collision, # Safe or Unsafe
            recovery_started, # Did recovery start or not
        )

    finally:
        ctrl.close()
        env.close()

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

    for queue_pending in PENDINGS:
        for mission_deadline in DEADLINES:
            successes = 0
            collisions = 0
            recoveries = 0
            recovery_successes = 0
            recovery_benefits = 0
            recovery_harms = 0
            counterfactual_successes = 0

            for scenario_id in range(SCENARIOS):
                s = generate_scenario(scenario_id)
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

                success, collision, recovery_started = run_actual(
                    s, snapshot,
                    dispatch_step,
                    recover_step,
                    deadline_step,
                    recover,
                    # gui=(queue_pending == 0.0 and mission_deadline == 4.2)
                    debug=(queue_pending == 0.0 and mission_deadline == 4.2)
                )

                counterfactual_success, _, _ = run_actual(
                    s, snapshot,
                    dispatch_step,
                    recover_step,
                    deadline_step,
                    False
                )

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
                counterfactual_successes += int(counterfactual_success)

            net_recovery_benefit_count = recovery_benefits - recovery_harms
            assert (
                net_recovery_benefit_count
                == successes - counterfactual_successes
            )

            print(f"D={queue_pending:.1f}s T={mission_deadline:.1f}s N={SCENARIOS}")
            print(
                f"safe_completion={successes / SCENARIOS:.1%} "
                f"(count={successes}/{SCENARIOS})"
            )
            print(
                f"unsafe={collisions / SCENARIOS:.1%} "
                f"(count={collisions}/{SCENARIOS})"
            )
            print(
                f"recovery_trigger={recoveries / SCENARIOS:.1%} "
                f"(count={recoveries}/{SCENARIOS})"
            )
            if recoveries == 0:
                print("recovery_success=N/A (count=0/0)")
            else:
                print(
                    f"recovery_success={recovery_successes / recoveries:.1%} "
                    f"(count={recovery_successes}/{recoveries})"
                )
            print(
                f"recovery_benefit={recovery_benefits / SCENARIOS:.1%} "
                f"(count={recovery_benefits}/{SCENARIOS})"
            )
            print(
                f"recovery_harm={recovery_harms / SCENARIOS:.1%} "
                f"(count={recovery_harms}/{SCENARIOS})"
            )
            print(
                f"net_recovery_benefit="
                f"{100 * net_recovery_benefit_count / SCENARIOS:.1f} "
                f"percentage points "
                f"(count={net_recovery_benefit_count}/{SCENARIOS})"
            )
            print('\n')







if __name__ == "__main__":
    main()

"""
---------------------------------------------------------------------------------------------------------------------------
t_obs_sent --- UPLINK ------ t_obs_recv    t_approved -----DOWN LINK----- t_cmd_recv    t_staleguard             t_cmd_applied

"""
