import time
from experiment import generate_scenario, apply_descent_command, unsafe, DELAY_GRID, CTRL_FREQ_HZ, PYB_FREQ_HZ, \
    SIM_SEED, make_forward_trajectory
from mission import capture_approval
from safe_control_gym.utils.registration import make
from model.lightweight import LightweightPredictiveModel
from model.scg import load_scg_platform_params, make_scg_env_factory
from staleguard import predict


PENDINGS = [0.0, 0.5, 1.0] # Pending delay for command received but not yet applied. (from t_cmd_recv)
DEADLINES = [4.2, 5.0] # Deadline for mission (from t_cmd_recv)
SCENARIOS = 10 # Run 10 scenarios for each PENDING x DEADLINE


def should_recover(result, pending):
    # Should recover if command is unsafe at t_cmd_recv.
    if result["classification"] in {"INVALID_AT_APPROVAL", "INVALID_AT_NOMINAL"}:
        return True
    # Should also recover if command is unsafe before it is applied.
    first_unsafe = result["first_bad_delay"]
    return first_unsafe is not None and first_unsafe <= pending


SCAN_DWELL_S = 0.18
def run_actual(
    s,
    snapshot,
    dispatch_step,
    recover_step,
    deadline_step,
    recover
):
    env_factory = make_scg_env_factory(
        init_state={
            "init_x": 0.0,
            "init_y": 0.0,
            "init_z": s.usual_height,
        },
        pyb_freq=PYB_FREQ_HZ,
    )
    env = env_factory()
    ctrl = make("pid", env_factory)

    obs, info = env.reset(seed=SIM_SEED)
    ctrl.reset_before_run(obs=obs, info=info, env=env)
    traj = make_forward_trajectory(env, s)

    env.X_GOAL = traj.copy()
    ctrl.reference = traj.copy()


    scan_x = s.box_x_min - 0.25
    recovering = False
    descending = False
    scan_steps = 0
    completed = False
    collision = False

    try:
        for step in range(deadline_step):

            # StaleGuard cancels pending command and starts recovery.
            if recover and step == recover_step:
                recovering = True
                traj = traj.copy()
                traj[step:, 0] = scan_x
                traj[step:, 1] = 0.0
                traj[step:, 4] = s.usual_height
                env.X_GOAL = traj
                ctrl.reference = traj

            # Recovery: descend after reaching inspection position.
            if recovering and not descending:
                if abs(obs[0] - scan_x) < 0.08 and abs(obs[1]) < 0.15:
                    descending = True
                    traj[step:, 4] = s.low_height
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

            # Actual safety.
            if unsafe(obs, snapshot):
                collision = True
                break

            # Actual inspection completion.
            scanning = (
                abs(obs[0] - scan_x) < 0.12
                and abs(obs[1]) < 0.20
                and abs(obs[4] - s.low_height) < 0.08
            )

            scan_steps = scan_steps + 1 if scanning else 0

            if scan_steps >= round(SCAN_DWELL_S * CTRL_FREQ_HZ):
                completed = True

            if done:
                break

        return completed and not collision, collision

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

                success, collision = run_actual(
                    s, snapshot,
                    dispatch_step,
                    recover_step,
                    deadline_step,
                    recover
                )
                successes += int(success)
                collisions += int(collision)
                recoveries += int(recover)

            print(
                f"D={queue_pending:.1f}s T={mission_deadline:.1f}s "
                f"safe_completion={successes / SCENARIOS:.1%} "
                f"unsafe={collisions / SCENARIOS:.1%} "
                f"recovery={recoveries / SCENARIOS:.1%}"
            )







if __name__ == "__main__":
    main()

"""
---------------------------------------------------------------------------------------------------------------------------
t_obs_sent --- UPLINK ------ t_obs_recv    t_approved -----DOWN LINK----- t_cmd_recv    t_staleguard             t_cmd_applied

"""