import numpy as np
from classification import classify_delay_results
from config import CTRL_FREQ_HZ, MEASURE_SECONDS, PYB_FREQ_HZ, SIM_SEED
from model.scg import make_scg_env_factory
from safe_control_gym.utils.registration import make
from scenario import unsafe, make_forward_trajectory, apply_descent_command


def first_unsafe_time(states, first_step, last_step, s):
    for step in range(first_step, last_step):
        if unsafe(states[step], s):
            return step / CTRL_FREQ_HZ
    return None


def run_one(s, delay):
    """
    delay=None:
        Never execute the dip command.
        Used to verify that the nominal trajectory is safe.

    delay=float:
        Execute the approved command after that delay.
    """
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
        ctrl = make("pid", env_factory)
        obs, info = env.reset(seed=SIM_SEED)
        ctrl.reset_before_run(obs=obs, info=info, env=env)

        traj = make_forward_trajectory(env, s)
        env.X_GOAL = traj.copy()
        ctrl.reference = traj.copy()

        total_steps = round(MEASURE_SECONDS * CTRL_FREQ_HZ) # total ctrl step of experiment
        approval_step = round(s.approval_time * CTRL_FREQ_HZ) # ctrl step at which command is approved.
        
        # ctrl step at which command is actually dispatched to Quadrotor
        if delay is None:
            dispatch_step = None
        else:
            dispatch_step = (
                approval_step
                + round(delay * CTRL_FREQ_HZ)
            )

        states = [obs.copy()] # Store historical states (observation) from start to end.
        for step in range(total_steps):
            # Apply new trajectory to Quadrotor (note: PID takes time to drive Quadrotor to the desired trajectory)
            if (
                dispatch_step is not None
                and step == dispatch_step
            ):
                changed_traj = apply_descent_command(
                    reference=traj,
                    start_step=step,
                    low_steps=round(s.low_seconds * CTRL_FREQ_HZ), # number of ctrl steps for low-height inspection descent.
                    low_height=s.low_height
                )
                env.X_GOAL = changed_traj
                ctrl.reference = changed_traj

            # Continue the old trajectory 
            motor_action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(motor_action)
            states.append(obs.copy())

            if done:
                break

        states = np.asarray(states)

        # No command:
        # check whether nominal flight ever becomes unsafe.
        if dispatch_step is None:
            hit_nominal = first_unsafe_time(
                states,
                approval_step,
                len(states),
                s,
            )

            return {
                "hit_nominal": hit_nominal,
                "hit_while_waiting": None,
                "hit_after_command": None,
            }

        # Delayed command:
        # first check the waiting period.
        hit_while_waiting = first_unsafe_time(
            states,
            approval_step,
            dispatch_step + 1,
            s,
        )

        # Then check everything after dispatch.
        hit_after_command = first_unsafe_time(
            states,
            dispatch_step + 1,
            len(states),
            s,
        )

        return {
            "hit_nominal": None,
            "hit_while_waiting": hit_while_waiting,
            "hit_after_command": hit_after_command,
        }

    finally:
        if ctrl is not None:
            ctrl.close()
        env.close()


def classify_scenario(s, delays):
    # 1. Check nominal trajectory.
    nominal = run_one(s, None)
    if nominal["hit_nominal"] is not None:
        return classify_delay_results(
            delays,
            results=None,
            nominal_safe=False,
        )
    # 2. Simulate each dispatch delay.
    results = []
    for delay in delays:
        result = run_one(s, delay)
        if result["hit_while_waiting"] is not None:
            status = "WAIT_UNSAFE"
        elif result["hit_after_command"] is not None:
            status = "UNSAFE"
        else:
            status = "SAFE"
        results.append(status)

    # 3. Shared classification.
    return classify_delay_results(delays, results)