"""PyBullet ground truth using the operational physics-step safety monitor."""

from experiment import (
    CTRL_FREQ_HZ,
    MEASURE_SECONDS,
    PYB_FREQ_HZ,
    SIM_SEED,
    apply_descent_command,
    classify_delay_results,
    make_forward_trajectory,
    unsafe,
)
from model.scg import make_scg_env_factory
from model.common import STATE_X, STATE_Y, STATE_Z
from physics_monitor import monitor_physics_steps
from safe_control_gym.utils.registration import make


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

        result = {"hit_nominal": None, "hit_while_waiting": None, "hit_after_command": None}
        ticks_per_control = PYB_FREQ_HZ // CTRL_FREQ_HZ
        approval_tick = approval_step * ticks_per_control
        dispatch_tick = None if dispatch_step is None else dispatch_step * ticks_per_control

        def monitor_position(position, tick):
            if tick < approval_tick:
                return
            state = obs.copy()
            state[[STATE_X, STATE_Y, STATE_Z]] = position
            if not unsafe(state, s):
                return
            if dispatch_tick is None:
                key = "hit_nominal"
            elif tick <= dispatch_tick:
                key = "hit_while_waiting"
            else:
                key = "hit_after_command"
            if result[key] is None:
                result[key] = tick / PYB_FREQ_HZ

        sample_physics = monitor_physics_steps(env, monitor_position)
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
            sample_physics()

            if done:
                break

        return result

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
