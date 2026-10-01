from dataclasses import dataclass
from functools import partial
import numpy as np
from safe_control_gym.utils.registration import get_config, make


FREQ = 50
EPISODE_SECONDS = 6
MEASURE_SECONDS = 5.5
DELAYS = np.arange(0.0, 1.51, 0.1)


@dataclass
class Scenario:
    # Nominal flight
    speed: float
    usual_height: float

    # Structure
    box_x_min: float
    box_x_max: float
    box_y_limit: float
    unsafe_below_z: float

    # Approved command
    approval_time: float
    low_height: float
    low_seconds: float


def generate_scenario(seed):
    rng = np.random.default_rng(seed)

    speed = rng.uniform(0.4, 0.8)
    usual_height = rng.uniform(1.15, 1.40)

    structure_start = rng.uniform(0.8, 1.5)
    structure_length = rng.uniform(0.3, 0.6)

    unsafe_below_z = rng.uniform(0.90, 1.05) # Drone will be unsafe under this z altitude.

    # Nominal requested height must be above the unsafe threshold.
    if usual_height <= unsafe_below_z + 0.10:
        usual_height = unsafe_below_z + 0.10

    # The dip command goes below the unsafe threshold,
    # so timing can potentially matter.
    low_height = rng.uniform(0.60, unsafe_below_z - 0.10)

    return Scenario(
        speed=speed,
        usual_height=usual_height,

        box_x_min=structure_start,
        box_x_max=structure_start + structure_length,
        box_y_limit=0.35,
        unsafe_below_z=unsafe_below_z,

        approval_time=rng.uniform(0.1, 0.5),
        low_height=low_height,
        low_seconds=rng.uniform(0.4, 1.0),
    )


def make_forward_trajectory(env, s):
    n = env.X_GOAL.shape[0]
    dt = 1.0 / env.CTRL_FREQ
    times = np.arange(n) * dt
    
    # Gentle acceleration during first 0.4 s.
    speeds = s.speed * np.minimum(times / 0.4, 1.0)

    traj = np.zeros_like(env.X_GOAL)
    traj[:, 0] = np.cumsum(speeds) * dt # x
    traj[:, 1] = speeds # v_x
    traj[:, 4] = s.usual_height # z

    return traj


def unsafe(state, s):
    x = state[0]
    y = state[2]
    z = state[4]
    
    return (
        s.box_x_min <= x <= s.box_x_max
        and abs(y) <= s.box_y_limit
        and z <= s.unsafe_below_z
    )


def first_unsafe_time(states, first_step, last_step, s):
    for step in range(first_step, last_step):
        if unsafe(states[step], s):
            return step / FREQ

    return None


def run_one(s, delay):
    """
    delay=None:
        Never execute the dip command.
        Used to verify that the nominal trajectory is safe.

    delay=float:
        Execute the approved command after that delay.
    """

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

    env_generator = partial(make, "quadrotor", **config)
    env = env_generator()
    ctrl = None

    try:
        ctrl = make("pid", env_generator)
        obs, info = env.reset(seed=42)
        ctrl.reset_before_run(obs=obs, info=info, env=env)

        traj = make_forward_trajectory(env, s)
        env.X_GOAL = traj.copy()
        ctrl.reference = traj.copy()

        
        total_steps = round(MEASURE_SECONDS * FREQ) # total ctrl step of experiment
        approval_step = round(s.approval_time * FREQ) # ctrl step at which command is approved.
        
        # ctrl step at which command is actually dispatched to Quadrotor
        if delay is None:
            dispatch_step = None
        else:
            dispatch_step = (
                approval_step
                + round(delay * FREQ)
            )
        low_steps = round(s.low_seconds * FREQ) # number of ctrl steps for low-height inspection descent.
        
        states = [obs.copy()] # Store historical states (observation) from start to end.
        for step in range(total_steps):
            # Apply new trajectory to Quadrotor (note: PID takes time to drive Quadrotor to the desired trajectory)
            if (
                dispatch_step is not None
                and step == dispatch_step
            ):
                changed_traj = traj.copy()
                changed_traj[
                    step:step + low_steps,
                    4
                ] = s.low_height
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


def classify_scenario(s):
    # --------------------------------------------------
    # 1. Original trajectory must be safe.
    # --------------------------------------------------

    nominal = run_one(s, None)

    if nominal["hit_nominal"] is not None:
        return {
            "classification": "INVALID_NOMINAL",
            "results": None,
            "first_bad_delay": None,
            "last_safe_delay": None,
        }

    # --------------------------------------------------
    # 2. Test all command delays.
    # --------------------------------------------------

    results = []

    for delay in DELAYS:
        result = run_one(s, delay)
        # This should normally already be prevented by
        # the safe nominal trajectory, but check explicitly.
        if result["hit_while_waiting"] is not None:
            status = "WAIT_UNSAFE"
        elif result["hit_after_command"] is not None:
            status = "UNSAFE"
        else:
            status = "SAFE"
        results.append(status)

    # --------------------------------------------------
    # 3. Immediate execution must be safe.
    # --------------------------------------------------

    if results[0] != "SAFE":
        return {
            "classification": "INVALID_AT_APPROVAL",
            "results": results,
            "first_bad_delay": 0.0,
            "last_safe_delay": None,
        }

    # --------------------------------------------------
    # 4. Always-safe negative control.
    # --------------------------------------------------

    if all(status == "SAFE" for status in results):
        return {
            "classification": "ALWAYS_SAFE",
            "results": results,
            "first_bad_delay": None,
            "last_safe_delay": DELAYS[-1],
        }

    # --------------------------------------------------
    # 5. Find first unsafe delay.
    # --------------------------------------------------

    first_bad_index = next(
        i
        for i, status in enumerate(results)
        if status != "SAFE"
    )

    first_bad_delay = DELAYS[first_bad_index]

    last_safe_delay = (
        DELAYS[first_bad_index - 1]
        if first_bad_index > 0
        else None
    )

    # --------------------------------------------------
    # 6. Check monotonicity.
    #
    # FINITE_TTL:
    #
    # S S S U U U U
    #
    # NON_MONOTONIC:
    #
    # S S U U S S
    # --------------------------------------------------

    after_first_bad = results[first_bad_index:]
    monotonic = all(
        status != "SAFE"
        for status in after_first_bad
    )
    if monotonic:
        classification = "FINITE_TTL"
    else:
        classification = "NON_MONOTONIC"

    return {
        "classification": classification,
        "results": results,
        "first_bad_delay": first_bad_delay,
        "last_safe_delay": last_safe_delay,
    }


def print_scenario(s):
    print(
        f"speed={s.speed:.2f} m/s, "
        f"height={s.usual_height:.2f} m"
    )

    print(
        f"structure x=[{s.box_x_min:.2f}, "
        f"{s.box_x_max:.2f}], "
        f"unsafe z<={s.unsafe_below_z:.2f}"
    )

    print(
        f"approval={s.approval_time:.2f}s, "
        f"command: z={s.low_height:.2f}m "
        f"for {s.low_seconds:.2f}s"
    )


if __name__ == "__main__":

    NUM_SCENARIOS = 10
    counts = {
        "INVALID_NOMINAL": 0,
        "INVALID_AT_APPROVAL": 0,
        "ALWAYS_SAFE": 0,
        "FINITE_TTL": 0,
        "NON_MONOTONIC": 0,
    }

    for scenario_id in range(NUM_SCENARIOS):
        s = generate_scenario(seed=scenario_id)

        print()
        print("=" * 70)
        print(f"SCENARIO {scenario_id}")
        print_scenario(s)

        outcome = classify_scenario(s)

        classification = outcome["classification"]
        counts[classification] += 1
        print(f"classification: {classification}")

        if outcome["results"] is not None:
            for delay, status in zip(
                DELAYS,
                outcome["results"],
            ):
                print(
                    f"  delay={delay:.1f}s "
                    f"-> {status}"
                )
        if classification == "FINITE_TTL":
            print(
                "TTL interval: "
                f"[{outcome['last_safe_delay']:.1f}, "
                f"{outcome['first_bad_delay']:.1f}) s"
            )
        elif classification == "ALWAYS_SAFE":
            print(
                f"No expiry observed up to "
                f"{DELAYS[-1]:.1f}s"
            )
        elif classification == "NON_MONOTONIC":
            print(
                "Safety is non-monotonic with delay."
            )

    print()
    print("=" * 70)
    print("SUMMARY")
    for name, count in counts.items():
        print(f"{name}: {count}")