
"""T0 mission runner: capture approval observation, then compare independent runs.

Usage (in your Safe-Control-Gym environment):
    python mission.py --scenario 0
    python mission.py --count 10

The predictor never imports ground_truth.py. This runner imports it only to
supply benchmark scenarios and obtain OFFLINE labels after prediction.
"""

import argparse
from dataclasses import dataclass
from functools import partial

import numpy as np
from safe_control_gym.utils.registration import get_config, make

FREQ = 50
EPISODE_SECONDS = 6
MEASURE_SECONDS = 5.5
DELAYS = tuple(np.arange(0.0, 1.51, 0.1))  # Match frozen GT grid exactly
SEED = 42

@dataclass(frozen=True)
class ApprovalSnapshot:
    scenario: object            # geometry, nominal plan, approved command
    approval_step: int          # integer controller ticks, not rounded seconds
    observation: np.ndarray    # exact observation BEFORE approval-step action
    seed: int                   # for deterministic reconstruction of PID state

def unsafe(obs, s):
    return (s.box_x_min <= obs[0] <= s.box_x_max
            and abs(obs[2]) <= s.box_y_limit
            and obs[4] <= s.unsafe_below_z)

def forward_trajectory(env, s):
    # Intentionally matches the FROZEN ground-truth simulator.
    n = env.X_GOAL.shape[0]
    dt = 1.0 / env.CTRL_FREQ
    times = np.arange(n) * dt
    speeds = s.speed * np.minimum(times / 0.4, 1.0)
    traj = np.zeros_like(env.X_GOAL)
    traj[:, 0] = np.cumsum(speeds) * dt
    traj[:, 1] = speeds
    traj[:, 4] = s.usual_height
    return traj

def start_simulator(s, seed=SEED):
    config = get_config("quadrotor")
    config.update({
        "quad_type": 3, "task": "traj_tracking", "cost": "quadratic",
        "ctrl_freq": FREQ, "pyb_freq": 1000,
        "episode_len_sec": EPISODE_SECONDS,
        "init_state": {"init_x": 0.0, "init_y": 0.0,
                       "init_z": s.usual_height},
        "randomized_init": False,
        "randomized_inertial_prop": False,
        "disturbances": None,
        "adversary_disturbance": None,
        "done_on_out_of_bound": False,
        "gui": False,
    })
    env_factory = partial(make, "quadrotor", **config)
    env = env_factory()
    ctrl = None
    try:
        ctrl = make("pid", env_factory)
        obs, info = env.reset(seed=seed)
        ctrl.reset_before_run(obs=obs, info=info, env=env)
        traj = forward_trajectory(env, s)
        env.X_GOAL = traj.copy()
        ctrl.reference = traj.copy()
        return env, ctrl, obs, info, traj
    except Exception:
        if ctrl is not None:
            ctrl.close()
        env.close()
        raise

def replay_until_approval(s, seed=SEED):
    """Replay nominal control through the tick BEFORE approval.

    This reconstructs the *entire* simulator and PID internal state; an
    observation vector alone cannot restore a stateful PID controller.
    Caller must close env and ctrl (including on exceptions).
    """
    env, ctrl, obs, info, traj = start_simulator(s, seed)
    approval_step = round(s.approval_time * FREQ)
    try:
        for _ in range(approval_step):
            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)
            if done:
                raise RuntimeError("Mission terminated before approval")
        return env, ctrl, obs, info, traj, approval_step
    except Exception:
        ctrl.close()
        env.close()
        raise

def capture_approval(s, seed=SEED):
    env, ctrl, obs, info, traj, approval_step = replay_until_approval(s, seed)
    try:
        return ApprovalSnapshot(s, approval_step, np.asarray(obs).copy(), seed)
    finally:
        ctrl.close()
        env.close()

def counterfactual(snapshot, delay):
    """Predict one delay from approval; None means no command (nominal).

    T0: exact known dynamics, geometry, and state. For each candidate, replay
    to approval with the same seed to reconstruct PyBullet + PID state, verify
    the captured observation, then branch. No ground-truth results are read.
    """
    s = snapshot.scenario
    env, ctrl, obs, info, traj, approval_step = replay_until_approval(
        s, snapshot.seed
    )
    try:
        if approval_step != snapshot.approval_step:
            raise RuntimeError("Approval tick changed")
        if not np.allclose(obs, snapshot.observation, rtol=1e-9, atol=1e-9):
            raise RuntimeError("Approval replay differs: simulator not deterministic")

        dispatch_step = (None if delay is None else
                         approval_step + round(float(delay) * FREQ))
        low_steps = round(s.low_seconds * FREQ)
        total_steps = round(MEASURE_SECONDS * FREQ)
        if dispatch_step is not None and dispatch_step >= total_steps:
            raise ValueError("Dispatch beyond evaluation horizon")

        # Exactly mirrors ground_truth.run_one's state indexing:
        # states[approval_step] = pre-dispatch observation at approval.
        states = [np.asarray(obs).copy()]
        for step in range(approval_step, total_steps):
            if step == dispatch_step:
              changed = traj.copy()
              changed[step:step + low_steps, 4] = s.low_height
              env.X_GOAL = changed
              ctrl.reference = changed
            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)
            states.append(np.asarray(obs).copy())
            if done:
                break

        if dispatch_step is None:
            return "NOMINAL_UNSAFE" if any(unsafe(x, s) for x in states) else "SAFE"

        boundary = dispatch_step - approval_step + 1
        if any(unsafe(x, s) for x in states[:boundary]):
            return "WAIT_UNSAFE"
        if any(unsafe(x, s) for x in states[boundary:]):
            return "UNSAFE"
        return "SAFE"
    finally:
        ctrl.close()
        env.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", type=int, default=0,
                        help="Single scenario ID (default: 0)")
    parser.add_argument("--count", type=int, default=None,
                        help="Instead evaluate scenarios 0..count-1")
    args = parser.parse_args()

    # The benchmark is imported ONLY here, not by staleguard.py.
    import ground_truth
    from staleguard import predict

    ids = range(args.count) if args.count is not None else [args.scenario]
    for scenario_id in ids:
        s = ground_truth.generate_scenario(scenario_id)
        snapshot = capture_approval(s)
        predicted = predict(snapshot, DELAYS)         # No access to labels
        actual = ground_truth.classify_scenario(s)    # Independent replay

        print(f"\nSCENARIO {scenario_id}")
        print("delay   predicted      actual")
        if predicted["results"] is not None and actual["results"] is not None:
            for d, pred, gt in zip(DELAYS, predicted["results"], actual["results"]):
                print(f"{d:>4.1f}s   {pred:<14} {gt}")
        print(f"class: {predicted['classification']} / {actual['classification']}")
        print(f"first bad: {predicted['first_bad_delay']} / {actual['first_bad_delay']}")
        print(f"last safe: {predicted['last_safe_delay']} / {actual['last_safe_delay']}")
        match = (predicted["classification"] == actual["classification"]
                 and predicted["results"] == actual["results"]
                 and predicted["first_bad_delay"] == actual["first_bad_delay"]
                 and predicted["last_safe_delay"] == actual["last_safe_delay"])
        print("MATCH" if match else "MISMATCH")
        if not match:
            raise AssertionError(f"Scenario {scenario_id}: T0 prediction differs")

if __name__ == "__main__":
    main()