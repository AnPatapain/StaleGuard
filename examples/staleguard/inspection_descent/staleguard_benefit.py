"""Experiment B: benefit of early intervention for queued UAV commands.

Place next to mission.py, staleguard.py, and experiment.py.
Run: python staleguard_benefit.py --count 50

Important: mission success below is a CAMERA FLY-BY inspection, not a hover.
A hover-based task needs a new primary command (the current command only dips z).
"""
import argparse
import csv
import math
import time

import numpy as np

from experiment import (
    ApprovalSnapshot, CTRL_FREQ_HZ, DELAY_GRID, MEASURE_SECONDS,
    PYB_FREQ_HZ, SIM_SEED, apply_descent_command,
    generate_scenario, make_forward_trajectory, unsafe,
)
from model.common import capture_pid_state
from model.lightweight import LightweightPredictiveModel
from model.scg import load_scg_platform_params, make_scg_env_factory
from safe_control_gym.utils.registration import make
from staleguard import predict

HZ = CTRL_FREQ_HZ
# Explicit operational assumptions to vary in sensitivity analysis.
UPLINK_S = 0.04
# LLM_S = 0.15
DOWNLINK_S = 0.10
SCAN_DWELL_S = 0.18
FIXED_TTL_S = 0.60
MODES = ("use_time", "fixed_ttl", "staleguard", "scheduled_rollout", "always_early")


def steps(seconds):
    return round(seconds * HZ)


def ceil_steps(seconds):
    return max(1, math.ceil(seconds * HZ))


def snapshot_now(obs, ctrl, reference, current_step, scenario):
    """Fresh reception/dispatch snapshot; reference is in absolute mission indices."""
    return ApprovalSnapshot(
        state=np.asarray(obs, dtype=float).copy(),
        pid_state=capture_pid_state(ctrl),
        remaining_reference=np.asarray(reference[current_step:]).copy(),
        low_height=float(scenario.low_height),
        low_steps=steps(scenario.low_seconds),
        box_x_min=float(scenario.box_x_min),
        box_x_max=float(scenario.box_x_max),
        box_y_limit=float(scenario.box_y_limit),
        unsafe_below_z=float(scenario.unsafe_below_z),
        horizon_steps=len(reference) - current_step,
    )


def check(snapshot, params, delays):
    """Use the SAME lightweight predictor for all predictive policies."""
    start = time.perf_counter()
    result = predict(
        snapshot, LightweightPredictiveModel(*params), delays=delays,
    )
    return result, time.perf_counter() - start


def predicted_unsafe(result, index=0):
    results = result["results"]
    return results is None or results[index] != "SAFE"


def early_staleguard_decision(result, queue_delay):
    """Conservative first-unsafe-delay interpretation (also for non-monotonic profiles)."""
    if result["classification"] in {"INVALID_NOMINAL", "INVALID_AT_APPROVAL"}:
        return True
    first_bad = result["first_bad_delay"]
    return first_bad is not None and queue_delay + 1e-8 >= first_bad


def smooth_sample(a, b, progress, duration):
    """Position and velocity for a zero-endpoint-velocity quintic segment."""
    u = min(1., max(0., progress))
    w = 10*u**3 - 15*u**4 + 6*u**5
    wd = (30*u**2 - 60*u**3 + 30*u**4) / duration
    return a + (b - a)*w, (b - a)*wd


def recovery_trajectory(reference, start, obs, scenario):
    """Climb -> reposition outside structure -> descend -> scan/hold.

    This trajectory is generated from the ACTUAL state at intervention time,
    with no teleportation. Safety and successful scanning are measured in SCG.
    """
    ref = np.asarray(reference).copy()
    x0, y0, z0 = float(obs[0]), float(obs[2]), float(obs[4])
    x_target = scenario.box_x_min - 0.16  # front-face camera position
    z_cruise = max(scenario.usual_height + 0.08,
                   scenario.unsafe_below_z + 0.18)
    z_target = scenario.low_height

    # Each stage is a physical reference trajectory, not a simulated wait.
    stages = (
        (max(0.55, 1.5*abs(z_cruise-z0)/0.45),
         (x0, y0, z0), (x0, 0., z_cruise)),
        (max(0.65, 1.5*abs(x_target-x0)/0.55),
         (x0, 0., z_cruise), (x_target, 0., z_cruise)),
        (max(0.65, 1.5*abs(z_cruise-z_target)/0.45),
         (x_target, 0., z_cruise), (x_target, 0., z_target)),
    )

    idx = start
    for duration, p0, p1 in stages:
        n = max(1, steps(duration))
        for j in range(n):
            if idx >= len(ref):
                return ref
            x, vx = smooth_sample(p0[0], p1[0], j/n, duration)
            y, vy = smooth_sample(p0[1], p1[1], j/n, duration)
            z, vz = smooth_sample(p0[2], p1[2], j/n, duration)
            ref[idx, :] = 0.
            ref[idx, [0, 1, 2, 3, 4, 5]] = [x, vx, y, vy, z, vz]
            idx += 1
    if idx < len(ref):
        ref[idx:, :] = 0.
        ref[idx:, 0] = x_target
        ref[idx:, 4] = z_target
    return ref


def scanning(obs, s):
    """Front-face image acquisition while inside a narrow observation window."""
    return (s.box_x_min - 0.27 <= obs[0] <= s.box_x_min - 0.055
            and abs(obs[2]) <= s.box_y_limit
            and abs(obs[4] - s.low_height) <= 0.11)


def run_trial(s, mode, queue_delay, deadline, params):
    """One independent SCG flight, replayed under one policy.

    D is queue wait AFTER command receipt, distinct from downlink latency.
    The queue slot is known at receipt in this initial experiment.
    """
    factory = make_scg_env_factory(
        init_state={"init_x": 0., "init_y": 0., "init_z": s.usual_height},
        pyb_freq=PYB_FREQ_HZ,
    )
    env = factory()
    ctrl = None
    try:
        ctrl = make("pid", factory)
        obs, info = env.reset(seed=SIM_SEED)
        ctrl.reset_before_run(obs=obs, info=info, env=env)
        nominal = make_forward_trajectory(env, s)
        env.X_GOAL = nominal.copy()
        ctrl.reference = nominal.copy()

        # Observation -> uplink -> LLM reasoning -> approval -> downlink -> receipt.
        t_obs_sent = 0 # UAV sends observation to ground-station.
        t_obs_recv = t_obs_sent + UPLINK_S
        t_cmd_approved = s.approval_time
        t_cmd_recv = t_cmd_approved + DOWNLINK_S
        recv = steps(t_cmd_recv)
        slot = recv + steps(queue_delay)
        total = min(steps(deadline), len(nominal))

        pending = True
        early_ready = None
        early_recover = False
        check_ready = None
        check_is_safe = None
        compute_s = 0.
        recovered = False
        scan_count = 0
        completed_at = None
        unsafe_at = None

        for i in range(total):
            if i == recv:
                fresh = snapshot_now(obs, ctrl, ctrl.reference, i, s)
                if mode == "staleguard":
                    result, elapsed = check(fresh, params, DELAY_GRID)
                    compute_s += elapsed
                    early_ready = recv + ceil_steps(elapsed)
                    early_recover = early_staleguard_decision(result, queue_delay)
                elif mode == "scheduled_rollout":
                    result, elapsed = check(fresh, params, [queue_delay])
                    compute_s += elapsed
                    early_ready = recv + ceil_steps(elapsed)
                    early_recover = predicted_unsafe(result)
                elif mode == "always_early":
                    early_ready, early_recover = recv, True
                elif mode == "fixed_ttl":
                    early_ready, early_recover = recv, queue_delay >= FIXED_TTL_S

            # StaleGuard cannot intervene until its calculation finishes.
            if pending and i == early_ready and early_recover:
                ctrl.reference = recovery_trajectory(ctrl.reference, i, obs, s)
                recovered = True
                pending = False  # cancel queued descent command

            # Dispatch-time check is SHARED among policies not already recovering.
            scheduled_check = slot
            if early_ready is not None:
                scheduled_check = max(slot, early_ready)
            if pending and i == scheduled_check and i >= recv:
                fresh = snapshot_now(obs, ctrl, ctrl.reference, i, s)
                result, elapsed = check(fresh, params, [0.])
                compute_s += elapsed
                check_is_safe = not predicted_unsafe(result)
                check_ready = i + ceil_steps(elapsed)

            if pending and i == check_ready:
                if check_is_safe:
                    ctrl.reference = apply_descent_command(
                        reference=ctrl.reference,
                        start_step=i,
                        low_steps=steps(s.low_seconds),
                        low_height=s.low_height,
                    )
                else:
                    ctrl.reference = recovery_trajectory(ctrl.reference, i, obs, s)
                    recovered = True
                pending = False

            action = ctrl.select_action(obs, info)
            obs, _, done, info = env.step(action)

            # Actual PyBullet state is the ONLY ground truth for outcome.
            if unsafe(obs, s):
                unsafe_at = (i + 1) / HZ
                break
            scan_count = scan_count + 1 if scanning(obs, s) else 0
            if scan_count >= ceil_steps(SCAN_DWELL_S):
                completed_at = (i + 1) / HZ
                break
            if done:
                break

        return {
            "success": completed_at is not None and unsafe_at is None,
            "unsafe": unsafe_at is not None,
            "completed_at": completed_at,
            "recovered": recovered,
            "compute_ms": 1000*compute_s,
        }
    finally:
        if ctrl is not None:
            ctrl.close()
        env.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--delays", nargs="+", type=float,
                        default=[0.0, 0.6, 1.2])
    parser.add_argument("--deadlines", nargs="+", type=float,
                        default=[4.2, 5.0])
    parser.add_argument("--save", default="benefit_trials.csv")
    args = parser.parse_args()
    params = load_scg_platform_params()

    # JIT warmup: compilation cost is not part of on-board decision latency.
    warm_s = generate_scenario(0)
    # Warm up against a real reception snapshot (without changing mission state).
    from mission import capture_approval
    warm = capture_approval(warm_s)
    check(warm, params, DELAY_GRID)

    print("mode,queue_delay_s,deadline_s,n,success_rate,unsafe_rate,"
          "recovery_rate,mean_compute_ms")
    with open(args.save, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "scenario", "mode", "queue_delay_s", "deadline_s",
            "success", "unsafe", "completed_at", "recovered", "compute_ms",
        ])
        writer.writeheader()
        for delay in args.delays:
            for deadline in args.deadlines:
                for mode in MODES:
                    rows = []
                    for j in range(args.count):
                        result = run_trial(generate_scenario(j), mode, delay,
                                           deadline, params)
                        writer.writerow(dict(scenario=j, mode=mode,
                                             queue_delay_s=delay,
                                             deadline_s=deadline, **result))
                        rows.append(result)
                    f.flush()
                    n = len(rows)
                    print(f"{mode},{delay:.2f},{deadline:.2f},{n},"
                          f"{sum(r['success'] for r in rows)/n:.4f},"
                          f"{sum(r['unsafe'] for r in rows)/n:.4f},"
                          f"{sum(r['recovered'] for r in rows)/n:.4f},"
                          f"{np.mean([r['compute_ms'] for r in rows]):.2f}",
                          flush=True)


if __name__ == "__main__":
    main()