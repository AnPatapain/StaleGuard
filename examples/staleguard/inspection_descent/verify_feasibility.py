"""Measure D=0 feasibility without changing the continuous flight or guard.

Run with the SCG Python environment:
    python examples/staleguard/inspection_descent/verify_feasibility.py
    python examples/staleguard/inspection_descent/verify_feasibility.py --scenario 1

JSON lines include original and planned outcomes. Inspection/safety are observed
at PYB_FREQ_HZ; dwell is elapsed time between consecutive qualifying states.
"""

import argparse
import contextlib
import io
import json
from dataclasses import asdict, replace

import numpy as np

from experiment import ACCELERATION_SECONDS, CTRL_FREQ_HZ, PYB_FREQ_HZ, generate_scenario, inspection_x
from mission import capture_approval
from model.common import STATE_VX, STATE_X, STATE_Y, STATE_Z
from staleguard_benefit import DEADLINES, plan_feasible_command, run_actual


def intervals(mask, times):
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    return [
        {
            "entry_s": float(times[group[0]]),
            "last_inside_s": float(times[group[-1]]),
            "first_outside_s": (
                float(times[group[-1] + 1])
                if group[-1] + 1 < len(times) else None
            ),
            "elapsed_dwell_s": (len(group) - 1) / PYB_FREQ_HZ,
        }
        for group in groups
    ]


def measure(scenario):
    snapshot = capture_approval(scenario)
    approval_step = round(scenario.approval_time * CTRL_FREQ_HZ)
    trace = []
    physics_trace = []
    success, collision, _ = run_actual(
        scenario, snapshot, approval_step, 0,
        round((scenario.approval_time + min(DEADLINES)) * CTRL_FREQ_HZ),
        False, trace=trace, physics_trace=physics_trace,
    )
    # capture_approval and actual execution must fly the same nominal prefix.
    np.testing.assert_array_equal(trace[approval_step][1], snapshot.state)
    times = np.asarray([row[0] / CTRL_FREQ_HZ for row in trace[1:]])
    states = np.asarray([row[1] for row in trace[1:]])
    references = np.asarray([row[2] for row in trace[1:]])
    remaining = snapshot.remaining_reference[:len(references) - approval_step]
    np.testing.assert_array_equal(
        references[approval_step:, [STATE_X, STATE_VX]],
        remaining[:, [STATE_X, STATE_VX]],
    )
    physics_times = np.asarray([row[0] / PYB_FREQ_HZ for row in physics_trace])
    positions = np.asarray([row[1] for row in physics_trace])
    x_mask = np.abs(positions[:, 0] - inspection_x(scenario)) < 0.12
    z_mask = np.abs(positions[:, 2] - scenario.low_height) < 0.08
    scan_mask = x_mask & z_mask & (np.abs(positions[:, 1]) < 0.20)
    scan_intervals = intervals(scan_mask, physics_times)
    crossing = np.flatnonzero(states[:, STATE_X] >= inspection_x(scenario))
    first = int(crossing[0]) if len(crossing) else None
    return {
        "command": asdict(scenario),
        "safe_completion": bool(success),
        "unsafe_entry": bool(collision),
        "best_dwell_s": max(
            (entry["elapsed_dwell_s"] for entry in scan_intervals), default=0.0,
        ),
        "x_intervals": intervals(x_mask, physics_times),
        "altitude_intervals": intervals(z_mask, physics_times),
        "inspection_intervals": scan_intervals,
        "min_z_m": float(positions[:, 2].min()),
        "min_vx_after_ramp_m_s": float(states[times >= ACCELERATION_SECONDS, STATE_VX].min()),
        "crossing": None if first is None else {
            "time_s": float(times[first]),
            "x_m": float(states[first, STATE_X]),
            "reference_x_m": float(references[first, STATE_X]),
            "z_m": float(states[first, STATE_Z]),
            "reference_z_m": float(references[first, STATE_Z]),
            "vx_m_s": float(states[first, STATE_VX]),
            "reference_vx_m_s": float(references[first, STATE_VX]),
        },
        "snapshot_and_forward_reference_match": True,
        "monitored_until_s": float(times[-1]),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", type=int)
    args = parser.parse_args()
    seeds = [args.scenario] if args.scenario is not None else range(10)
    passed = 0
    for seed in seeds:
        original = generate_scenario(seed)
        print(json.dumps({"seed": seed, "kind": "original", **measure(original)}), flush=True)
        if seed == 1:
            # Reproduce the handoff and isolate approval timing from duration.
            for approval, duration in [(0.1, 0.9), (0.1, 1.3), (0.0, 0.9)]:
                candidate = replace(original, approval_time=approval, low_seconds=duration)
                print(json.dumps({"seed": seed, "kind": "diagnostic", **measure(candidate)}), flush=True)
        with contextlib.redirect_stdout(io.StringIO()):
            planned = plan_feasible_command(original)
        if planned is None:
            print(json.dumps({"seed": seed, "kind": "planned", "feasible": False}), flush=True)
            continue
        measured = measure(planned)
        passed += int(measured["safe_completion"])
        print(json.dumps({"seed": seed, "kind": "planned", **measured}), flush=True)
    print(json.dumps({"feasible": passed, "total": len(seeds), "filtered": 0}), flush=True)
    if passed != len(seeds):
        raise SystemExit("D=0 feasibility failed; do not run the benefit matrix.")


if __name__ == "__main__":
    main()
