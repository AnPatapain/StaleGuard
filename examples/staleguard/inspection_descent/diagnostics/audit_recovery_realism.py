"""Replay historical mission-retry schedules serially without changing them.

The saved StaleGuard decision times are replayed, not recomputed. Identical
executions across policies/delays run once. Raw recovery traces are retained.
"""

import argparse
import csv
import fcntl
import hashlib
import json
from pathlib import Path
import resource
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent)]

# Sets numerical library threads to one before importing the simulator.
from validate_official_calibration import check_memory, trace_hash
import numpy as np
import staleguard_benefit as benefit
from experiment import Scenario
from mission import capture_approval
from model.common import STATE_SIZE, STATE_X, STATE_Y, STATE_Z

SAVED = ROOT / "results" / "always_recover" / "official_experiment.jsonl"
OUTPUT = ROOT / "results" / "recovery_realism_audit"


def execution_key(row):
    # Dispatch has no effect when the pending command is cancelled. Conversely,
    # recovery time has no effect when recovery was not requested.
    start = row["recover_step"] if row["recover"] else row["dispatch_step"]
    return row["seed"], row["recover"], start, row["deadline_step"]


def measure_recovery(row, scenario, snapshot, trace, physics):
    steps = np.array([entry[0] for entry in trace])
    states = np.array([entry[1] for entry in trace])
    references = np.array([entry[2] for entry in trace])
    ticks = np.array([entry[0] for entry in physics])
    positions = np.array([entry[1] for entry in physics])
    scan_x = benefit.inspection_x(scenario)
    recovery = row["recover_step"]
    # Row k+1 contains the reference applied during control interval k.
    low_rows = np.flatnonzero(references[1:, 4] == scenario.low_height)
    descent = int(steps[low_rows[0] + 1] - 1) if len(low_rows) else None
    scan_start = None
    completion = None
    longest = 0
    unsafe = False
    state = [0.0] * STATE_SIZE
    for tick, (x, y, z) in zip(ticks, positions):
        state[STATE_X], state[STATE_Y], state[STATE_Z] = x, y, z
        unsafe |= benefit.unsafe(state, snapshot)
        if benefit.in_inspection_region(state, scenario, scan_x):
            if scan_start is None:
                scan_start = int(tick)
            longest = max(longest, int(tick) - scan_start)
            if completion is None and tick - scan_start >= round(benefit.SCAN_DWELL_S * benefit.PYB_FREQ_HZ):
                completion = int(tick)
        else:
            scan_start = None
    assert bool(completion is not None) == bool(row["outcome"][0])
    assert bool(unsafe) == bool(row["outcome"][1])
    if descent is not None:
        assert abs(states[descent, 0] - scan_x) < 0.08
        assert abs(states[descent, 1]) < 0.15
    after = states[recovery:]
    after_positions = positions[ticks >= recovery * benefit.PYB_FREQ_HZ // benefit.CTRL_FREQ_HZ]
    dt = 1 / benefit.CTRL_FREQ_HZ
    result = {
        "seed": row["seed"], "example_policy": row["policy"],
        "approval_s": snapshot.approval_time, "recovery_s": recovery * dt,
        "deadline_s": row["deadline_step"] * dt,
        "start_x_m": float(states[recovery, 0]), "start_vx_m_s": float(states[recovery, 1]),
        "target_x_m": scan_x,
        "recovery_to_descent_s": None if descent is None else (descent - recovery) * dt,
        "recovery_to_completion_s": None if completion is None else completion / benefit.PYB_FREQ_HZ - recovery * dt,
        "completion_deadline_margin_s": None if completion is None else row["deadline_step"] * dt - completion / benefit.PYB_FREQ_HZ,
        "best_inspection_dwell_s": longest / benefit.PYB_FREQ_HZ,
        "low_reference_duration_s": snapshot.low_steps * dt,
        "low_reference_end_s": None if descent is None else (descent + snapshot.low_steps) * dt,
        "descent_start_vz_m_s": None if descent is None else float(states[descent, 5]),
        "descent_start_z_error_m": None if descent is None else float(states[descent, 4] - scenario.usual_height),
        "descent_start_tilt_degrees": None if descent is None else float(np.rad2deg(np.arccos(np.clip(np.cos(states[descent, 6]) * np.cos(states[descent, 7]), -1, 1)))),
        "descent_start_angular_speed_rad_s": None if descent is None else float(np.linalg.norm(states[descent, 9:12])),
        "nominal_reference_speed_m_s": scenario.speed,
        "minimum_structure_x_margin_m": float(scenario.box_x_min - np.max(after_positions[:, 0])),
        "backward_distance_m": float(-np.minimum(np.diff(after_positions[:, 0]), 0).sum()),
        "minimum_vx_m_s": float(np.min(after[:, 1])),
        "peak_abs_vx_m_s": float(np.max(np.abs(after[:, 1]))),
        "peak_abs_vz_m_s": float(np.max(np.abs(after[:, 5]))),
        "peak_tilt_degrees": float(np.rad2deg(np.max(np.arccos(np.clip(np.cos(after[:, 6]) * np.cos(after[:, 7]), -1, 1))))),
        "final_x_error_m": float(states[-1, 0] - scan_x),
        "final_z_error_m": float(states[-1, 4] - scenario.usual_height),
        "final_vx_m_s": float(states[-1, 1]), "final_vz_m_s": float(states[-1, 5]),
        "flight_end_s": float(ticks[-1] / benefit.PYB_FREQ_HZ),
    }
    assert ticks[-1] == row["deadline_step"] * benefit.PYB_FREQ_HZ // benefit.CTRL_FREQ_HZ
    return result, dict(control_steps=steps, states=states, references=references,
                        physics_ticks=ticks, positions=positions)


def main(seeds=None):
    rows = [json.loads(line) for line in SAVED.read_text().splitlines()]
    if seeds is not None:
        rows = [row for row in rows if row["seed"] in seeds]
    groups = {}
    for row in rows:
        groups.setdefault(execution_key(row), []).append(row)
    files = list(ROOT.glob("*.py")) + list((ROOT / "model").glob("*.py")) + [benefit.CALIBRATION_FILE, SAVED]
    hashes = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}
    (OUTPUT / "preserved_files.json").write_text(json.dumps(hashes, indent=2) + "\n")
    snapshots = {}
    recoveries = []
    checked = []
    with (OUTPUT / "replays.jsonl").open("w") as stream:
        for index, (key, group) in enumerate(groups.items(), 1):
            check_memory()
            row = group[0]
            scenario = Scenario(**row["scenario"])
            if row["seed"] not in snapshots:
                snapshots[row["seed"]] = capture_approval(scenario)
            snapshot = snapshots[row["seed"]]
            assert all(other["scenario"] == row["scenario"] for other in group)
            if row["recover"]:
                assert round(snapshot.approval_time * benefit.CTRL_FREQ_HZ) <= row["recover_step"] < row["dispatch_step"]
            trace, physics = [], []
            outcome = benefit.run_actual(scenario, snapshot, row["dispatch_step"], row["recover_step"],
                                         row["deadline_step"], row["recover"], trace=trace, physics_trace=physics,
                                         recovery_mode="mission-retry")
            assert outcome[3] == (outcome[0] and not outcome[1])
            digest = trace_hash(trace, physics)
            for previous in group:
                assert list(outcome) == previous["outcome"], (key, outcome, previous)
                if previous["trace_sha256"] is not None:
                    assert digest == previous["trace_sha256"], (key, "trace changed")
                checked.append(previous)
            if row["recover"]:
                measured, arrays = measure_recovery(row, scenario, snapshot, trace, physics)
                measured["represented_executions"] = len(group)
                recoveries.append(measured)
                np.savez_compressed(OUTPUT / f"trace_seed_{row['seed']}_recovery_{row['recover_step']}.npz", **arrays)
            stream.write(json.dumps({"key": key, "outcome": outcome, "trace_sha256": digest,
                                     "represented_executions": len(group)}) + "\n")
            stream.flush()
            if index % 20 == 0 or index == len(groups):
                print(f"Replayed {index}/{len(groups)} distinct flights; matched {len(checked)} saved outcomes; RSS={check_memory()[0]:.0f} MiB", flush=True)
    with (OUTPUT / "recovery_measurements.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(recoveries[0]))
        writer.writeheader()
        writer.writerows(recoveries)
    metrics = []
    for delay in benefit.DELAY_GRID:
        block = [r for r in checked if abs(r["delay_s"] - float(delay)) < 1e-9]
        totals = {policy: {name: sum(r["outcome"][j] for r in block if r["policy"] == policy)
                           for j, name in enumerate(("inspection", "unsafe", "recoveries", "safe"))}
                  for policy in ("No Recovery", "Always-Recover", "StaleGuard")}
        paired = {p: {r["seed"]: r for r in block if r["policy"] == p} for p in totals}
        gain = harm = 0
        for seed, sg in paired["StaleGuard"].items():
            s1, r = sg["outcome"][3], sg["outcome"][2]
            s0 = paired["No Recovery"][seed]["outcome"][3]
            gain += bool(r and s1 and not s0)
            harm += bool(r and not s1 and s0)
        assert gain - harm == totals["StaleGuard"]["safe"] - totals["No Recovery"]["safe"]
        metrics.append({"delay_s": float(delay), "N": len(paired["StaleGuard"]),
                        "policies": totals, "benefit": gain, "harm": harm})
    for name, digest in hashes.items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, name
    summary = {"protocol_changed": False, "staleguard_decision_times": "replayed from saved experiment",
               "distinct_flights_executed": len(groups), "saved_outcomes_matched": len(checked),
               "distinct_recovery_flights": len(recoveries),
               "previous_trace_hashes_matched": sum(r["trace_sha256"] is not None for r in checked),
               "peak_rss_mib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
               "metrics": metrics}
    (OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print("D (s)  N   Safe completions: No Recovery / Always-Recover / StaleGuard", flush=True)
    for entry in metrics:
        safe = [str(entry["policies"][p]["safe"]) for p in ("No Recovery", "Always-Recover", "StaleGuard")]
        print(f"{entry['delay_s']:.1f}    {entry['N']:2}   " + " / ".join(safe), flush=True)
    print(f"PASS: outcomes, recorded trace hashes, individual safe-completion definitions, paired identities, and unchanged mission sources. Peak RSS={summary['peak_rss_mib']:.0f} MiB.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seeds", type=int, nargs="+", help="Optional small check before the full replay.")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    OUTPUT = args.output
    OUTPUT.mkdir(parents=True, exist_ok=True)
    with (ROOT / "results" / "always_recover" / ".validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        main(args.seeds)
