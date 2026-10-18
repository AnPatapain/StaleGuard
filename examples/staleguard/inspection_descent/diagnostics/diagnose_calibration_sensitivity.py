"""Diagnose first-feasible calibration sensitivity without changing the planner.

Predeclared design: a 3x3 local grid per existing calibration, with approval
0/1/2 planner grid steps earlier and low duration 0/1/2 steps longer. Deduplicate
approval times clipped at zero and retain the planner's structure-arrival bound.
First execute ALL candidates at D=0. Only after freezing D=0 eligibility, execute
EVERY eligible candidate at D=0.1 and 0.2. Also test the selected commands at the
four intervening control-step delays (0.02, 0.04, 0.06, 0.08 s).

This is a diagnostic inventory, not a replacement calibration policy. Positive
delay outcomes never select candidates or change any production command.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import csv
from dataclasses import asdict, fields, replace
import json

import numpy as np

from experiment import (
    ACCELERATION_SECONDS, CTRL_FREQ_HZ, DELAY_GRID, PYB_FREQ_HZ,
    Scenario, generate_scenario, inspection_x,
)
from mission import capture_approval
from staleguard_benefit import SCAN_DWELL_S, SCENARIOS, run_actual


DEADLINE_S = 4.2


def groups(mask):
    indices = np.flatnonzero(mask)
    return np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1) if len(indices) else []


def measure(scenario, snapshot, delay):
    physics_trace = []
    dispatch_step = round((scenario.approval_time + delay) * CTRL_FREQ_HZ)
    deadline_step = round((scenario.approval_time + DEADLINE_S) * CTRL_FREQ_HZ)
    completed, unsafe, recovered, safe = run_actual(
        scenario, snapshot, dispatch_step, 0, deadline_step, False,
        physics_trace=physics_trace,
    )
    assert not recovered and safe == (completed and not unsafe)
    ticks = np.asarray([row[0] for row in physics_trace])
    assert np.array_equal(ticks, np.arange(len(ticks)))
    positions = np.asarray([row[1] for row in physics_trace])
    # Measurement of the existing inspection predicate; outcomes above come
    # directly from the unchanged run_actual, not from this reconstruction.
    x_ok = np.abs(positions[:, 0] - inspection_x(scenario)) < 0.12
    y_ok = np.abs(positions[:, 1]) < 0.20
    z_ok = np.abs(positions[:, 2] - scenario.low_height) < 0.08
    scans = groups(x_ok & y_ok & z_ok)
    best = max(scans, key=len) if scans else None
    dwell_ticks = len(best) - 1 if best is not None else 0
    assert completed == (dwell_ticks >= round(SCAN_DWELL_S * PYB_FREQ_HZ))
    x_groups, z_groups = groups(x_ok), groups(z_ok)
    height_entry = int(np.flatnonzero(z_ok)[0]) if z_ok.any() else None
    exit_cause = "no_overlap"
    if best is not None:
        next_index = int(best[-1]) + 1
        exit_cause = "+".join(
            axis for axis, mask in [("x", x_ok), ("y", y_ok), ("z", z_ok)]
            if next_index < len(ticks) and not mask[next_index]
        ) or "rollout_end"
    return {
        "delay_s": round(delay, 8),
        "inspection_completed": completed,
        "unsafe_entry": unsafe,
        "recovery_started": recovered,
        "safe_completion": safe,
        "classification": "C" if unsafe else "A" if completed else "B",
        "dispatch_step": dispatch_step,
        "deadline_step": deadline_step,
        "best_dwell_s": dwell_ticks / PYB_FREQ_HZ,
        "dwell_margin_s": (dwell_ticks - round(SCAN_DWELL_S * PYB_FREQ_HZ)) / PYB_FREQ_HZ,
        "inspection_entry_s": int(best[0]) / PYB_FREQ_HZ if best is not None else None,
        "inspection_last_inside_s": int(best[-1]) / PYB_FREQ_HZ if best is not None else None,
        "inspection_exit_cause": exit_cause,
        "first_height_entry_s": height_entry / PYB_FREQ_HZ if height_entry is not None else None,
        "x_at_first_height_entry_m": float(positions[height_entry, 0]) if height_entry is not None else None,
        "max_x_window_s": max((len(g) - 1 for g in x_groups), default=0) / PYB_FREQ_HZ,
        "max_height_window_s": max((len(g) - 1 for g in z_groups), default=0) / PYB_FREQ_HZ,
        "max_abs_y_m": float(np.abs(positions[:, 1]).max()),
        "monitored_until_s": int(ticks[-1]) / PYB_FREQ_HZ,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    directory = Path(__file__).resolve().parents[1] / "results"
    parser.add_argument("--cohort-csv", type=Path, default=directory / "no_recovery_delay_sweep.csv")
    parser.add_argument("--output-dir", type=Path, default=directory / "calibration_sensitivity")
    args = parser.parse_args()
    with args.cohort_csv.open(newline="") as stream:
        baseline_rows = [row for row in csv.DictReader(stream) if float(row["delay_s"]) == 0]
    assert len(baseline_rows) == SCENARIOS
    baseline_rows.sort(key=lambda row: int(row["scenario_id"]))
    assert [int(row["scenario_id"]) for row in baseline_rows] == list(range(SCENARIOS))
    step = round((DELAY_GRID[1] - DELAY_GRID[0]) * CTRL_FREQ_HZ)
    candidates = []
    for row in baseline_rows:
        scenario_id = int(row["scenario_id"])
        selected = Scenario(**{field.name: float(row[field.name]) for field in fields(Scenario)})
        original = generate_scenario(scenario_id)
        for field in fields(Scenario):
            if field.name not in {"approval_time", "low_seconds"}:
                assert getattr(original, field.name) == getattr(selected, field.name)
        approval = round(selected.approval_time * CTRL_FREQ_HZ)
        duration = round(selected.low_seconds * CTRL_FREQ_HZ)
        structure_step = round((selected.box_x_min / selected.speed + ACCELERATION_SECONDS / 2) * CTRL_FREQ_HZ)
        seen = set()
        for earlier in range(3):
            for longer in range(3):
                a, length = max(0, approval - earlier * step), duration + longer * step
                if (a, length) in seen or a + length > structure_step:
                    continue
                seen.add((a, length))
                scenario = replace(selected, approval_time=a / CTRL_FREQ_HZ, low_seconds=length / CTRL_FREQ_HZ)
                candidates.append({
                    "scenario_id": scenario_id,
                    "candidate_id": f"s{scenario_id}_a{a}_l{length}",
                    "earlier_grid_steps": earlier,
                    "longer_grid_steps": longer,
                    "selected": earlier == longer == 0,
                    "scenario": scenario,
                    "snapshot": capture_approval(scenario),
                })

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        "deadline_s": DEADLINE_S,
        "planner_grid_step_s": step / CTRL_FREQ_HZ,
        "candidate_design": "Approval 0/1/2 grid steps earlier x duration 0/1/2 steps longer; zero clipping and original structure bound",
        "eligibility_rule": "D=0 safe completion only, frozen before any positive-delay simulation",
        "positive_delays_s": [float(DELAY_GRID[1]), float(DELAY_GRID[2])],
        "baseline_fine_delays_s": [tick / CTRL_FREQ_HZ for tick in range(1, step)],
        "candidates": [{key: value for key, value in candidate.items() if key not in {"scenario", "snapshot"}}
                       | asdict(candidate["scenario"]) for candidate in candidates],
    }
    (args.output_dir / "design.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Predeclared {len(candidates)} local candidates across {SCENARIOS} scenarios.", flush=True)
    results = []

    def record(candidate, phase, delay):
        outcome = measure(candidate["scenario"], candidate["snapshot"], delay)
        result = {key: value for key, value in candidate.items() if key not in {"scenario", "snapshot"}}
        result |= {"phase": phase, **asdict(candidate["scenario"]), **outcome}
        results.append(result)
        log.write(json.dumps(result) + "\n")
        log.flush()
        return outcome

    with (args.output_dir / "executions.jsonl").open("w") as log:
        eligible = []
        for candidate in candidates:
            outcome = record(candidate, "d0_inventory", 0.0)
            if candidate["selected"]:
                assert outcome["safe_completion"], candidate["scenario_id"]
            if outcome["safe_completion"]:
                eligible.append(candidate)
        # This immutable eligibility inventory precedes all delayed outcomes.
        (args.output_dir / "d0_eligible.json").write_text(json.dumps(
            [candidate["candidate_id"] for candidate in eligible], indent=2,
        ) + "\n")
        print(f"D=0 inventory complete: {len(eligible)}/{len(candidates)} candidates safely feasible.", flush=True)
        for delay in (float(DELAY_GRID[1]), float(DELAY_GRID[2])):
            for candidate in eligible:
                record(candidate, "eligible_delayed", delay)
            delayed = [r for r in results if r["phase"] == "eligible_delayed" and r["delay_s"] == round(delay, 8)]
            print(f"D={delay:.1f}: {sum(r['safe_completion'] for r in delayed)}/{len(delayed)} D=0-feasible candidates safely complete.", flush=True)
        for tick in range(1, step):
            for candidate in candidates:
                if candidate["selected"]:
                    record(candidate, "selected_fine_delay", tick / CTRL_FREQ_HZ)
    with (args.output_dir / "executions.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(results[0]))
        writer.writeheader()
        writer.writerows(results)
    print(f"Executed {len(results)} rollouts. Results: {args.output_dir}", flush=True)


if __name__ == "__main__":
    main()
