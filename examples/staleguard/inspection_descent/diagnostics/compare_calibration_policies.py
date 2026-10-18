"""Compare first-feasible and maximum-D=0-dwell calibration without delay leakage.

Default: calibrate seeds 0-19 on the complete existing candidate grid, freeze
both policies for both cohorts, then execute No Recovery at every DELAY_GRID
point with the existing minimum deadline (4.2 s). Fresh seeds are fixed at 10-19.

The prepare/calibrate/evaluate stages allow the two independent D=0 inventories
to run in separate processes. Evaluation requires all 20 completed inventories.
Completed per-seed inventories are reusable with --resume and matching hashes.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
import time

from experiment import ACCELERATION_SECONDS, CTRL_FREQ_HZ, DELAY_GRID, PYB_FREQ_HZ, SIM_SEED, Scenario, generate_scenario, inspection_x
from max_dwell_calibration import command_candidates, inspection_dwell_ticks, plan_max_dwell_command
from mission import capture_approval
from staleguard_benefit import DEADLINES, SCAN_DWELL_S, SCENARIOS, run_actual


POLICIES = ("first_feasible", "max_d0_dwell")
COHORTS = {"original": list(range(SCENARIOS)), "fresh": list(range(SCENARIOS, 2 * SCENARIOS))}
DIRECTORY = Path(__file__).resolve().parents[1]


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()


def protocol():
    protected = [DIRECTORY / name for name in (
        "experiment.py", "mission.py", "staleguard_benefit.py", "staleguard.py",
        "max_dwell_calibration.py", "diagnostics/compare_calibration_policies.py",
    )]
    protected.extend(sorted((DIRECTORY / "model").glob("*.py")))
    return {
        "cohorts": COHORTS,
        "policies": POLICIES,
        "candidate_domain": "Complete original plan_feasible_command grid and order",
        "objective": "Maximum consecutive inspection dwell at D=0 among no-unsafe-entry candidates",
        "tie_rule": "First in original candidate order, using integer physics ticks",
        "feasibility": "No unsafe entry and at least the unchanged inspection dwell threshold at D=0",
        "deadline_s": min(DEADLINES),
        "inspection_dwell_s": SCAN_DWELL_S,
        "delays_s": [round(float(d), 8) for d in DELAY_GRID],
        "ctrl_hz": CTRL_FREQ_HZ,
        "physics_hz": PYB_FREQ_HZ,
        "simulation_seed": SIM_SEED,
        "recovery": False,
        "code_sha256": {str(path.relative_to(DIRECTORY)): hashlib.sha256(path.read_bytes()).hexdigest() for path in protected},
    }


def prepare(output):
    output.mkdir(parents=True, exist_ok=True)
    path = output / "protocol.json"
    current = protocol()
    if path.exists():
        assert json.loads(path.read_text()) == json.loads(json.dumps(current)), "Protocol or code changed"
    else:
        path.write_text(json.dumps(current, indent=2) + "\n")
    return digest(current)


def calibrate(output, cohort, protocol_hash, resume):
    for seed in COHORTS[cohort]:
        result_path = output / f"calibration_{seed}.json"
        if resume and result_path.exists():
            stored = json.loads(result_path.read_text())
            assert stored["protocol_hash"] == protocol_hash and stored["complete"]
            print(f"{cohort} seed={seed}: reused complete D=0 inventory", flush=True)
            continue
        original = generate_scenario(seed)
        started = time.perf_counter()
        records = []
        with (output / f"candidates_{seed}.jsonl").open("w") as stream:
            def record(rank, candidate, outcome):
                row = {"seed": seed, "rank": rank, **asdict(candidate), **outcome}
                records.append(row)
                stream.write(json.dumps(row) + "\n")
                stream.flush()
            maximum = plan_max_dwell_command(original, on_candidate=record)
        expected = list(command_candidates(original))
        assert len(records) == len(expected)
        completed = [r for r in records if r["safe_completion"]]
        first = completed[0] if completed else None
        assert (first is None) == (maximum is None)
        best = None
        if maximum is not None:
            best = next(r for r, c in zip(records, expected) if asdict(c) == asdict(maximum))
            safe = [r for r in records if not r["unsafe_entry"]]
            optimum = max(r["best_dwell_ticks"] for r in safe)
            assert best["best_dwell_ticks"] == optimum
            assert best["rank"] == next(r["rank"] for r in safe if r["best_dwell_ticks"] == optimum)
        selections = {}
        for policy, row in zip(POLICIES, (first, best)):
            selections[policy] = None if row is None else {
                "command": asdict(expected[row["rank"] - 1]),
                "rank": row["rank"],
                "d0_dwell_ticks": row["best_dwell_ticks"],
            }
        result = {
            "protocol_hash": protocol_hash, "complete": True,
            "seed": seed, "cohort": cohort, "generated_scenario": asdict(original),
            "candidate_count": len(records),
            "d0_safe_count": sum(not r["unsafe_entry"] for r in records),
            "d0_feasible_count": len(completed),
            "feasible": first is not None,
            "best_safe_dwell_ticks": max((r["best_dwell_ticks"] for r in records if not r["unsafe_entry"]), default=None),
            "selections": selections,
            "elapsed_s": time.perf_counter() - started,
        }
        temporary = result_path.with_suffix(".tmp")
        temporary.write_text(json.dumps(result, indent=2) + "\n")
        temporary.replace(result_path)
        print(
            f"{cohort} seed={seed}: candidates={len(records)} D0-feasible={len(completed)} "
            f"first/max dwell(ms)={first['best_dwell_ticks'] if first else 'N/A'}/"
            f"{best['best_dwell_ticks'] if best else 'N/A'}", flush=True,
        )


def evaluate(output, protocol_hash):
    frozen = []
    for cohort, seeds in COHORTS.items():
        for seed in seeds:
            result = json.loads((output / f"calibration_{seed}.json").read_text())
            assert result["complete"] and result["protocol_hash"] == protocol_hash
            frozen.append(result)
    selection_hash = digest(frozen)
    freeze_path = output / "frozen_selections.json"
    freeze_path.write_text(json.dumps({
        "protocol_hash": protocol_hash, "selection_hash": selection_hash,
        "calibrations": frozen,
    }, indent=2) + "\n")
    frozen_bytes = freeze_path.read_bytes()
    outcomes = []
    fields = [
        "cohort", "seed", "calibration_policy", "delay_s", "deadline_s",
        "classification", "inspection_completed", "unsafe_entry", "recovery_started", "safe_completion",
        "dispatch_step", "deadline_step", "d0_selection_dwell_ticks", "selected_rank",
        "d0_replay_dwell_ticks", "min_vx_after_ramp", "min_vx_during_inspection", "final_x",
        "protocol_hash", "selection_hash", *asdict(generate_scenario(0)),
    ]
    with (output / "outcomes.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for cohort, seeds in COHORTS.items():
            cohort_results = [r for r in frozen if r["cohort"] == cohort]
            feasible = sum(r["feasible"] for r in cohort_results)
            print(f"\n{cohort}: calibration feasible {feasible}/{len(seeds)} for BOTH policies", flush=True)
            print("D(s) | first A/B/C | max-dwell A/B/C | infeasible", flush=True)
            snapshots = {}
            for delay in DELAY_GRID:
                counts = {policy: {label: 0 for label in 'ABC'} for policy in POLICIES}
                for calibration in cohort_results:
                    if not calibration["feasible"]:
                        continue
                    seed = calibration["seed"]
                    for policy in POLICIES:
                        choice = calibration["selections"][policy]
                        scenario = Scenario(**choice["command"])
                        key = (seed, policy)
                        if key not in snapshots:
                            snapshots[key] = capture_approval(scenario)
                        snapshot = snapshots[key]
                        dispatch = round((snapshot.approval_time + float(delay)) * CTRL_FREQ_HZ)
                        deadline = round((snapshot.approval_time + min(DEADLINES)) * CTRL_FREQ_HZ)
                        physics_trace = [] if float(delay) == 0 else None
                        trace = [] if float(delay) == 0 else None
                        completed, unsafe, recovered, safe = run_actual(
                            scenario, snapshot, dispatch, 0, deadline, False,
                            trace=trace, physics_trace=physics_trace,
                        )
                        assert not recovered and safe == (completed and not unsafe)
                        label = 'C' if unsafe else 'A' if completed else 'B'
                        replay_ticks = min_vx = min_scan_vx = final_x = None
                        if physics_trace is not None:
                            from model.common import STATE_VX, STATE_X
                            from staleguard_benefit import in_inspection_region
                            replay_ticks = inspection_dwell_ticks(physics_trace, scenario)
                            assert replay_ticks == choice["d0_dwell_ticks"] and safe
                            min_vx = min(row[1][STATE_VX] for row in trace if row[0] / CTRL_FREQ_HZ >= ACCELERATION_SECONDS)
                            scan_velocities = [row[1][STATE_VX] for row in trace if in_inspection_region(row[1], scenario, inspection_x(scenario))]
                            min_scan_vx = min(scan_velocities) if scan_velocities else None
                            final_x = float(trace[-1][1][STATE_X])
                        row = {
                            "cohort": cohort, "seed": seed, "calibration_policy": policy,
                            "delay_s": round(float(delay), 8), "deadline_s": min(DEADLINES),
                            "classification": label, "inspection_completed": completed,
                            "unsafe_entry": unsafe, "recovery_started": recovered, "safe_completion": safe,
                            "dispatch_step": dispatch, "deadline_step": deadline,
                            "d0_selection_dwell_ticks": choice["d0_dwell_ticks"], "selected_rank": choice["rank"],
                            "d0_replay_dwell_ticks": replay_ticks, "min_vx_after_ramp": min_vx,
                            "min_vx_during_inspection": min_scan_vx, "final_x": final_x,
                            "protocol_hash": protocol_hash, "selection_hash": selection_hash,
                            **asdict(scenario),
                        }
                        writer.writerow(row)
                        outcomes.append(row)
                        counts[policy][label] += 1
                for policy in POLICIES:
                    assert sum(counts[policy].values()) == feasible
                stream.flush()
                first = '/'.join(str(counts[POLICIES[0]][c]) for c in 'ABC')
                maximum = '/'.join(str(counts[POLICIES[1]][c]) for c in 'ABC')
                print(f"{float(delay):.1f} | {first:>11} | {maximum:>15} | {len(seeds)-feasible}", flush=True)
    assert freeze_path.read_bytes() == frozen_bytes
    summary = {"protocol_hash": protocol_hash, "selection_hash": selection_hash, "cohorts": {}}
    for cohort, seeds in COHORTS.items():
        calibrations = [r for r in frozen if r["cohort"] == cohort]
        feasible = sum(r["feasible"] for r in calibrations)
        summary["cohorts"][cohort] = {
            "attempted": len(seeds), "feasible": feasible,
            "infeasible_seeds": [r["seed"] for r in calibrations if not r["feasible"]],
            "candidate_evaluations": sum(r["candidate_count"] for r in calibrations),
            "counts": [{"delay_s": round(float(d), 8), **{
                policy: {c: sum(r["cohort"] == cohort and r["calibration_policy"] == policy and r["delay_s"] == round(float(d), 8) and r["classification"] == c for r in outcomes) for c in 'ABC'}
                for policy in POLICIES}} for d in DELAY_GRID],
        }
    summary["evaluation_rollouts"] = len(outcomes)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"\nVerified frozen selections, per-execution identities and all A+B+C denominators. Executed {len(outcomes)} delay trials.", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("prepare", "calibrate", "evaluate", "all"), default="all")
    parser.add_argument("--cohort", choices=("original", "fresh", "all"), default="all")
    parser.add_argument("--output-dir", type=Path, default=DIRECTORY / "results" / "max_dwell_policy")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()
    protocol_hash = prepare(args.output_dir)
    if args.stage in {"calibrate", "all"}:
        cohorts = COHORTS if args.cohort == "all" else [args.cohort]
        for cohort in cohorts:
            calibrate(args.output_dir, cohort, protocol_hash, args.resume)
    if args.stage in {"evaluate", "all"}:
        evaluate(args.output_dir, protocol_hash)


if __name__ == "__main__":
    main()
