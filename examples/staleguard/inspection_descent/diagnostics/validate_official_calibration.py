"""Check saved scenarios, manual calibration, and simulation regressions.

Run from examples/staleguard with the scg Python environment. The pre-cleanup
source snapshot and previously measured max-dwell results must be available.
"""

import os
import sys
from pathlib import Path

# Limit library threads before importing NumPy, SCG, or the controller.
for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
                 "NUMEXPR_NUM_THREADS", "VECLIB_MAXIMUM_THREADS"):
    os.environ[variable] = "1"

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import argparse
import ast
import contextlib
import csv
from dataclasses import asdict
import fcntl
import hashlib
import importlib.util
import json
import resource
from unittest.mock import patch

import numpy as np

from experiment import Scenario, generate_scenario
import max_dwell_calibration as calibration
from mission import capture_approval
import staleguard_benefit as benefit


OUTPUT = ROOT / "results" / "official_calibration_validation"
REPORTS = OUTPUT / "saved_scenarios"
PREVIOUS = ROOT / "results" / "max_dwell_policy"
MAX_RSS_MB = 2048
MIN_AVAILABLE_MB = 2048


def check_memory():
    """Stop between simulations if the process or host memory budget is tight."""
    status = Path("/proc/self/status").read_text().splitlines()
    rss_mb = int(next(line.split()[1] for line in status if line.startswith("VmRSS:"))) / 1024
    memory = Path("/proc/meminfo").read_text().splitlines()
    available_mb = int(next(line.split()[1] for line in memory if line.startswith("MemAvailable:"))) / 1024
    if rss_mb > MAX_RSS_MB or available_mb < MIN_AVAILABLE_MB:
        raise RuntimeError(f"Memory budget reached: process={rss_mb:.0f} MiB, available={available_mb:.0f} MiB. Stop validation and free memory before retrying.")
    return rss_mb, available_mb


def calibrate_seed(seed):
    check_memory()
    expected = json.loads((PREVIOUS / f"calibration_{seed}.json").read_text())
    previous = [json.loads(line) for line in (PREVIOUS / f"candidates_{seed}.jsonl").read_text().splitlines()]
    original_plan = calibration.plan_max_dwell_command
    observed = 0

    def checked_plan(scenario, on_candidate=None):
        def check(rank, candidate, outcome):
            nonlocal observed
            row = previous[rank - 1]
            assert asdict(candidate) == {k: row[k] for k in asdict(candidate)}
            assert outcome == {k: row[k] for k in outcome}, (seed, rank)
            observed += 1
            rss, available = check_memory()
            if rank % 50 == 0:
                print(f"seed={seed} checked={rank}/{len(previous)} RSS={rss:.0f} MiB available={available:.0f} MiB", flush=True)
            if on_candidate is not None:
                on_candidate(rank, candidate, outcome)
        return original_plan(scenario, on_candidate=check)

    with patch.object(calibration, "plan_max_dwell_command", checked_plan):
        stored = calibration.calibrate_scenarios([seed])[str(seed)]
    saved = json.loads(benefit.CALIBRATION_FILE.read_text())[str(seed)]
    assert stored == saved
    assert observed == expected["candidate_count"]
    selected = expected["selections"]["max_d0_dwell"]
    result = {"seed": seed, "candidates": observed, "command": stored["command"],
              "dwell_ticks": None if selected is None else selected["d0_dwell_ticks"],
              "best_safe_dwell_ticks": stored["best_safe_dwell_ticks"]}
    (REPORTS / f"calibration_{seed}.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def validate_calibration(seeds):
    results = []
    for seed in seeds:
        result = calibrate_seed(seed)
        results.append(result)
        print(f"seed={result['seed']} candidates={result['candidates']} dwell_ticks={result['dwell_ticks']}", flush=True)
    if seeds != list(range(20)):
        print(f"PASS: requested seeds {seeds} match previously measured calibrations.", flush=True)
        return
    assert sum(r["candidates"] for r in results) == 2241
    assert [r["seed"] for r in results if r["command"] is None] == [16]
    (REPORTS / "calibrations.json").write_text(json.dumps(results, indent=2) + "\n")
    print("PASS: all 2,241 candidate outcomes/dwells and all 20 selections match previous executions; seed 16 remains infeasible.", flush=True)


def validate_saved_scenarios():
    saved = json.loads(benefit.CALIBRATION_FILE.read_text())
    assert list(saved) == [str(seed) for seed in range(20)]
    for seed in range(20):
        record = saved[str(seed)]
        previous = json.loads((PREVIOUS / f"calibration_{seed}.json").read_text())
        selected = previous["selections"]["max_d0_dwell"]
        assert record["generated_scenario"] == asdict(generate_scenario(seed))
        assert record["command"] == (None if selected is None else selected["command"])
        assert record["best_safe_dwell_ticks"] == previous["best_safe_dwell_ticks"]
    assert [seed for seed, record in saved.items() if record["command"] is None] == ["16"]
    print("PASS: all 20 saved scenarios and dwell measurements match prior executions; 19 feasible, seed 16 infeasible. No calibration simulations executed.", flush=True)


def before_module():
    path = OUTPUT / "before" / "staleguard_benefit.py"
    spec = importlib.util.spec_from_file_location("benefit_before_cleanup", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def trace_hash(trace, physics):
    result = hashlib.sha256()
    for row in trace:
        for value in row:
            result.update(np.asarray(value).tobytes())
    for tick, position in physics:
        result.update(np.asarray(tick).tobytes())
        result.update(np.asarray(position).tobytes())
    return result.hexdigest()


def validate_experiment():
    before = before_module()
    actual = benefit.run_actual
    records = []

    def checked_actual(*args, **kwargs):
        trace, physics, old_trace, old_physics = [], [], [], []
        check_memory()
        outcome = actual(*args, **kwargs, trace=trace, physics_trace=physics)
        old_outcome = before.run_actual(*args, **kwargs, trace=old_trace, physics_trace=old_physics)
        assert outcome == old_outcome
        new_hash = trace_hash(trace, physics)
        assert new_hash == trace_hash(old_trace, old_physics), asdict(args[0])
        scenario, snapshot, dispatch, recovery, deadline, recover = args
        records.append({"scenario": asdict(scenario), "dispatch_step": dispatch,
                        "recover_step": recovery, "deadline_step": deadline, "recover": bool(recover),
                        "outcome": outcome, "trace_sha256": new_hash})
        if len(records) % 38 == 0:
            rss, _ = check_memory()
            print(f"Matched {len(records)}/228 before/after executions; RSS={rss:.0f} MiB", file=sys.__stdout__, flush=True)
        check_memory()
        return outcome

    with patch.object(calibration, "plan_max_dwell_command", side_effect=AssertionError("Experiment must not calibrate")), \
         patch.object(benefit, "run_actual", checked_actual), (REPORTS / "official_experiment.log").open("w") as stream:
        with contextlib.redirect_stdout(stream):
            benefit.main(count=20)
    assert len(records) == 19 * len(benefit.PENDINGS) * len(benefit.DEADLINES) * 2
    with (REPORTS / "execution_regression.jsonl").open("w") as stream:
        for row in records:
            stream.write(json.dumps(row) + "\n")

    # Preserve the actual GUI branches without opening a window on this host.
    def gui_branches(path):
        tree = ast.parse(path.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "run_actual")
        return [ast.dump(n) for n in ast.walk(function) if isinstance(n, ast.If)
                and isinstance(n.test, ast.Name) and n.test.id == "gui"]
    assert gui_branches(ROOT / "staleguard_benefit.py") == gui_branches(OUTPUT / "before" / "staleguard_benefit.py")
    print(f"PASS: official 20-seed experiment; all {len(records)} executions match pre-cleanup outcomes, control states/references and physics positions exactly. Paired identities passed. GUI branches unchanged.", flush=True)


def validate_delay_grid():
    with (PREVIOUS / "outcomes.csv").open() as stream:
        previous = [r for r in csv.DictReader(stream) if r["calibration_policy"] == "max_d0_dwell"]
    snapshots = {}
    records = []
    saved = json.loads(benefit.CALIBRATION_FILE.read_text())
    for row in previous:
        check_memory()
        seed = int(row["seed"])
        command = Scenario(**saved[str(seed)]["command"])
        assert command is not None
        assert asdict(command) == {k: float(row[k]) for k in asdict(command)}
        if seed not in snapshots:
            snapshots[seed] = capture_approval(command)
        physics = [] if float(row["delay_s"]) == 0 else None
        outcome = benefit.run_actual(command, snapshots[seed], int(row["dispatch_step"]), 0,
                                     int(row["deadline_step"]), False, physics_trace=physics)
        assert outcome == tuple(row[k] == "True" for k in (
            "inspection_completed", "unsafe_entry", "recovery_started", "safe_completion"))
        if physics is not None:
            assert calibration.inspection_dwell_ticks(physics, command) == int(row["d0_selection_dwell_ticks"])
        records.append({"seed": seed, "delay_s": float(row["delay_s"]), "outcome": outcome})
        if len(records) % 50 == 0:
            print(f"Matched {len(records)}/304 saved delay outcomes", flush=True)
    assert len(records) == 304
    (REPORTS / "delay_regression.json").write_text(json.dumps(records, indent=2) + "\n")
    print("PASS: all 304 maximum-dwell No Recovery outcomes across the full delay grid match previous executions; D=0 replay dwells match.", flush=True)


def main():
    global MAX_RSS_MB, MIN_AVAILABLE_MB
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("calibrate", "saved", "experiment", "delays", "all"), default="all")
    parser.add_argument("--workers", type=int, choices=(1,), default=1, help="Only serial validation is supported.")
    parser.add_argument("--seeds", type=int, nargs="+", default=list(range(20)))
    parser.add_argument("--max-rss-mb", type=int, default=MAX_RSS_MB)
    parser.add_argument("--min-available-mb", type=int, default=MIN_AVAILABLE_MB)
    args = parser.parse_args()
    MAX_RSS_MB = args.max_rss_mb
    MIN_AVAILABLE_MB = args.min_available_mb
    REPORTS.mkdir(parents=True, exist_ok=True)
    check_memory()
    with (REPORTS / ".validation.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another validation run is active; wait for it to finish.")
        for stage, run in (("calibrate", lambda: validate_calibration(args.seeds)),
                           ("saved", validate_saved_scenarios), ("experiment", validate_experiment),
                           ("delays", validate_delay_grid)):
            if args.stage in (stage, "all"):
                run()
                rss, available = check_memory()
                print(f"Resources: one worker; RSS={rss:.0f} MiB; peak RSS={resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024:.0f} MiB; available={available:.0f} MiB", flush=True)


if __name__ == "__main__":
    main()
