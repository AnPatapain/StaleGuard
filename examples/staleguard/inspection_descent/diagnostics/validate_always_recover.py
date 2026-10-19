"""Serial regression checks and measured results for Always-Recover only."""

import argparse
import ast
import contextlib
from dataclasses import asdict
import fcntl
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent)]

# This import limits library threads before importing NumPy or SCG.
from validate_official_calibration import check_memory, trace_hash
import staleguard_benefit as benefit


OUTPUT = ROOT / "results" / "always_recover"
POLICIES = ("StaleGuard", "No Recovery", "Always-Recover")


def run_recorded(module, count, policies, name, zero_decision_time=False):
    saved = json.loads(benefit.CALIBRATION_FILE.read_text())
    seed_by_command = {
        json.dumps(row["command"], sort_keys=True): int(seed)
        for seed, row in saved.items() if row["command"] is not None
    }
    feasible = sum(saved[str(seed)]["command"] is not None for seed in range(count))
    expected = feasible * len(benefit.DELAY_GRID) * len(policies)
    actual = module.run_actual
    records = []

    def checked_actual(s, snapshot, dispatch, recovery, deadline, recover):
        check_memory()
        policy = policies[len(records) % len(policies)]
        approval = round(snapshot.approval_time * benefit.CTRL_FREQ_HZ)
        assert deadline == round((snapshot.approval_time + benefit.MISSION_DEADLINE_S) * benefit.CTRL_FREQ_HZ)
        if policy == "Always-Recover":
            assert recovery == approval
            assert bool(recover) == (approval < dispatch)
        if policy == "No Recovery":
            assert not recover
        # Small tests record every flight. The full run records Always-Recover
        # traces to check that immediate recovery is identical across delays.
        traced = zero_decision_time or policy == "Always-Recover"
        trace, physics = ([] if traced else None), ([] if traced else None)
        result = actual(s, snapshot, dispatch, recovery, deadline, recover,
                        trace=trace, physics_trace=physics)
        assert result[3] == (result[0] and not result[1])
        if policy == "Always-Recover":
            assert result[2] == (approval < dispatch)
        records.append({
            "seed": seed_by_command[json.dumps(asdict(s), sort_keys=True)],
            "policy": policy, "scenario": asdict(s),
            "snapshot_id": id(snapshot),
            "delay_s": (dispatch - approval) / benefit.CTRL_FREQ_HZ,
            "dispatch_step": dispatch, "recover_step": recovery,
            "deadline_step": deadline, "recover": bool(recover),
            "outcome": result,
            "trace_sha256": trace_hash(trace, physics) if traced else None,
        })
        if count > 1 and len(records) % (feasible * len(policies)) == 0:
            print(f"Completed {len(records)}/{expected} policy executions; RSS={check_memory()[0]:.0f} MiB", file=sys.__stdout__, flush=True)
        return result

    # Only the small controlled regression uses zero StaleGuard compute time,
    # so its recovery can start at exactly the same time as Always-Recover.
    clock = patch.object(module, "time", SimpleNamespace(perf_counter=lambda: 0.0)) if zero_decision_time else contextlib.nullcontext()
    with clock, patch.object(module, "run_actual", checked_actual):
        with (OUTPUT / f"{name}.log").open("w") as stream, contextlib.redirect_stdout(stream):
            module.main(count=count)
    assert len(records) == expected
    for start in range(0, len(records), len(policies)):
        group = records[start:start + len(policies)]
        assert len({row["snapshot_id"] for row in group}) == 1
        assert all(row["scenario"] == group[0]["scenario"] for row in group)
        assert all(row["dispatch_step"] == group[0]["dispatch_step"] for row in group)
    (OUTPUT / f"{name}.jsonl").write_text("".join(json.dumps(row) + "\n" for row in records))
    return records


def small_regression():
    path = OUTPUT / "before" / "staleguard_benefit.py"
    spec = importlib.util.spec_from_file_location("benefit_before_always_recover", path)
    before = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(before)
    before.CALIBRATION_FILE = benefit.CALIBRATION_FILE
    old_functions = {n.name: ast.dump(n) for n in ast.parse(path.read_text()).body if isinstance(n, ast.FunctionDef)}
    new_functions = {n.name: ast.dump(n) for n in ast.parse(Path(benefit.__file__).read_text()).body if isinstance(n, ast.FunctionDef)}
    for name in ("run_actual", "should_recover", "in_inspection_region"):
        assert old_functions[name] == new_functions[name]
    old = run_recorded(before, 1, POLICIES[:2], "small_before", zero_decision_time=True)
    new = run_recorded(benefit, 1, POLICIES, "small_after", zero_decision_time=True)
    equal_recoveries = 0
    for i in range(len(benefit.DELAY_GRID)):
        sg, nr, always = new[i*3:i*3+3]
        for previous, current in zip(old[i*2:i*2+2], (sg, nr)):
            assert previous["outcome"] == current["outcome"]
            assert previous["trace_sha256"] == current["trace_sha256"]
        if always["delay_s"] == 0:
            assert not always["outcome"][2]
            assert sg["trace_sha256"] == nr["trace_sha256"] == always["trace_sha256"]
        if sg["outcome"][2]:
            assert sg["recover_step"] == always["recover_step"]
            assert sg["outcome"] == always["outcome"]
            assert sg["trace_sha256"] == always["trace_sha256"]
            equal_recoveries += 1
    assert equal_recoveries > 0
    summary = {"executed_flights": len(old) + len(new),
               "existing_policy_trace_matches": len(old),
               "equal_time_recovery_trace_matches": equal_recoveries,
               "d0_three_policy_trace_match": True,
               "controlled_staleguard_compute_time_s": 0.0}
    (OUTPUT / "small_checks.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"PASS: {len(old)} existing-policy flight traces unchanged; {equal_recoveries} equal-time recovery traces identical; all three policies identical at D=0.", flush=True)


def full_experiment():
    rows = run_recorded(benefit, 20, POLICIES, "official_experiment")
    path = ROOT / "results" / "official_calibration_validation" / "delay_grid_0_to_1" / "benefit_executions.jsonl"
    previous = [json.loads(line) for line in path.read_text().splitlines()]
    previous_by_key = {
        (json.dumps(row["scenario"], sort_keys=True), row["delay_s"], POLICIES[i % 2]): row
        for i, row in enumerate(previous)
    }
    matched = 0
    immediate_traces = {}
    for row in rows:
        if row["policy"] != "Always-Recover":
            old = previous_by_key[(json.dumps(row["scenario"], sort_keys=True), row["delay_s"], row["policy"])]
            assert row["outcome"] == tuple(old["outcome"]), (row, old)
            assert row["recover"] == old["recover"]
            matched += 1
        elif row["delay_s"] > 0:
            seed = row["seed"]
            if seed in immediate_traces:
                assert row["trace_sha256"] == immediate_traces[seed]
            immediate_traces[seed] = row["trace_sha256"]
    assert matched == 418 and len(immediate_traces) == 19
    metrics = []
    for i, delay in enumerate(benefit.DELAY_GRID):
        block = rows[i*57:(i+1)*57]
        policy_rows = {name: [row for row in block if row["policy"] == name] for name in POLICIES}
        totals = {name: dict(zip(("inspection", "unsafe", "recoveries", "safe"),
                                (sum(row["outcome"][j] for row in group) for j in range(4))))
                  for name, group in policy_rows.items()}
        sg, nr, always = (policy_rows[name] for name in POLICIES)
        sg_only = sum(s["outcome"][3] and not a["outcome"][3] for s, a in zip(sg, always))
        always_only = sum(a["outcome"][3] and not s["outcome"][3] for s, a in zip(sg, always))
        gain = sum(s["outcome"][2] and s["outcome"][3] and not n["outcome"][3] for s, n in zip(sg, nr))
        harm = sum(s["outcome"][2] and not s["outcome"][3] and n["outcome"][3] for s, n in zip(sg, nr))
        assert sg_only - always_only == totals["StaleGuard"]["safe"] - totals["Always-Recover"]["safe"]
        assert gain - harm == totals["StaleGuard"]["safe"] - totals["No Recovery"]["safe"]
        if float(delay) == 0:
            assert all(s["outcome"] == n["outcome"] == a["outcome"] for s, n, a in zip(sg, nr, always))
        metrics.append({"delay_s": float(delay), "N": 19, "policies": totals,
                        "staleguard_only_successes": sg_only, "always_only_successes": always_only,
                        "recovery_benefit": gain, "recovery_harm": harm,
                        "safe_difference_pp": 100 * (sg_only - always_only) / 19,
                        "recovery_frequency_difference_pp": 100 * (totals["StaleGuard"]["recoveries"] - totals["Always-Recover"]["recoveries"]) / 19})
    for name, digest in json.loads((OUTPUT / "preserved_files.json").read_text()).items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, name
    summary = {"considered_seeds": list(range(20)), "feasible": 19, "infeasible_seeds": [16],
               "policy_executions": len(rows), "existing_outcomes_unchanged": matched,
               "identical_immediate_recovery_across_positive_delays": 19,
               "metrics": metrics}
    (OUTPUT / "experiment_checks.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(f"PASS: all {len(rows)} three-policy executions; all {matched} existing StaleGuard/No Recovery outcomes unchanged; paired assertions passed; saved commands and mission sources unchanged.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("small", "experiment"), default="small")
    args = parser.parse_args()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    check_memory()
    with (OUTPUT / ".validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        {"small": small_regression, "experiment": full_experiment}[args.stage]()
