"""Serial checks and official results for the safety-only recovery protocol."""

import argparse
import ast
import contextlib
from dataclasses import asdict
import fcntl
import hashlib
import json
from pathlib import Path
import resource
import sys
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent)]

# Limits numerical library threads before NumPy and the simulator are imported.
from validate_official_calibration import check_memory, trace_hash
import numpy as np
import staleguard_benefit as benefit

OUTPUT = ROOT / "results" / "safety_only_recovery"
POLICIES = ("StaleGuard", "No Recovery", "Always-Recover")
OLD_CASES = ROOT / "results" / "always_recover" / "official_experiment.jsonl"
OLD_TRACES = ROOT / "results" / "recovery_realism_audit" / "replays.jsonl"


def check_preserved():
    for name, digest in json.loads((OUTPUT / "preserved_files.json").read_text()).items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, name
    saved = json.loads(benefit.CALIBRATION_FILE.read_text())
    assert [int(seed) for seed, row in saved.items() if row["command"] is None] == [16]
    before = ast.parse((OUTPUT / "before" / "staleguard_benefit.py").read_text())
    after = ast.parse(Path(benefit.__file__).read_text())
    for name in ("should_recover", "in_inspection_region", "plan_feasible_command"):
        old = next(n for n in before.body if isinstance(n, ast.FunctionDef) and n.name == name)
        new = next(n for n in after.body if isinstance(n, ast.FunctionDef) and n.name == name)
        assert ast.dump(old) == ast.dump(new), name
    # GUI code and physics monitoring must survive unchanged.
    old_run = next(n for n in before.body if isinstance(n, ast.FunctionDef) and n.name == "run_actual")
    new_run = next(n for n in after.body if isinstance(n, ast.FunctionDef) and n.name == "run_actual")
    for name in ("monitor", "monitored_physics"):
        old = next(n for n in old_run.body if isinstance(n, ast.FunctionDef) and n.name == name)
        new = next(n for n in new_run.body if isinstance(n, ast.FunctionDef) and n.name == name)
        assert ast.dump(old) == ast.dump(new), name
    def gui_nodes(function):
        return [ast.dump(n) for n in ast.walk(function) if isinstance(n, ast.If)
                and isinstance(n.test, ast.Name) and n.test.id == "gui"]
    assert gui_nodes(old_run) == gui_nodes(new_run)


def run_experiment(count, name, zero_decision_time=False):
    saved = json.loads(benefit.CALIBRATION_FILE.read_text())
    seed_by_command = {json.dumps(r["command"], sort_keys=True): int(seed)
                       for seed, r in saved.items() if r["command"] is not None}
    previous = {tuple(r["key"]): r for r in map(json.loads, OLD_TRACES.read_text().splitlines())}
    actual = benefit.run_actual
    records = []
    feasible = sum(saved[str(seed)]["command"] is not None for seed in range(count))

    with (OUTPUT / f"{name}.jsonl").open("w") as stream:
        def checked_actual(s, snapshot, dispatch, recovery, deadline, recover, **kwargs):
            check_memory()
            assert kwargs["recovery_mode"] == "safety-only"
            policy = POLICIES[len(records) % 3]
            seed = seed_by_command[json.dumps(asdict(s), sort_keys=True)]
            approval = round(snapshot.approval_time * benefit.CTRL_FREQ_HZ)
            assert deadline == round((snapshot.approval_time + benefit.MISSION_DEADLINE_S) * benefit.CTRL_FREQ_HZ)
            if policy == "No Recovery":
                assert not recover
            if policy == "Always-Recover":
                assert recovery == approval and bool(recover) == (approval < dispatch)
            if recover:
                assert approval <= recovery < dispatch
            trace, physics = [], []
            result = actual(s, snapshot, dispatch, recovery, deadline, recover,
                            trace=trace, physics_trace=physics, **kwargs)
            inspected, unsafe, started, safe = result
            categories = (safe, not inspected and not unsafe, unsafe)
            assert safe == (inspected and not unsafe)
            assert sum(categories) == 1
            assert started == bool(recover)
            assert np.array_equal(trace[approval][1], snapshot.state)
            digest = trace_hash(trace, physics)
            if not recover:
                old = previous[(seed, False, dispatch, deadline)]
                assert list(result) == old["outcome"]
                assert digest == old["trace_sha256"], (seed, dispatch, "original flight changed")
            else:
                nominal = benefit.make_forward_trajectory(
                    SimpleNamespace(X_GOAL=np.zeros((deadline + 1, 12)), CTRL_FREQ=benefit.CTRL_FREQ_HZ), s)
                references = np.array([entry[2] for entry in trace[1:]])
                assert np.array_equal(references, nominal[:len(references)])
            record = dict(seed=seed, policy=policy, scenario=asdict(s), snapshot_id=id(snapshot),
                          delay_s=(dispatch - approval) / benefit.CTRL_FREQ_HZ,
                          dispatch_step=dispatch, recover_step=recovery, deadline_step=deadline,
                          recover=bool(recover), outcome=result, trace_sha256=digest,
                          categories=categories, fallback_safe=bool(started and not unsafe))
            stream.write(json.dumps(record) + "\n")
            stream.flush()
            records.append(record)
            if len(records) % (3 * feasible) == 0:
                print(f"Executed {len(records)}/{3 * feasible * len(benefit.DELAY_GRID)} policy flights; RSS={check_memory()[0]:.0f} MiB", file=sys.__stdout__, flush=True)
            return result

        clock = patch.object(benefit, "time", SimpleNamespace(perf_counter=lambda: 0.0)) if zero_decision_time else contextlib.nullcontext()
        with clock, patch.object(benefit, "run_actual", checked_actual):
            with (OUTPUT / f"{name}.log").open("w") as output, contextlib.redirect_stdout(output):
                benefit.main(count=count)

    assert len(records) == 3 * feasible * len(benefit.DELAY_GRID)
    same_time_matches = 0
    for index in range(0, len(records), 3):
        sg, nr, always = records[index:index+3]
        assert sg["scenario"] == nr["scenario"] == always["scenario"]
        assert sg["snapshot_id"] == nr["snapshot_id"] == always["snapshot_id"]
        assert sg["dispatch_step"] == nr["dispatch_step"] == always["dispatch_step"]
        if sg["delay_s"] == 0:
            assert not any(r["recover"] for r in (sg, nr, always))
            assert sg["trace_sha256"] == nr["trace_sha256"] == always["trace_sha256"]
        if sg["recover"] and always["recover"] and sg["recover_step"] == always["recover_step"]:
            assert sg["trace_sha256"] == always["trace_sha256"]
            same_time_matches += 1
    if zero_decision_time:
        assert same_time_matches > 0

    metrics = []
    log = (OUTPUT / f"{name}.log").read_text()
    blocks = log.split("\nMISSION OUTCOMES")[1:]
    assert len(blocks) == len(benefit.DELAY_GRID)
    for delay, block in zip(benefit.DELAY_GRID, blocks):
        rows = [r for r in records if abs(r["delay_s"] - float(delay)) < 1e-9]
        totals = {}
        for policy in POLICIES:
            group = [r for r in rows if r["policy"] == policy]
            totals[policy] = {name: sum(r["categories"][j] for r in group)
                              for j, name in enumerate(("safe_completion", "safe_non_completion", "unsafe"))}
            totals[policy]["fallbacks"] = sum(r["recover"] for r in group)
            totals[policy]["fallback_safe"] = sum(r["fallback_safe"] for r in group)
            assert sum(totals[policy][k] for k in ("safe_completion", "safe_non_completion", "unsafe")) == feasible
        for label, key in (("Safe mission completion", "safe_completion"),
                           ("Safe mission non-completion", "safe_non_completion"),
                           ("Unsafe execution", "unsafe"), ("Fallback triggered", "fallbacks")):
            expected = f"{label:<27}" + "".join(f"{benefit.format_count(totals[p][key], feasible):>17}" for p in ("No Recovery", "Always-Recover", "StaleGuard"))
            assert expected in block, (delay, label)
        rates = [benefit.format_count(totals[p]["fallback_safe"], totals[p]["fallbacks"])
                 if totals[p]["fallbacks"] else "N/A (0/0)" for p in ("No Recovery", "Always-Recover", "StaleGuard")]
        assert f"{'Fallback safe (no entry)':<27}" + "".join(f"{r:>17}" for r in rates) in block
        by_policy = {p: {r["seed"]: r for r in rows if r["policy"] == p} for p in POLICIES}
        paired = {}
        for other in ("No Recovery", "Always-Recover"):
            sg_only = sum(sg["outcome"][3] and not by_policy[other][seed]["outcome"][3] for seed, sg in by_policy["StaleGuard"].items())
            other_only = sum(not sg["outcome"][3] and by_policy[other][seed]["outcome"][3] for seed, sg in by_policy["StaleGuard"].items())
            difference = totals["StaleGuard"]["safe_completion"] - totals[other]["safe_completion"]
            assert difference == sg_only - other_only
            paired[other] = dict(staleguard_only=sg_only, other_only=other_only, difference_pp=100*difference/feasible)
        metrics.append(dict(delay_s=float(delay), N=feasible, policies=totals, paired=paired))
    if count == 20:
        assert "Calibration feasible: 19/20" in log and "Infeasible seeds: [16]" in log
    old = {(r["seed"], r["policy"], r["delay_s"]): r for r in map(json.loads, OLD_CASES.read_text().splitlines())}
    decision_changes = sum(r["recover"] != old[(r["seed"], r["policy"], r["delay_s"])]["recover"] for r in records)
    summary = dict(policy_executions=len(records), original_command_trace_matches=sum(not r["recover"] for r in records),
                   same_time_fallback_trace_matches=same_time_matches,
                   recovery_decisions_different_from_saved_run=decision_changes,
                   controlled_zero_decision_time=zero_decision_time,
                   peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
                   metrics=metrics)
    (OUTPUT / f"{name}_checks.json").write_text(json.dumps(summary, indent=2) + "\n")
    check_preserved()
    print(f"PASS: {len(records)} policy flights, individual outcome categories, original-command traces, report values, and paired comparisons; historical files unchanged.", flush=True)


def historical_regression():
    previous = {tuple(r["key"]): r for r in map(json.loads, OLD_TRACES.read_text().splitlines())}
    cases = {}
    for row in map(json.loads, OLD_CASES.read_text().splitlines()):
        if row["recover"]:
            cases.setdefault((row["seed"], True, row["recover_step"], row["deadline_step"]), row)
    records = []
    snapshots = {}
    for key, row in cases.items():
        check_memory()
        s = benefit.Scenario(**row["scenario"])
        if row["seed"] not in snapshots:
            snapshots[row["seed"]] = benefit.capture_approval(s)
        trace, physics = [], []
        result = benefit.run_actual(s, snapshots[row["seed"]], row["dispatch_step"], row["recover_step"],
                                    row["deadline_step"], True, recovery_mode="mission-retry", trace=trace, physics_trace=physics)
        assert list(result) == row["outcome"]
        assert trace_hash(trace, physics) == previous[key]["trace_sha256"]
        records.append(dict(key=key, outcome=result, trace_sha256=previous[key]["trace_sha256"]))
    assert len(records) == 43
    (OUTPUT / "mission_retry_regression.json").write_text(json.dumps(records, indent=2) + "\n")
    check_preserved()
    print("PASS: all 43 historical recovery flights reproduce their original complete traces and outcomes.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("small", "experiment", "historical"), default="small")
    args = parser.parse_args()
    check_preserved()
    with (ROOT / "results" / "always_recover" / ".validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if args.stage == "historical":
            historical_regression()
        else:
            run_experiment(1 if args.stage == "small" else 20,
                           "small" if args.stage == "small" else "official_experiment",
                           zero_decision_time=args.stage == "small")
