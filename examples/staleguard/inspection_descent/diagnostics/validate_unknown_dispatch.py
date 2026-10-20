"""Serial timing checks and Experiment B; historical experiment logs stay intact."""
import argparse
import contextlib
from dataclasses import asdict
import fcntl
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
import resource
import sys
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent)]
from validate_official_calibration import check_memory, trace_hash
import staleguard_benefit as b
from experiment import classify_delay_results

OUTPUT = ROOT / "results/unknown_dispatch_delay"
POLICIES = ("StaleGuard", "No Recovery", "Always-Recover")


def check_preserved():
    for name, digest in json.loads((OUTPUT / "preserved_files.json").read_text()).items():
        assert hashlib.sha256((ROOT / name).read_bytes()).hexdigest() == digest, name


def execute(module, s, snapshot, dispatch, recovery, recover, **kwargs):
    check_memory()
    trace, physics = [], []
    result = module.run_actual(s, snapshot, dispatch, recovery,
                              round((s.approval_time + b.MISSION_DEADLINE_S) * b.CTRL_FREQ_HZ),
                              recover, trace=trace, physics_trace=physics, **kwargs)
    assert result[3] == (result[0] and not result[1])
    assert sum((result[3], not result[0] and not result[1], result[1])) == 1
    return result, trace_hash(trace, physics)


def small_checks():
    # Every classification uses the same conservative first-unsafe rule as A.
    sequences = [["SAFE"] * 11, ["UNSAFE"] + ["SAFE"] * 10,
                 ["SAFE"] * 6 + ["UNSAFE"] * 5,
                 ["SAFE", "UNSAFE"] + ["SAFE"] * 9]
    predictions = [classify_delay_results(b.DELAY_GRID, r) for r in sequences]
    predictions.append(classify_delay_results(b.DELAY_GRID, None, nominal_safe=False))
    for predicted in predictions:
        for delay in b.DELAY_GRID:
            assert b.ttl_expired_at_dispatch(predicted, round(float(delay) * b.CTRL_FREQ_HZ)) == b.should_recover(predicted, delay)
    assert tuple(inspect.signature(b.predict_at_reception).parameters) == ("snapshot", "quad_params", "pid_params")
    assert b.ttl_expired_at_dispatch(predictions[3], 10)  # Later SAFE cannot reopen validity.

    saved = json.loads(b.CALIBRATION_FILE.read_text())
    s = b.Scenario(**saved["0"]["command"])
    snapshot = b.capture_approval(s)
    finite = {"classification": "FINITE_TTL", "first_bad_delay": 0.5}
    records = []
    for delay, compute_s, prediction, cancelled in (
        (0.5, 0.04, finite, True),
        (0.5, 0.5, finite, True),  # Available exactly at dispatch.
        (0.5, 0.52, None, False),  # Unavailable result must never be read.
        (0.0, 0.02, None, False),
        (0.6, 0.04, {"classification": "FINITE_TTL", "first_bad_delay": float(b.DELAY_GRID[6])}, True),
    ):
        dispatch = round((snapshot.approval_time + delay) * b.CTRL_FREQ_HZ)
        trace, physics, calls = [], [], []
        original_check = b.ttl_expired_at_dispatch
        def checked_expiry(predicted, waited_steps):
            assert trace[-1][0] == dispatch  # Check is inside the actual dispatch event.
            assert waited_steps == dispatch - round(snapshot.approval_time * b.CTRL_FREQ_HZ)
            calls.append(waited_steps)
            return original_check(predicted, waited_steps)
        with patch.object(b, "ttl_expired_at_dispatch", checked_expiry):
            result = b.run_actual(s, snapshot, dispatch, dispatch,
                                  round((s.approval_time + b.MISSION_DEADLINE_S) * b.CTRL_FREQ_HZ),
                                  False, ttl_prediction=(prediction, compute_s), trace=trace, physics_trace=physics)
        assert result[2] == cancelled
        assert len(calls) == int(compute_s <= delay)
        reference, digest = execute(b, s, snapshot, dispatch, dispatch, cancelled)
        assert result == reference and trace_hash(trace, physics) == digest
        records.append(dict(delay_s=delay, computation_s=compute_s, cancelled=cancelled, outcome=result))
    try:
        execute(b, s, snapshot, 25, 25, False, recovery_mode="mission-retry", ttl_prediction=(finite, 0.04))
    except ValueError:
        pass
    else:
        raise AssertionError("B must refuse the historical recovery variant")

    # Execute the previous and new A runners with the same controlled timing.
    path = OUTPUT / "before/staleguard_benefit.py"
    spec = importlib.util.spec_from_file_location("before_experiment_b", path)
    before = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(before)
    before.CALIBRATION_FILE = b.CALIBRATION_FILE
    comparisons = []
    for module, name in ((before, "A_before"), (b, "A_after")):
        actual = module.run_actual
        flights = []
        def recorded(*args, **kwargs):
            check_memory()
            trace, physics = [], []
            result = actual(*args, **kwargs, trace=trace, physics_trace=physics)
            flights.append((result, trace_hash(trace, physics)))
            return result
        with patch.object(module, "run_actual", recorded), patch.object(module, "time", SimpleNamespace(perf_counter=lambda: 0.0)):
            with (OUTPUT / f"{name}.log").open("w") as stream, contextlib.redirect_stdout(stream):
                module.main(count=1)
        comparisons.append(flights)
    assert comparisons[0] == comparisons[1] and len(comparisons[0]) == 33
    assert (OUTPUT / "A_before.log").read_bytes() == (OUTPUT / "A_after.log").read_bytes()
    old = execute(before, s, snapshot, 25, 0, True, recovery_mode="mission-retry")
    new = execute(b, s, snapshot, 25, 0, True, recovery_mode="mission-retry")
    assert old == new
    (OUTPUT / "small_checks.json").write_text(json.dumps(dict(
        classification_comparisons=55, timing_cases=records,
        A_trace_matches=33, A_terminal_output_identical=True, historical_recovery_trace_match=True), indent=2) + "\n")
    check_preserved()
    print("PASS: TTL classifications/boundaries, late/exact-time prediction, D=0, common fallback traces, 33 unchanged A flights and output, and historical recovery.", flush=True)


def full_experiment():
    saved = json.loads(b.CALIBRATION_FILE.read_text())
    seed_by_command = {json.dumps(r["command"], sort_keys=True): int(seed)
                       for seed, r in saved.items() if r["command"] is not None}
    previous = {(r["seed"], r["policy"], r["delay_s"]): r for r in
                map(json.loads, (ROOT / "results/safety_only_recovery/official_experiment.jsonl").read_text().splitlines())}
    actual = b.run_actual
    reception = b.predict_at_reception
    reception_records, rows = [], []

    def recorded_reception(snapshot, quad_params, pid_params):
        assert not rows  # Every reception prediction precedes all scheduler cases.
        result = reception(snapshot, quad_params, pid_params)
        reception_records.append(dict(prediction=result[0], computation_s=result[1]))
        return result

    with (OUTPUT / "executions.jsonl").open("w") as stream:
        def recorded(s, snapshot, dispatch, recovery, deadline, recover, **kwargs):
            check_memory()
            policy = POLICIES[len(rows) % 3]
            seed = seed_by_command[json.dumps(asdict(s), sort_keys=True)]
            approval = round(snapshot.approval_time * b.CTRL_FREQ_HZ)
            waited_steps = dispatch - approval
            assert deadline == round((snapshot.approval_time + b.MISSION_DEADLINE_S) * b.CTRL_FREQ_HZ)
            trace, physics = [], []
            ttl = kwargs.get("ttl_prediction")
            if policy == "StaleGuard":
                assert len(reception_records) == 19 and ttl is not None and not recover
                assert kwargs["recovery_mode"] == "safety-only"
            else:
                assert ttl is None
            result = actual(s, snapshot, dispatch, recovery, deadline, recover,
                            trace=trace, physics_trace=physics, **kwargs)
            digest = trace_hash(trace, physics)
            category = (result[3], not result[0] and not result[1], result[1])
            assert sum(category) == 1 and result[3] == (result[0] and not result[1])
            ready = ttl is not None and ttl[1] <= waited_steps / b.CTRL_FREQ_HZ
            if ttl is not None:
                assert result[2] == bool(ready and b.ttl_expired_at_dispatch(ttl[0], waited_steps))
            row = dict(seed=seed, policy=policy, snapshot_id=id(snapshot), scenario=asdict(s),
                       delay_s=waited_steps / b.CTRL_FREQ_HZ, dispatch_step=dispatch,
                       recovery_step=dispatch if ttl is not None and result[2] else recovery,
                       deadline_step=deadline, outcome=result, categories=category,
                       trace_sha256=digest, prediction=ttl[0] if ttl else None,
                       computation_s=ttl[1] if ttl else None, prediction_ready=bool(ready))
            old = previous[(seed, policy, row["delay_s"])]
            reference_policy = policy
            if policy == "StaleGuard":
                reference_policy = "Always-Recover" if result[2] else "No Recovery"
            reference = previous[(seed, reference_policy, row["delay_s"])]
            assert list(result) == reference["outcome"]
            assert digest == reference["trace_sha256"]
            rows.append(row)
            stream.write(json.dumps(row) + "\n"); stream.flush()
            if len(rows) % 57 == 0:
                print(f"Executed {len(rows)}/627 B policy flights; RSS={check_memory()[0]:.0f} MiB", file=sys.__stdout__, flush=True)
            return result

        with patch.object(b, "run_actual", recorded), patch.object(b, "predict_at_reception", recorded_reception):
            with (ROOT / "result/baseline_unknown_dispatch.log").open("w") as output, contextlib.redirect_stdout(output):
                b.main(count=20, experiment="B")
    assert len(rows) == 627 and len(reception_records) == 19
    metrics = []
    for start in range(0, len(rows), 3):
        sg, nr, ar = rows[start:start+3]
        assert sg["snapshot_id"] == nr["snapshot_id"] == ar["snapshot_id"]
        assert sg["scenario"] == nr["scenario"] == ar["scenario"]
        assert sg["dispatch_step"] == nr["dispatch_step"] == ar["dispatch_step"]
        if sg["delay_s"] == 0:
            assert sg["trace_sha256"] == nr["trace_sha256"] == ar["trace_sha256"]
            assert not any(r["outcome"][2] for r in (sg, nr, ar))
    for delay in b.DELAY_GRID:
        block = [r for r in rows if abs(r["delay_s"] - float(delay)) < 1e-9]
        totals = {p: {k: sum(r["categories"][j] for r in block if r["policy"] == p)
                      for j, k in enumerate(("safe_completion", "safe_non_completion", "unsafe"))} for p in POLICIES}
        for p in POLICIES:
            totals[p]["recoveries"] = sum(r["outcome"][2] for r in block if r["policy"] == p)
            totals[p]["recovery_safe"] = sum(r["outcome"][2] and not r["outcome"][1] for r in block if r["policy"] == p)
            assert sum(totals[p][k] for k in ("safe_completion", "safe_non_completion", "unsafe")) == 19
        late = sum(not r["prediction_ready"] for r in block if r["policy"] == "StaleGuard")
        metrics.append(dict(delay_s=float(delay), N=19, policies=totals, prediction_not_ready=late))
    summary = dict(experiment="B", considered=20, feasible=19, infeasible_seeds=[16],
                   policy_executions=len(rows), reception_predictions=reception_records,
                   baseline_trace_matches=418,
                   same_outcomes_as_A=sum(list(r["outcome"]) == previous[(r["seed"],r["policy"],r["delay_s"])]["outcome"] for r in rows),
                   peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024, metrics=metrics)
    (OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    check_preserved()
    print("PASS: 627 B flights; reception has no actual D; dispatch gating, readiness, baseline traces, categories, and original artifacts checked.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("small", "experiment"), default="small")
    args = parser.parse_args()
    check_preserved()
    with (ROOT / "results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        {"small": small_checks, "experiment": full_experiment}[args.stage]()
