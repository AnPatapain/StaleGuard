"""Supplementary B experiment: measured reception latency and short dispatches.

Prediction keeps the official 0.0--1.0 s grid. Only actual dispatch delays
change. Run predict once, then run (resumable), then report, in separate processes.
"""
import argparse
from dataclasses import asdict
import fcntl
import gc
import json
import math
from pathlib import Path
import resource
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_corrected_experiment_b import (
    ROOT, INPUTS, POLICIES, LIBC, b, check_memory, digest, load_inputs, trace_hash,
)
import numpy as np

OUTPUT = ROOT / "results/short_delay_latency"
OFFICIAL = ROOT / "results/corrected_experiment_b"
REPORT = ROOT / "result/short_delay_latency.json"
DELAY_STEPS = (0, 1, 2, 3, 4, 5)  # Actual dispatch: 0, 20, ..., 100 ms.


def check_preserved():
    path = OUTPUT / "preserved_files.json"
    if not path.exists():
        files = list(ROOT.glob("*.py")) + list((ROOT / "model").glob("*.py"))
        files += list((ROOT / "result").glob("*"))
        files += [b.CALIBRATION_FILE, INPUTS]
        for name in ("unknown_dispatch_delay", "corrected_experiment_b", "temporal_safety_correction"):
            files += list((ROOT / "results" / name).glob("*.json"))
            files += list((ROOT / "results" / name).glob("*.jsonl"))
        path.write_text(json.dumps({str(p.relative_to(ROOT)): digest(p) for p in files
                                    if p.is_file() and p != REPORT}, indent=2) + "\n")
    for name, expected in json.loads(path.read_text()).items():
        assert digest(ROOT / name) == expected, name


def predict():
    path = OUTPUT / "receptions.json"
    assert not path.exists(), "Preserve the measured timing pass"
    inputs = load_inputs()
    quad, pid = inputs["quad_params"], inputs["pid_params"]
    prior = {r["seed"]: r for r in json.loads((OFFICIAL / "receptions.json").read_text())}
    check_memory()
    b.predict(inputs["warmup"], b.LightweightPredictiveModel(quad, pid), delays=b.DELAY_GRID)
    rows = []
    for seed, snapshot in inputs["snapshots"].items():
        check_memory()
        prediction, elapsed = b.predict_at_reception(snapshot, quad, pid)
        assert prediction == prior[seed]["prediction"]
        first_ready = math.ceil(elapsed * b.CTRL_FREQ_HZ)
        rows.append(dict(seed=seed, prediction=prediction, computation_s=elapsed,
                         first_ready_control_step=first_ready))
        print(f"Seed {seed}: {elapsed*1000:.3f} ms; first usable control boundary "
              f"{first_ready/b.CTRL_FREQ_HZ:.2f} s", flush=True)
    assert len(rows) == 19
    path.write_text(json.dumps(rows, indent=2) + "\n")
    check_preserved()


def run():
    inputs = load_inputs()
    saved = json.loads(b.CALIBRATION_FILE.read_text())
    commands = {int(seed): b.Scenario(**r["command"]) for seed, r in saved.items() if r["command"] is not None}
    assert len(commands) == 19 and saved["16"]["command"] is None
    receptions = {r["seed"]: r for r in json.loads((OUTPUT / "receptions.json").read_text())}
    path = OUTPUT / "executions.jsonl"
    rows = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    completed = {(r["seed"], r["delay_steps"], r["policy"]): r for r in rows}
    expiry = b.ttl_expired_at_dispatch
    with path.open("a") as stream:
        for delay_steps in DELAY_STEPS:
            for seed, s in commands.items():
                snapshot = inputs["snapshots"][seed]
                received = round(snapshot.approval_time * b.CTRL_FREQ_HZ)
                dispatch = received + delay_steps
                assert dispatch == round((snapshot.approval_time + delay_steps/b.CTRL_FREQ_HZ) * b.CTRL_FREQ_HZ)
                deadline = round((snapshot.approval_time + b.MISSION_DEADLINE_S) * b.CTRL_FREQ_HZ)
                r = receptions[seed]
                prediction, elapsed = r["prediction"], r["computation_s"]
                ready = elapsed <= delay_steps / b.CTRL_FREQ_HZ
                assert ready == (delay_steps >= r["first_ready_control_step"])
                group = {}
                for policy in POLICIES:
                    key = seed, delay_steps, policy
                    if key in completed:
                        group[policy] = completed[key]
                        continue
                    check_memory()
                    trace, physics, checks = [], [], []
                    def checked_expiry(predicted, waited):
                        assert policy == "StaleGuard" and ready
                        assert waited == delay_steps and trace[-1][0] == dispatch
                        checks.append(dispatch)
                        return expiry(predicted, waited)
                    recover = policy == "Always-Recover" and received < dispatch
                    recovery_step = received if policy == "Always-Recover" else dispatch
                    kwargs = {"ttl_prediction": (prediction, elapsed)} if policy == "StaleGuard" else {}
                    with patch.object(b, "ttl_expired_at_dispatch", checked_expiry):
                        outcome = b.run_actual(s, snapshot, dispatch, recovery_step, deadline, recover,
                                               trace=trace, physics_trace=physics,
                                               recovery_mode="safety-only", **kwargs)
                    expected_recovery = ready and expiry(prediction, delay_steps) if policy == "StaleGuard" else recover
                    assert outcome[2] == expected_recovery
                    assert checks == ([dispatch] if policy == "StaleGuard" and ready else [])
                    assert outcome[3] == (outcome[0] and not outcome[1])
                    # The reference applied at the dispatch action proves no postponement.
                    assert trace[dispatch][2][b.STATE_Z] == s.usual_height
                    assert trace[dispatch+1][0] == dispatch+1
                    assert trace[dispatch+1][2][b.STATE_Z] == (s.usual_height if outcome[2] else s.low_height)
                    first_unsafe = None
                    state = snapshot.state.copy()
                    for tick, position in physics:
                        state[[b.STATE_X, b.STATE_Y, b.STATE_Z]] = position
                        if b.unsafe(state, snapshot):
                            first_unsafe = tick / b.PYB_FREQ_HZ
                            break
                    assert outcome[1] == (first_unsafe is not None)
                    row = dict(seed=seed, scenario=asdict(s), delay_steps=delay_steps,
                               delay_s=delay_steps / b.CTRL_FREQ_HZ, policy=policy,
                               dispatch_step=dispatch, deadline_step=deadline,
                               outcome=outcome, trace_sha256=trace_hash(trace, physics),
                               computation_s=elapsed if policy == "StaleGuard" else None,
                               prediction_ready=ready if policy == "StaleGuard" else None,
                               ttl_would_reject=bool(expiry(prediction, delay_steps)),
                               expiry_check_steps=checks,
                               original_command_step=None if outcome[2] else dispatch,
                               first_unsafe_time_s=first_unsafe,
                               worker_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024)
                    stream.write(json.dumps(row) + "\n"); stream.flush()
                    completed[key] = row
                    rows.append(row)
                    group[policy] = row
                    del trace, physics
                    gc.collect()
                    LIBC.malloc_trim(0)
                sg = group["StaleGuard"]
                reference = group["Always-Recover" if sg["outcome"][2] else "No Recovery"]
                assert tuple(sg["outcome"]) == tuple(reference["outcome"])
                assert sg["trace_sha256"] == reference["trace_sha256"]
                if not ready:
                    assert not sg["outcome"][2]  # Whole-flight NR match also excludes later cancellation.
            print(f"D={delay_steps/b.CTRL_FREQ_HZ:.2f}: {len(rows)}/342 flights saved; "
                  f"RSS={check_memory()[0]:.0f} MiB", flush=True)
    assert len(rows) == 342
    check_preserved()


def report():
    rows = [json.loads(line) for line in (OUTPUT / "executions.jsonl").read_text().splitlines()]
    receptions = json.loads((OUTPUT / "receptions.json").read_text())
    assert len(rows) == len({(r["seed"], r["delay_steps"], r["policy"]) for r in rows}) == 342
    times = np.array([r["computation_s"] * 1000 for r in receptions])
    timing = dict(samples=len(times), mean_ms=float(times.mean()), median_ms=float(np.median(times)),
                  p95_ms=float(np.percentile(times, 95)), min_ms=float(times.min()), max_ms=float(times.max()))
    per_delay, latency_cases, false_safe_cases, false_rejections = [], [], [], []
    for steps in DELAY_STEPS:
        block = [r for r in rows if r["delay_steps"] == steps]
        sg = [r for r in block if r["policy"] == "StaleGuard"]
        unavailable = [r for r in sg if not r["prediction_ready"]]
        attributable = [r for r in unavailable if r["outcome"][1] and r["ttl_would_reject"]]
        for r in attributable:
            assert r["first_unsafe_time_s"] > r["dispatch_step"] / b.CTRL_FREQ_HZ
            fallback = next(a for a in block if a["seed"] == r["seed"] and a["policy"] == "Always-Recover")
            assert not fallback["outcome"][1]
        latency_cases.extend(attributable)
        false_safe_cases.extend(r for r in sg if r["outcome"][1] and not r["ttl_would_reject"])
        nr = {r["seed"]: r for r in block if r["policy"] == "No Recovery"}
        false_rejections.extend(r for r in sg if r["outcome"][2] and not nr[r["seed"]]["outcome"][1])
        metrics = {p: dict(unsafe=sum(r["outcome"][1] for r in block if r["policy"] == p),
                           safe_completion=sum(r["outcome"][3] for r in block if r["policy"] == p),
                           inspection_completion=sum(r["outcome"][0] for r in block if r["policy"] == p),
                           fallbacks=sum(r["outcome"][2] for r in block if r["policy"] == p)) for p in POLICIES}
        per_delay.append(dict(delay_s=steps/b.CTRL_FREQ_HZ, N=19, prediction_timing=timing,
                              ready=19-len(unavailable), unavailable=len(unavailable),
                              original_executions_because_unavailable=len(unavailable),
                              unavailable_unsafe=sum(r["outcome"][1] for r in unavailable),
                              unavailable_unsafe_would_reject=len(attributable),
                              unavailable_unsafe_would_allow=sum(r["outcome"][1] and not r["ttl_would_reject"] for r in unavailable),
                              ready_unsafe=sum(r["outcome"][1] and r["prediction_ready"] for r in sg),
                              completion_harm_vs_no_recovery=sum(not r["outcome"][3] and nr[r["seed"]]["outcome"][3] for r in sg),
                              policies=metrics))
    official = [json.loads(line) for line in (OFFICIAL / "executions.jsonl").read_text().splitlines()]
    prior = {(r["seed"], r["delay_s"], r["policy"]): r for r in official}
    overlap = [r for r in rows if r["delay_steps"] in (0, 5)]
    changes = []
    for r in overlap:
        old = prior[r["seed"], r["delay_s"], r["policy"]]
        if r["policy"] != "StaleGuard":
            assert r["trace_sha256"] == old["trace_sha256"]
        if r["outcome"] != old["outcome"]:
            changes.append(dict(seed=r["seed"], delay_s=r["delay_s"], policy=r["policy"], before=old["outcome"], after=r["outcome"]))
    old_times = np.array([r["computation_s"]*1000 for r in json.loads((OFFICIAL / "receptions.json").read_text())])
    summary = dict(experiment="Supplementary Experiment B: short dispatch delays", considered_seeds=20,
                   evaluated=19, excluded_seeds=[16], policy_flights=342,
                   mission_deadline_after_approval_s=b.MISSION_DEADLINE_S,
                   prediction_delay_grid_s=list(b.DELAY_GRID), prediction_timing=timing,
                   reception_predictions=receptions, per_delay=per_delay,
                   unsafe_attributable_to_latency=latency_cases, unsafe_despite_allowing_ttl=false_safe_cases,
                   operational_false_rejections=false_rejections,
                   official_comparison=dict(common_delay_outcome_changes=changes, baseline_trace_matches=76,
                                            common_delay_trace_matches=sum(r["trace_sha256"] == prior[r["seed"], r["delay_s"], r["policy"]]["trace_sha256"] for r in overlap),
                                            prediction_mean_ms=float(old_times.mean()),
                                            prediction_median_ms=float(np.median(old_times)),
                                            prediction_p95_ms=float(np.percentile(old_times, 95))),
                   memory_limits_mib=dict(max_process_rss=2048, min_host_available=2048),
                   recorded_worker_peak_rss_mib=max(r["worker_peak_rss_mib"] for r in rows))
    REPORT.write_text(json.dumps(summary, indent=2) + "\n")
    check_preserved()
    print(json.dumps({k: v for k, v in summary.items() if k != "reception_predictions"}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("predict", "run", "report"), required=True)
    args = parser.parse_args()
    OUTPUT.mkdir(exist_ok=True)
    with (ROOT / "results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        check_preserved()
        {"predict": predict, "run": run, "report": report}[args.stage]()
