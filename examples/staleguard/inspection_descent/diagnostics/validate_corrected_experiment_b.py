"""Revalidate B serially, preserving measured reception times and old results.

Run --stage predict once, then --stage run. The latter can resume completed
flights. Separate processes keep the JIT compiler out of simulator memory.
Only snapshot/parameter loading and already measured predictions are substituted
in the official main(); dispatch, flight, aggregation, and reporting run normally.
"""
import argparse
import contextlib
import ctypes
from dataclasses import asdict
import fcntl
import gc
import hashlib
import json
from pathlib import Path
import pickle
import resource
import sys
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(Path(__file__).resolve().parent)]
from validate_official_calibration import check_memory, trace_hash
import staleguard_benefit as b
from experiment import MEASURE_SECONDS

OUTPUT = ROOT / "results/corrected_experiment_b"
OLD = ROOT / "results/unknown_dispatch_delay"
INPUTS = ROOT / "results/temporal_safety_correction/prediction_inputs.pkl"
LOG = ROOT / "result/baseline_unknown_dispatch_swept.log"
POLICIES = ("StaleGuard", "No Recovery", "Always-Recover")
LIBC = ctypes.CDLL(None)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_preserved():
    path = OUTPUT / "preserved_files.json"
    if not path.exists():
        files = list(ROOT.glob("*.py")) + list((ROOT / "model").glob("*.py"))
        files += list(OLD.rglob("*.json")) + list(OLD.rglob("*.jsonl"))
        files += [b.CALIBRATION_FILE, INPUTS,
                  ROOT / "result/baseline_unknown_dispatch.log",
                  ROOT / "results/temporal_safety_correction/ground_truth_results.json"]
        path.write_text(json.dumps({str(p.relative_to(ROOT)): digest(p) for p in files}, indent=2) + "\n")
    for name, expected in json.loads(path.read_text()).items():
        assert digest(ROOT / name) == expected, name


def load_inputs():
    with INPUTS.open("rb") as stream:
        inputs = pickle.load(stream)
    assert digest(b.CALIBRATION_FILE) == inputs["calibration_sha256"]
    return inputs


def measure_predictions():
    assert not (OUTPUT / "receptions.json").exists(), "Do not replace the recorded timing pass"
    inputs = load_inputs()
    quad, pid = inputs["quad_params"], inputs["pid_params"]
    check_memory()
    b.predict(inputs["warmup"], b.LightweightPredictiveModel(quad, pid), delays=b.DELAY_GRID)
    records = []
    for seed, snapshot in inputs["snapshots"].items():
        check_memory()
        prediction, duration = b.predict_at_reception(snapshot, quad, pid)
        records.append(dict(seed=seed, prediction=prediction, computation_s=duration))
        print(f"Seed {seed}: {1000*duration:.2f} ms; {prediction['classification']}; "
              f"first unsafe D={prediction['first_bad_delay']}", flush=True)
    assert len(records) == 19
    (OUTPUT / "receptions.json").write_text(json.dumps(records, indent=2) + "\n")
    check_preserved()


def run_experiment():
    inputs = load_inputs()
    saved = json.loads(b.CALIBRATION_FILE.read_text())
    seeds = {json.dumps(r["command"], sort_keys=True): int(seed)
             for seed, r in saved.items() if r["command"] is not None}
    reception = {r["seed"]: r for r in json.loads((OUTPUT / "receptions.json").read_text())}
    snapshots = inputs["snapshots"]
    seed_by_snapshot = {id(snapshot): seed for seed, snapshot in snapshots.items()}
    previous = {(r["seed"], r["policy"], r["delay_s"]): r for r in
                map(json.loads, (OLD / "executions.jsonl").read_text().splitlines())}
    path = OUTPUT / "executions.jsonl"
    completed = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    rows, receptions_used = [], []
    actual = b.run_actual

    def saved_snapshot(s):
        seed = seeds.get(json.dumps(asdict(s), sort_keys=True))
        if seed is None:
            assert asdict(s) == asdict(b.generate_scenario(0))
            return inputs["warmup"]
        return snapshots[seed]

    def measured_reception(snapshot, quad, pid):
        assert not rows
        seed = seed_by_snapshot[id(snapshot)]
        receptions_used.append(seed)
        r = reception[seed]
        return r["prediction"], r["computation_s"]

    with path.open("a") as stream:
        def recorded(s, snapshot, dispatch, recovery, deadline, recover, **kwargs):
            policy = POLICIES[len(rows) % 3]
            seed = seeds[json.dumps(asdict(s), sort_keys=True)]
            waited = dispatch - round(snapshot.approval_time * b.CTRL_FREQ_HZ)
            delay = waited / b.CTRL_FREQ_HZ
            assert len(receptions_used) == 19
            assert deadline == round((snapshot.approval_time + b.MISSION_DEADLINE_S) * b.CTRL_FREQ_HZ)
            ttl = kwargs.get("ttl_prediction")
            assert (ttl is not None) == (policy == "StaleGuard")
            assert kwargs["recovery_mode"] == "safety-only"
            if len(rows) < len(completed):
                row = completed[len(rows)]
                assert (row["seed"], row["policy"], row["delay_s"]) == (seed, policy, delay)
                assert row["prediction"] == (ttl[0] if ttl else None)
                assert row["computation_s"] == (ttl[1] if ttl else None)
                rows.append(row)
                return tuple(row["outcome"])

            check_memory()
            trace, physics = [], []
            result = actual(s, snapshot, dispatch, recovery, deadline, recover,
                            trace=trace, physics_trace=physics, **kwargs)
            trace_digest = trace_hash(trace, physics)
            unsafe_time = None
            state = snapshot.state.copy()
            for tick, position in physics:
                state[[b.STATE_X, b.STATE_Y, b.STATE_Z]] = position
                if b.unsafe(state, snapshot):
                    unsafe_time = tick / b.PYB_FREQ_HZ
                    break
            ready = ttl is not None and ttl[1] <= delay
            if ttl:
                assert result[2] == bool(ready and b.ttl_expired_at_dispatch(ttl[0], waited))
            assert result[3] == (result[0] and not result[1])
            assert bool(unsafe_time is not None) == result[1]
            reference_policy = policy
            if ttl:
                reference_policy = "Always-Recover" if result[2] else "No Recovery"
            reference = previous[(seed, reference_policy, delay)]
            assert list(result) == reference["outcome"]
            assert trace_digest == reference["trace_sha256"]
            row = dict(seed=seed, policy=policy, scenario=asdict(s), delay_s=delay,
                       dispatch_step=dispatch, recovery_step=dispatch if ttl and result[2] else recovery,
                       deadline_step=deadline, outcome=result, trace_sha256=trace_digest,
                       prediction=ttl[0] if ttl else None, computation_s=ttl[1] if ttl else None,
                       prediction_ready=bool(ready), first_unsafe_time_s=unsafe_time,
                       worker_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
                       final_physics_time_s=physics[-1][0] / b.PYB_FREQ_HZ,
                       categories=(result[3], not result[0] and not result[1], result[1]))
            stream.write(json.dumps(row) + "\n")
            stream.flush()
            rows.append(row)
            del trace, physics
            gc.collect()
            LIBC.malloc_trim(0)  # Return unused allocator pages between Linux flights.
            if len(rows) % 57 == 0:
                print(f"Executed {len(rows)}/627 flights; RSS={check_memory()[0]:.0f} MiB",
                      file=sys.__stdout__, flush=True)
            return result

        # Warmup was executed in the prediction process, before measuring timing.
        with patch.object(b, "capture_approval", saved_snapshot), \
             patch.object(b, "load_scg_platform_params", return_value=(inputs["quad_params"], inputs["pid_params"])), \
             patch.object(b, "predict", return_value=None), \
             patch.object(b, "predict_at_reception", measured_reception), \
             patch.object(b, "run_actual", recorded), \
             LOG.open("w") as output, contextlib.redirect_stdout(output):
            b.main(count=20, experiment="B", recovery_mode="safety-only")
    assert len(rows) == 627
    check_preserved()
    (OUTPUT / "run_metadata.json").write_text(json.dumps(dict(
        policy_flights=627, reception_predictions=19, baseline_trace_matches=418,
        recorded_worker_peak_rss_mib=max(r.get("worker_peak_rss_mib", 0) for r in rows),
        memory_limits_mib=dict(max_process_rss=2048, min_host_available=2048),
        execution="Official main and run_actual; saved snapshots and freshly measured reception results; serial checkpoint/resume."
    ), indent=2) + "\n")
    print("PASS: all 627 flights, dispatch decisions, baseline traces, and preserved files.", flush=True)


def summarize():
    rows = [json.loads(line) for line in (OUTPUT / "executions.jsonl").read_text().splitlines()]
    old = [json.loads(line) for line in (OLD / "executions.jsonl").read_text().splitlines()]
    assert len(rows) == len(old) == 627
    before = {(r["seed"], r["policy"], r["delay_s"]): r for r in old}
    truth = json.loads((ROOT / "results/temporal_safety_correction/ground_truth_results.json").read_text())

    def counts(selected):
        return dict(flights=len(selected),
                    safe_completion=sum(r["outcome"][3] for r in selected),
                    inspection_completion=sum(r["outcome"][0] for r in selected),
                    safe_non_completion=sum(r["categories"][1] for r in selected),
                    unsafe=sum(r["outcome"][1] for r in selected),
                    fallbacks=sum(r["outcome"][2] for r in selected),
                    safe_fallbacks=sum(r["outcome"][2] and not r["outcome"][1] for r in selected))

    totals = {p: dict(before=counts([r for r in old if r["policy"] == p]),
                      after=counts([r for r in rows if r["policy"] == p])) for p in POLICIES}
    per_delay, changes, latency_changes, horizon_differences = [], [], [], []
    for index, delay in enumerate(b.DELAY_GRID):
        block = [r for r in rows if r["delay_s"] == round(float(delay), 1)]
        per_delay.append(dict(delay_s=round(float(delay), 1), N=19,
                              policies={p: counts([r for r in block if r["policy"] == p]) for p in POLICIES},
                              prediction_ready=sum(r["prediction_ready"] for r in block if r["policy"] == "StaleGuard")))
        for r in block:
            prior = before[(r["seed"], r["policy"], r["delay_s"])]
            if r["outcome"] != prior["outcome"]:
                changes.append(dict(seed=r["seed"], policy=r["policy"], delay_s=r["delay_s"],
                                    before=prior["outcome"], after=r["outcome"]))
            if r["policy"] == "StaleGuard":
                expired = b.ttl_expired_at_dispatch(r["prediction"], round(r["delay_s"] * b.CTRL_FREQ_HZ))
                with_old_timing = prior["prediction_ready"] and expired
                if with_old_timing != r["outcome"][2]:
                    latency_changes.append(dict(seed=r["seed"], delay_s=r["delay_s"],
                                               recovery_with_old_timing=with_old_timing,
                                               actual_recovery=r["outcome"][2]))
            if r["policy"] == "No Recovery":
                ttl_unsafe = truth[str(r["seed"])]["actual"]["results"][index] != "SAFE"
                if ttl_unsafe != r["outcome"][1]:
                    horizon_differences.append(dict(seed=r["seed"], delay_s=r["delay_s"],
                                                   ttl_horizon_unsafe=ttl_unsafe,
                                                   operational_unsafe=r["outcome"][1]))
    baseline_rows = [r for r in rows if r["policy"] != "StaleGuard"]
    assert all(r["trace_sha256"] == before[(r["seed"], r["policy"], r["delay_s"])]["trace_sha256"]
               for r in baseline_rows)
    times = [r["computation_s"] * 1000 for r in json.loads((OUTPUT / "receptions.json").read_text())]
    old_times = [r["computation_s"] * 1000 for r in old if r["policy"] == "StaleGuard" and r["delay_s"] == 0]
    def timing(values):
        return dict(min_ms=min(values), mean_ms=sum(values) / len(values), max_ms=max(values))
    sg = [r for r in rows if r["policy"] == "StaleGuard"]
    nr = {(r["seed"], r["delay_s"]): r for r in rows if r["policy"] == "No Recovery"}
    ar = {(r["seed"], r["delay_s"]): r for r in rows if r["policy"] == "Always-Recover"}
    paired = dict(completion_benefit_vs_no_recovery=sum(r["outcome"][3] and not nr[r["seed"], r["delay_s"]]["outcome"][3] for r in sg),
                  completion_harm_vs_no_recovery=sum(not r["outcome"][3] and nr[r["seed"], r["delay_s"]]["outcome"][3] for r in sg),
                  sg_only_completion_vs_always=sum(r["outcome"][3] and not ar[r["seed"], r["delay_s"]]["outcome"][3] for r in sg))
    deadlines = [r["deadline_step"] / b.CTRL_FREQ_HZ for r in sg]
    summary = dict(totals=totals, per_delay=per_delay, outcome_changes=changes,
                   latency_caused_decision_changes=latency_changes, timing_before=timing(old_times), timing_after=timing(times),
                   paired=paired, baseline_trace_matches=len(baseline_rows),
                   unsafe_cases=[r for r in sg if r["outcome"][1]],
                   seed9_D03=next(r for r in sg if r["seed"] == 9 and r["delay_s"] == .3),
                   seed10=[r for r in sg if r["seed"] == 10],
                   horizons=dict(ttl_absolute_end_s=MEASURE_SECONDS,
                                 mission_duration_from_approval_s=b.MISSION_DEADLINE_S,
                                 mission_absolute_deadline_range_s=[min(deadlines), max(deadlines)],
                                 unsafe_label_differences=horizon_differences))
    (OUTPUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    check_preserved()
    print(json.dumps({k: v for k, v in summary.items() if k not in ("seed10", "unsafe_cases", "seed9_D03")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("predict", "run", "report"), required=True)
    args = parser.parse_args()
    OUTPUT.mkdir(exist_ok=True)
    check_preserved()
    with (ROOT / "results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        {"predict": measure_predictions, "run": run_experiment, "report": summarize}[args.stage]()
