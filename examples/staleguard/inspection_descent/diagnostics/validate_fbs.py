"""Serial FBS flights; reuse saved four-policy results and regression-replay traces.

The official five-policy main/report is exercised. Only the four already verified
policies are served from saved results; every FBS flight runs native PyBullet.
Epsilon sensitivity selects between verified original/fallback trajectories.
"""
import argparse
import contextlib
from dataclasses import asdict
import fcntl
import gc
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_corrected_experiment_b import ROOT, b, check_memory, digest, load_inputs, trace_hash, LIBC
import numpy as np
import fbs

OUTPUT = ROOT / "results/fbs"
PRIOR = ROOT / "results/use_time_recheck"
POLICIES = ("StaleGuard", "No Recovery", "Always-Recover", "Use-Time Recheck", "FBS")


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def check_preserved():
    for name, expected in json.loads((OUTPUT / "preserved_files.json").read_text()).items():
        if name != "staleguard_benefit.py":  # The sole existing scientific file edited.
            assert digest(ROOT / name) == expected, name
    protocol = json.loads((OUTPUT / "epsilon_protocol.json").read_text())
    assert fbs.EPSILON == protocol["epsilon_m_per_sample"]
    assert fbs.ALPHA == protocol["alpha"]
    assert fbs.MAX_TTL_STEPS == protocol["max_ttl_steps"]
    assert b.CTRL_FREQ_HZ == protocol["observation_hz"]
    return protocol


def run():
    check_preserved()
    sources = [ROOT / "fbs.py", ROOT / "staleguard_benefit.py",
               Path(__file__), OUTPUT / "epsilon_protocol.json"]
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in sources}
    path = OUTPUT / "implementation_hashes.json"
    if path.exists():
        assert json.loads(path.read_text()) == hashes, "Implementation changed since checkpoint"
    else:
        path.write_text(json.dumps(hashes, indent=2)+"\n")
    inputs = load_inputs()
    saved = json.loads(b.CALIBRATION_FILE.read_text())
    seeds = {json.dumps(r["command"], sort_keys=True): int(seed)
             for seed, r in saved.items() if r["command"]}
    old = {(r["seed"], r["delay_s"], r["policy"]): r
           for r in read_rows(PRIOR / "executions.jsonl")}
    receptions = {r["seed"]: r for r in json.loads((PRIOR / "receptions.json").read_text())}
    path = OUTPUT / "executions.jsonl"
    completed = read_rows(path)
    calls, rows = 0, []
    actual = b.run_actual

    def snapshot(s):
        seed = seeds.get(json.dumps(asdict(s), sort_keys=True))
        return inputs["snapshots"][seed] if seed is not None else inputs["warmup"]

    def reception(current, quad, pid):
        seed = next(seed for seed, value in inputs["snapshots"].items() if value is current)
        r = receptions[seed]
        return r["prediction"], r["computation_s"]

    with path.open("a") as stream:
        def recorded(s, current, dispatch, recovery, deadline, recover, **kwargs):
            nonlocal calls
            policy = POLICIES[calls % 5]
            calls += 1
            seed = seeds[json.dumps(asdict(s), sort_keys=True)]
            approval = round(current.approval_time * b.CTRL_FREQ_HZ)
            delay = (dispatch-approval)/b.CTRL_FREQ_HZ
            assert deadline == round((current.approval_time+b.MISSION_DEADLINE_S)*b.CTRL_FREQ_HZ)
            assert kwargs["recovery_mode"] == "safety-only"
            if policy != "FBS":
                r = old[seed, delay, policy]
                assert dispatch == r["dispatch_step"] and deadline == r["deadline_step"]
                if policy == "StaleGuard":
                    assert kwargs["ttl_prediction"] == (r["prediction"], r["computation_s"])
                elif policy == "Always-Recover":
                    assert recovery == approval and recover == (approval < dispatch)
                else:
                    assert not recover
                if policy == "Use-Time Recheck":
                    kwargs["recheck_log"].update(r["recheck"])
                return tuple(r["outcome"])
            if len(rows) < len(completed):
                r = completed[len(rows)]
                assert (r["seed"], r["delay_s"]) == (seed, delay)
                kwargs["fbs_log"].update(r["fbs"])
                rows.append(r)
                return tuple(r["outcome"])
            rss, available = check_memory()
            trace, physics = [], []
            outcome = actual(s, current, dispatch, recovery, deadline, recover,
                             trace=trace, physics_trace=physics, **kwargs)
            event = kwargs["fbs_log"]
            # Independently recompute the entire observed prefix and EMA.
            assert event["observation_steps"] == list(range(approval+1))
            observed = []
            volatility = 0.0
            for _, state, _ in trace[:approval+1]:
                feature = float(max(s.box_x_min-state[0], state[0]-s.box_x_max,
                                    abs(state[2])-s.box_y_limit, state[4]-s.unsafe_below_z))
                if observed:
                    volatility = .1*abs(feature-observed[-1]) + .9*volatility
                observed.append(feature)
            assert observed == event["features_m"]
            np.testing.assert_array_equal(event["observation_state"], trace[approval][1])
            np.testing.assert_array_equal(event["observation_state"], current.state)
            prediction = event["prediction"]
            assert volatility == prediction["volatility_m_per_sample"]
            assert prediction["margin_m"] == observed[-1]
            assert prediction["ttl_steps"] == math.floor(min(210, max(observed[-1], 0)/max(volatility, fbs.EPSILON)))
            assert event["waited_steps"] == dispatch-approval
            assert event["ready"] == (event["computation_s"] <= delay)
            expired = observed[-1] <= 0 or dispatch-approval > prediction["ttl_steps"]
            assert event["recover"] == (event["ready"] and expired) == outcome[2]
            assert trace[dispatch+1][2][b.STATE_Z] == (s.usual_height if outcome[2] else s.low_height)
            assert event["application_step"] == (None if outcome[2] else dispatch)
            assert event["fallback_step"] == (dispatch if outcome[2] else None)
            assert physics[dispatch*20][0] == dispatch*20
            original = old[seed, delay, "No Recovery"]
            assert trace_hash(trace[:dispatch+1], []) == original["prefix_sha256"]
            reference = old[seed, delay if delay else .1, "Always-Recover"] if outcome[2] else original
            observed_hash = trace_hash(trace, physics)
            assert list(outcome) == reference["outcome"]
            assert observed_hash == reference["trace_sha256"]
            event["observation_state"] = event["observation_state"].tolist()
            row = dict(seed=seed, delay_s=delay, policy=policy, scenario=asdict(s), outcome=outcome,
                       dispatch_step=dispatch, deadline_step=deadline, fbs=event,
                       trace_sha256=observed_hash, reference_trace_match=True,
                       first_unsafe_time_s=reference["first_unsafe_time_s"],
                       observation_dtype=str(current.state.dtype), worker_rss_mib=rss,
                       host_available_mib=available)
            stream.write(json.dumps(row)+"\n"); stream.flush()
            rows.append(row)
            del trace, physics
            gc.collect(); LIBC.malloc_trim(0)
            if len(rows) % 19 == 0:
                print(f"Executed {len(rows)}/209 FBS flights; RSS={check_memory()[0]:.0f} MiB", flush=True,
                      file=sys.__stdout__)
            return outcome

        with patch.object(b, "capture_approval", snapshot), \
             patch.object(b, "load_scg_platform_params", return_value=(inputs["quad_params"], inputs["pid_params"])), \
             patch.object(b, "predict", return_value=None), \
             patch.object(b, "predict_immediate", return_value="SAFE"), \
             patch.object(b, "predict_at_reception", reception), \
             patch.object(b, "run_actual", recorded), \
             (ROOT / "result/baseline_fbs.log").open("w") as output, contextlib.redirect_stdout(output):
            b.main(count=20, experiment="B", include_fbs=True)
    assert calls == 1045 and len(rows) == 209
    check_preserved()
    print("PASS: 209 actual FBS flights; 836 saved existing-policy flights reused.", flush=True)


def regression():
    """Replay existing executors with frozen recorded prediction results/times."""
    inputs = load_inputs()
    old = read_rows(PRIOR / "executions.jsonl")
    selected = [r for r in old if (r["seed"], r["delay_s"]) in ((0, 0), (7, .4), (11, .6))]
    records = []
    for row in selected:
        check_memory()
        current = inputs["snapshots"][row["seed"]]
        dispatch, deadline = row["dispatch_step"], row["deadline_step"]
        approval = round(current.approval_time*b.CTRL_FREQ_HZ)
        recover, recovery, kwargs = False, dispatch, {}
        if row["policy"] == "StaleGuard":
            kwargs["ttl_prediction"] = (row["prediction"], row["computation_s"])
        elif row["policy"] == "Always-Recover":
            recover, recovery = approval < dispatch, approval
        elif row["policy"] == "Use-Time Recheck":
            kwargs["use_time_check"] = lambda current: row["recheck"]["prediction"]
        trace, physics = [], []
        # Only the recheck branch reads this clock. Preserve its old measured time.
        elapsed = row.get("recheck", {}).get("computation_s", 0)
        with patch.object(b.time, "perf_counter", side_effect=[0, elapsed]):
            outcome = b.run_actual(b.Scenario(**row["scenario"]), current, dispatch, recovery,
                                   deadline, recover, trace=trace, physics_trace=physics, **kwargs)
        assert list(outcome) == row["outcome"]
        assert trace_hash(trace, physics) == row["trace_sha256"]
        records.append(dict(seed=row["seed"], delay_s=row["delay_s"], policy=row["policy"],
                            trace_match=True))
        del trace, physics
        gc.collect(); LIBC.malloc_trim(0)
    assert len(records) == 12
    (OUTPUT / "existing_policy_replays.json").write_text(json.dumps(records, indent=2)+"\n")
    check_preserved()
    print("PASS: 12 existing-policy control/physics traces match exactly.", flush=True)


def counts(rows):
    return dict(N=len(rows), unsafe=sum(r["outcome"][1] for r in rows),
                safe_completion=sum(r["outcome"][3] for r in rows),
                safe_non_completion=sum(not r["outcome"][0] and not r["outcome"][1] for r in rows),
                fallbacks=sum(r["outcome"][2] for r in rows))


def stats(values):
    return dict(min=float(min(values)), mean=float(np.mean(values)), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), max=float(max(values)))


def report():
    protocol = check_preserved()
    rows = read_rows(OUTPUT / "executions.jsonl")
    old_rows = read_rows(PRIOR / "executions.jsonl")
    assert len(rows) == 209 and len(old_rows) == 836
    all_rows = old_rows+rows
    old = {(r["seed"], r["delay_s"], r["policy"]): r for r in old_rows}
    # Approval features and TTL are independent of which future D is enumerated.
    unique = [r for r in rows if r["delay_s"] == 0]
    for r in rows:
        first = next(a for a in unique if a["seed"] == r["seed"])
        assert r["fbs"]["prediction"] == first["fbs"]["prediction"]
        assert r["fbs"]["features_m"] == first["fbs"]["features_m"]
    sensitivity = []
    for epsilon in [protocol["epsilon_m_per_sample"]]+protocol["sensitivity_epsilon_m_per_sample"]:
        paired, changes, ttl_values = [], 0, []
        for r in rows:
            event, prior = r["fbs"], r["fbs"]["prediction"]
            prediction = fbs.freeze_prediction(prior["margin_m"], prior["volatility_m_per_sample"], epsilon)
            status = fbs.dispatch_status(prediction, event["computation_s"], event["waited_steps"])
            reference = old[r["seed"], r["delay_s"] if r["delay_s"] else .1, "Always-Recover"] if status["recover"] else old[r["seed"], r["delay_s"], "No Recovery"]
            paired.append(reference)
            changes += status["recover"] != event["recover"]
            if r["delay_s"] == 0:
                ttl_values.append(prediction["ttl_steps"])
        sensitivity.append(dict(epsilon_m_per_sample=epsilon, decisions_changed=changes,
                                metrics=counts(paired), ttl_steps=stats(ttl_values)))
    ttl_steps = [r["fbs"]["prediction"]["ttl_steps"] for r in unique]
    cold_seeds = [r["seed"] for r in unique if r["fbs"]["cold_start"]]
    result = dict(totals={p: counts([r for r in all_rows if r["policy"] == p]) for p in POLICIES},
        actual_new_flights=209, reused_prior_flights=836, infeasible_seeds=[16],
        per_delay=[dict(delay_s=d/10, fbs=counts([r for r in rows if r["delay_s"] == d/10]),
                        ready=sum(r["fbs"]["ready"] for r in rows if r["delay_s"] == d/10)) for d in range(11)],
        approval_computation_us=stats([r["fbs"]["computation_s"]*1e6 for r in rows]),
        observation_update_us=stats([t*1e6 for r in rows for t in r["fbs"]["observation_update_s"][:-1]]),
        prior_monitoring_total_us=stats([sum(r["fbs"]["observation_update_s"][:-1])*1e6 for r in rows]),
        ttl_steps=stats(ttl_steps), ttl_seconds=stats([t/b.CTRL_FREQ_HZ for t in ttl_steps]),
        ttl_histogram={str(t): ttl_steps.count(t) for t in sorted(set(ttl_steps))},
        approvals_at_max_ttl=sum(t == fbs.MAX_TTL_STEPS for t in ttl_steps),
        ready=sum(r["fbs"]["ready"] for r in rows),
        unavailable=sum(not r["fbs"]["ready"] for r in rows),
        cold_start_seeds=cold_seeds,
        cold_start=counts([r for r in rows if r["fbs"]["cold_start"]]),
        with_history=counts([r for r in rows if not r["fbs"]["cold_start"]]),
        per_seed=[dict(seed=r["seed"], observations=len(r["fbs"]["features_m"]),
                       prediction=r["fbs"]["prediction"], cold_start=r["fbs"]["cold_start"]) for r in unique],
        epsilon_sensitivity=sensitivity,
        false_rejection_safe_completion_losses=sum(r["outcome"][2] and old[r["seed"], r["delay_s"], "No Recovery"]["outcome"][3] for r in rows),
        epsilon_protocol=protocol, reference_trace_matches=sum(r["reference_trace_match"] for r in rows),
        max_worker_rss_mib=max(r["worker_rss_mib"] for r in rows),
        timing_model="Measured final approval-update/TTL time; availability at scheduled simulated dispatch; fail-open, no postponed application.")
    for metrics in result["totals"].values():
        assert metrics["N"] == 209
        assert metrics["unsafe"]+metrics["safe_completion"]+metrics["safe_non_completion"] == 209
    (ROOT / "result/fbs.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({k:v for k,v in result.items() if k not in ("per_seed", "epsilon_protocol")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("run", "regression", "report"), required=True)
    args = parser.parse_args()
    check_preserved()
    with (ROOT / "results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        {"run": run, "regression": regression, "report": report}[args.stage]()
