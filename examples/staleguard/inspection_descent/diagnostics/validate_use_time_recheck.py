"""Serial four-policy B evaluation, with live dispatch snapshots and measured checks."""
import argparse
import contextlib
from dataclasses import asdict
import fcntl
import gc
import json
from pathlib import Path
import pickle
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_corrected_experiment_b import ROOT, b, check_memory, digest, load_inputs, trace_hash, LIBC
import numpy as np

OUTPUT = ROOT / "results/use_time_recheck"
RESULT = ROOT / "result/use_time_recheck.json"
LOG = ROOT / "result/baseline_use_time_recheck.log"
POLICIES = ("StaleGuard", "No Recovery", "Always-Recover", "Use-Time Recheck")


def check_preserved():
    for name, expected in json.loads((OUTPUT / "preserved_files.json").read_text()).items():
        if name != "staleguard_benefit.py":  # Only the opt-in policy integration changes.
            assert digest(ROOT / name) == expected, name


def experiment():
    inputs = load_inputs()
    saved = json.loads(b.CALIBRATION_FILE.read_text())
    seeds = {json.dumps(r["command"], sort_keys=True): int(seed) for seed, r in saved.items() if r["command"]}
    old = {(r["seed"], r["delay_s"], r["policy"]): r for r in map(json.loads,
           (ROOT / "results/corrected_experiment_b/executions.jsonl").read_text().splitlines())}
    path = OUTPUT / "executions.jsonl"
    completed = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    rows = []
    actual = b.run_actual
    original_reception = b.predict_at_reception
    reception_path = OUTPUT / "receptions.json"
    receptions = {r["seed"]: r for r in json.loads(reception_path.read_text())} if reception_path.exists() else {}

    def cached_snapshot(s):
        seed = seeds.get(json.dumps(asdict(s), sort_keys=True))
        return inputs["snapshots"][seed] if seed is not None else inputs["warmup"]

    def reception(snapshot, quad, pid):
        seed = next(seed for seed, value in inputs["snapshots"].items() if value is snapshot)
        if seed not in receptions:
            prediction, elapsed = original_reception(snapshot, quad, pid)
            receptions[seed] = dict(seed=seed, prediction=prediction, computation_s=elapsed)
            reception_path.write_text(json.dumps(list(receptions.values()), indent=2)+"\n")
        r = receptions[seed]
        return r["prediction"], r["computation_s"]

    with path.open("a") as stream:
        def recorded(s, snapshot, dispatch, recovery, deadline, recover, **kwargs):
            policy = POLICIES[len(rows) % 4]
            seed = seeds[json.dumps(asdict(s), sort_keys=True)]
            delay = (dispatch-round(snapshot.approval_time*b.CTRL_FREQ_HZ))/b.CTRL_FREQ_HZ
            if len(rows) < len(completed):
                row = completed[len(rows)]
                assert (row["seed"], row["policy"], row["delay_s"]) == (seed, policy, delay)
                if policy == "Use-Time Recheck":
                    # Native main needs timing fields for its report on resume.
                    kwargs["recheck_log"].update(row["recheck"])
                rows.append(row)
                return tuple(row["outcome"])
            check_memory()
            trace, physics = [], []
            result = actual(s, snapshot, dispatch, recovery, deadline, recover,
                            trace=trace, physics_trace=physics, **kwargs)
            row = dict(seed=seed, delay_s=delay, policy=policy, outcome=result,
                       dispatch_step=dispatch, deadline_step=deadline, scenario=asdict(s),
                       trace_sha256=trace_hash(trace, physics),
                       prefix_sha256=trace_hash(trace[:dispatch+1], []))
            state = snapshot.state.copy()
            row["first_unsafe_time_s"] = None
            for tick, position in physics:
                state[[b.STATE_X, b.STATE_Y, b.STATE_Z]] = position
                if b.unsafe(state, snapshot):
                    row["first_unsafe_time_s"] = tick/b.PYB_FREQ_HZ
                    break
            if policy == "Use-Time Recheck":
                event = kwargs["recheck_log"]
                current = event["snapshot"]
                np.testing.assert_array_equal(current.state, trace[dispatch][1])
                assert current.approval_time == dispatch/b.CTRL_FREQ_HZ
                assert current.horizon_steps == round(b.MEASURE_SECONDS*b.CTRL_FREQ_HZ)-dispatch
                assert current.low_steps == snapshot.low_steps
                assert current.remaining_reference[0, b.STATE_Z] == s.usual_height
                assert deadline == round((snapshot.approval_time+b.MISSION_DEADLINE_S)*b.CTRL_FREQ_HZ)
                decision = event["decision_step"]
                elapsed = event["computation_s"]
                assert (decision-dispatch-1)/b.CTRL_FREQ_HZ <= elapsed < (decision-dispatch)/b.CTRL_FREQ_HZ
                for k in range(dispatch, min(decision, deadline)):
                    assert trace[k+1][2][b.STATE_Z] == s.usual_height
                    np.testing.assert_array_equal(trace[k+1][2], current.remaining_reference[k-dispatch])
                if decision < deadline:
                    assert event["decision_applied_step"] == decision
                    assert trace[decision+1][2][b.STATE_Z] == (s.low_height if event["prediction"] == "SAFE" else s.usual_height)
                    assert physics[decision*(b.PYB_FREQ_HZ//b.CTRL_FREQ_HZ)][0] == decision*(b.PYB_FREQ_HZ//b.CTRL_FREQ_HZ)
                immediate = next(r for r in rows if r["seed"] == seed and r["delay_s"] == delay and r["policy"] == "No Recovery")
                assert row["prefix_sha256"] == immediate["prefix_sha256"]
                if result[2]:
                    fallback = old[seed, delay if delay > 0 else .1, "Always-Recover"]
                    assert row["trace_sha256"] == fallback["trace_sha256"]
                with (OUTPUT / f"snapshot_{seed}_{round(delay*10)}.pkl").open("wb") as cache:
                    pickle.dump(current, cache)
                event.pop("snapshot")
                event["dispatch_state"] = current.state.tolist()
                if "state_at_decision" in event:
                    event["state_at_decision"] = event["state_at_decision"].tolist()
                    event["distance_during_check_m"] = float(np.linalg.norm(np.array(event["state_at_decision"])[[0,2,4]]-current.state[[0,2,4]]))
                row["recheck"] = dict(event)
                row["immediate_command_unsafe"] = immediate["outcome"][1]
                row["check_to_use_unsafe"] = bool(event["prediction"] == "SAFE" and result[1] and not immediate["outcome"][1])
                row["already_false_safe_at_check"] = bool(event["prediction"] == "SAFE" and immediate["outcome"][1])
            else:
                row["previous_trace_match"] = row["trace_sha256"] == old[seed, delay, policy]["trace_sha256"]
                if "ttl_prediction" in kwargs:
                    row["prediction"], row["computation_s"] = kwargs["ttl_prediction"]
            stream.write(json.dumps(row) + "\n"); stream.flush()
            rows.append(row)
            del trace, physics
            gc.collect(); LIBC.malloc_trim(0)
            if len(rows) % 76 == 0:
                print(f"Executed {len(rows)}/836 policy flights; RSS={check_memory()[0]:.0f} MiB",
                      file=sys.__stdout__, flush=True)
            return result

        with patch.object(b, "capture_approval", cached_snapshot), \
             patch.object(b, "load_scg_platform_params", return_value=(inputs["quad_params"], inputs["pid_params"])), \
             patch.object(b, "predict_at_reception", reception), \
             patch.object(b, "run_actual", recorded), \
             LOG.open("w") as output, contextlib.redirect_stdout(output):
            b.main(count=20, experiment="B", use_time_recheck=True)
    assert len(rows) == 836
    check_preserved()
    print("PASS: 836 live policy flights, dispatch-state and nominal-wait trace checks.")


def report():
    rows = [json.loads(line) for line in (OUTPUT / "executions.jsonl").read_text().splitlines()]
    assert len(rows) == 836
    metrics = lambda block: {p: dict(N=sum(r["policy"]==p for r in block),
        unsafe=sum(r["outcome"][1] for r in block if r["policy"]==p),
        safe_completion=sum(r["outcome"][3] for r in block if r["policy"]==p),
        safe_non_completion=sum(not r["outcome"][0] and not r["outcome"][1] for r in block if r["policy"]==p),
        fallbacks=sum(r["outcome"][2] for r in block if r["policy"]==p)) for p in POLICIES}
    rechecks = [r for r in rows if r["policy"] == "Use-Time Recheck"]
    assert sum(r.get("previous_trace_match",False) for r in rows) == 627
    durations = np.array([r["recheck"]["computation_s"]*1000 for r in rechecks])
    delays = np.array([(r["recheck"]["decision_step"]-r["dispatch_step"])*1000/b.CTRL_FREQ_HZ for r in rechecks])
    application_delays = np.array([(r["recheck"]["application_step"]-r["dispatch_step"])*1000/b.CTRL_FREQ_HZ
                                  for r in rechecks if r["recheck"]["application_step"] is not None])
    stats = lambda a: dict(mean_ms=float(a.mean()), median_ms=float(np.median(a)),
                           p95_ms=float(np.percentile(a,95)), max_ms=float(a.max()), min_ms=float(a.min()))
    result = dict(evaluated=19, infeasible_seeds=[16], policy_flights=836, totals=metrics(rows),
                  per_delay=[dict(delay_s=round(float(d),1), policies=metrics([r for r in rows if r["delay_s"]==round(float(d),1)])) for d in b.DELAY_GRID],
                  recheck_computation=stats(durations), recheck_decision_delay=stats(delays),
                  command_application_delay=stats(application_delays),
                  decision_not_applied_by_deadline=sum(r["recheck"]["decision_applied_step"] is None for r in rechecks),
                  prediction_safe=sum(r["recheck"]["prediction"]=="SAFE" for r in rechecks),
                  check_to_use_unsafe=[r for r in rechecks if r["check_to_use_unsafe"]],
                  already_false_safe_at_check=[r for r in rechecks if r["already_false_safe_at_check"]],
                  unsafe_rechecks=[r for r in rechecks if r["outcome"][1]],
                  previous_baseline_trace_matches=sum(r.get("previous_trace_match",False) for r in rows),
                  maximum_dispatch_to_decision_motion_m=max(r["recheck"].get("distance_during_check_m",0) for r in rechecks),
                  timing_model="Measured wall time mapped to subsequent nominal PID/physics steps; serial simulator, not concurrent onboard execution")
    RESULT.write_text(json.dumps(result,indent=2)+"\n")
    check_preserved()
    print(json.dumps({k:v for k,v in result.items() if k not in ("check_to_use_unsafe","already_false_safe_at_check","unsafe_rechecks")},indent=2))


def timing_replays():
    """Offline physical comparisons only; no second predictive decision."""
    rows = [json.loads(line) for line in (OUTPUT/"executions.jsonl").read_text().splitlines()]
    rechecks = [r for r in rows if r["policy"]=="Use-Time Recheck"]
    selected = [next(r for r in rechecks if r["recheck"]["prediction"]=="SAFE")]
    selected += [r for r in rechecks if r["check_to_use_unsafe"] or r["already_false_safe_at_check"]]
    inputs = load_inputs()
    results = []
    for r in selected:
        tasks = [("delayed application", r["recheck"]["application_step"], r)]
        if r["check_to_use_unsafe"] or r["already_false_safe_at_check"]:
            original = next(a for a in rows if a["policy"]=="No Recovery" and a["seed"]==r["seed"] and a["delay_s"]==r["delay_s"])
            tasks.append(("immediate application", r["dispatch_step"], original))
        for label, application, expected in tasks:
            check_memory()
            trace, physics = [], []
            outcome = b.run_actual(b.Scenario(**r["scenario"]), inputs["snapshots"][r["seed"]],
                                   application, application, r["deadline_step"], False,
                                   trace=trace, physics_trace=physics)
            assert list(outcome)==expected["outcome"]
            observed_hash=trace_hash(trace,physics)
            assert observed_hash==expected["trace_sha256"]
            if r["check_to_use_unsafe"]:
                np.savez_compressed(OUTPUT/f"staleness_{r['seed']}_{label.split()[0]}.npz",
                                    control_steps=[a[0] for a in trace], states=[a[1] for a in trace],
                                    references=[a[2] for a in trace], physics_ticks=[a[0] for a in physics],
                                    positions=[a[1] for a in physics])
            results.append(dict(seed=r["seed"],delay_s=r["delay_s"],comparison=label,
                                application_step=application,outcome=outcome,trace_sha256=observed_hash))
            del trace,physics
            gc.collect();LIBC.malloc_trim(0)
    (OUTPUT/"timing_replays.json").write_text(json.dumps(results,indent=2)+"\n")
    check_preserved()
    print(f"PASS: {len(results)} direct-timing replays match whole control/physics traces.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("experiment","report","timing"), required=True)
    args = parser.parse_args()
    check_preserved()
    with (ROOT/"results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        {"experiment":experiment,"report":report,"timing":timing_replays}[args.stage]()
