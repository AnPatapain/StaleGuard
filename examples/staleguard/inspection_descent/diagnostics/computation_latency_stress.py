"""Availability-only slowdown sweep of frozen Experiment B predictions.

The sweep selects verified paired trajectories; validate replays boundary cases
through native PyBullet execution. No predictor or physical delay is changed.
"""
import argparse
import fcntl
import gc
import json
import math
from pathlib import Path
import resource
import shutil
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from short_delay_latency import (
    ROOT, INPUTS, DELAY_STEPS, LIBC, b, check_memory, digest, load_inputs, trace_hash,
)

OUTPUT = ROOT / "results/computation_latency_stress"
SOURCE = ROOT / "results/short_delay_latency"
RESULT = ROOT / "result/computation_latency_stress.json"


def freeze_and_check():
    OUTPUT.mkdir(exist_ok=True)
    manifest = OUTPUT / "preserved_files.json"
    if not manifest.exists():
        files = list(ROOT.glob("*.py")) + list((ROOT / "model").glob("*.py"))
        files += [INPUTS, b.CALIBRATION_FILE, ROOT / "diagnostics/short_delay_latency.py"]
        files += [p for p in (ROOT / "result").iterdir() if p.suffix in (".log", ".json") and p != RESULT]
        for name in ("short_delay_latency", "corrected_experiment_b", "unknown_dispatch_delay", "temporal_safety_correction"):
            files += [p for p in (ROOT / "results" / name).iterdir() if p.suffix in (".json", ".jsonl", ".log")]
        for name in ("receptions.json", "executions.jsonl"):
            target = OUTPUT / name
            assert not target.exists(), "Do not replace frozen inputs"
            shutil.copyfile(SOURCE / name, target)
            files.append(target)
        manifest.write_text(json.dumps({str(p.relative_to(ROOT)): digest(p) for p in files}, indent=2) + "\n")
    for name, expected in json.loads(manifest.read_text()).items():
        assert digest(ROOT / name) == expected, name


def load_cases():
    receptions = {r["seed"]: r for r in json.loads((OUTPUT / "receptions.json").read_text())}
    rows = [json.loads(line) for line in (OUTPUT / "executions.jsonl").read_text().splitlines()]
    assert len(rows) == 342 and len(receptions) == 19 and 16 not in receptions
    saved = {(r["seed"], r["delay_steps"], r["policy"]): r for r in rows}
    cases = []
    for steps in DELAY_STEPS:
        for seed, r in receptions.items():
            original = saved[seed, steps, "No Recovery"]
            measured = saved[seed, steps, "StaleGuard"]
            reject = bool(b.ttl_expired_at_dispatch(r["prediction"], steps))
            # D=0 stays immediate original-command dispatch in both arms.
            counterfactual = measured if steps > 0 and reject else original
            if steps > 0 and reject:
                assert counterfactual["outcome"][2]
                assert counterfactual["expiry_check_steps"] == [original["dispatch_step"]]
            assert original["dispatch_step"] == counterfactual["dispatch_step"]
            assert original["deadline_step"] == counterfactual["deadline_step"]
            cases.append(dict(seed=seed, steps=steps, delay_s=steps/b.CTRL_FREQ_HZ,
                              computation_s=r["computation_s"], prediction=r["prediction"],
                              would_reject=reject, original=original,
                              on_time=counterfactual, measured=measured))
    return cases, receptions


def last_ready_alpha(computation_s, delay_s):
    """Last float alpha whose native alpha*C <= D check is ready.

    Division/multiplication can round on opposite sides of equality. Adjust
    only alpha by adjacent representable values; never round or alter alpha*C.
    """
    alpha = delay_s / computation_s
    while alpha * computation_s > delay_s:
        alpha = math.nextafter(alpha, -math.inf)
    while math.nextafter(alpha, math.inf) * computation_s <= delay_s:
        alpha = math.nextafter(alpha, math.inf)
    return alpha


def transitions(cases):
    rows = []
    for c in cases:
        if c["steps"] == 0:
            continue
        at = last_ready_alpha(c["computation_s"], c["delay_s"])
        if at >= 1:
            rows.append(dict(seed=c["seed"], steps=c["steps"], delay_s=c["delay_s"],
                             alpha_last_ready=at, alpha_first_late=math.nextafter(at, math.inf),
                             changes_execution=c["would_reject"]))
    return sorted(rows, key=lambda r: r["alpha_last_ready"])


def selected(c, alpha, on_time=False):
    ready = c["steps"] > 0 if on_time else alpha*c["computation_s"] <= c["delay_s"]
    rejected = ready and c["would_reject"]
    return ready, rejected, c["on_time"] if rejected else c["original"]


def metrics(cases, alpha, on_time=False):
    counts = dict(N=len(cases), ready=0, unavailable=0, originals_because_unavailable=0,
                  unsafe=0, latency_unsafe=0, incorrect_prediction_unsafe=0,
                  unsafe_despite_on_time_rejection=0, safe_completion=0,
                  inspection_completion=0, safe_non_completion=0, fallbacks=0,
                  false_rejections=0, completion_harm_vs_no_recovery=0,
                  completion_gain_vs_on_time=0)
    for c in cases:
        ready, rejected, row = selected(c, alpha, on_time)
        inspected, unsafe, _, complete = row["outcome"]
        cf_unsafe = c["on_time"]["outcome"][1]
        late_unsafe = unsafe and not ready and c["would_reject"] and not cf_unsafe
        if late_unsafe:
            assert row["first_unsafe_time_s"] > row["dispatch_step"] / b.CTRL_FREQ_HZ
        counts["ready"] += int(ready)
        counts["unavailable"] += int(not ready)
        counts["originals_because_unavailable"] += int(not ready)
        counts["unsafe"] += int(unsafe)
        counts["latency_unsafe"] += int(late_unsafe)
        counts["incorrect_prediction_unsafe"] += int(unsafe and not c["would_reject"])
        counts["unsafe_despite_on_time_rejection"] += int(unsafe and c["would_reject"] and cf_unsafe)
        counts["safe_completion"] += int(complete)
        counts["inspection_completion"] += int(inspected)
        counts["safe_non_completion"] += int(not inspected and not unsafe)
        counts["fallbacks"] += int(rejected)
        counts["false_rejections"] += int(rejected and not c["original"]["outcome"][1])
        counts["completion_harm_vs_no_recovery"] += int(rejected and not complete and c["original"]["outcome"][3])
        counts["completion_gain_vs_on_time"] += int(complete and not c["on_time"]["outcome"][3])
    assert counts["ready"] + counts["unavailable"] == counts["N"]
    assert counts["safe_completion"] + counts["safe_non_completion"] + counts["unsafe"] == counts["N"]
    assert counts["latency_unsafe"] + counts["incorrect_prediction_unsafe"] + counts["unsafe_despite_on_time_rejection"] == counts["unsafe"]
    return counts


def analyse():
    cases, receptions = load_cases()
    events = transitions(cases)
    alphas = sorted({1.0} | {r[k] for r in events for k in ("alpha_last_ready", "alpha_first_late")})
    samples = [dict(alpha=a, origin="measured timing baseline" if a == 1 else "simulated availability stress",
                    totals=metrics(cases, a),
                    per_delay=[dict(delay_s=steps/b.CTRL_FREQ_HZ,
                                    **metrics([c for c in cases if c["steps"] == steps], a)) for steps in DELAY_STEPS])
               for a in alphas]
    for c in cases:
        assert selected(c, 1)[2]["trace_sha256"] == c["measured"]["trace_sha256"]
    for e in events:
        c = next(c for c in cases if (c["seed"], c["steps"]) == (e["seed"], e["steps"]))
        assert selected(c, e["alpha_last_ready"])[0]
        assert not selected(c, e["alpha_first_late"])[0]
    assert all(a["totals"]["ready"] >= z["totals"]["ready"] for a, z in zip(samples, samples[1:]))
    assert samples[-1]["totals"]["ready"] == 0
    assert all(not selected(c, a)[1] for c in cases if c["steps"] == 0 for a in alphas)
    outcome_changes = []
    for event in events:
        if not event["changes_execution"]:
            continue
        c = next(c for c in cases if (c["seed"], c["steps"]) == (event["seed"], event["steps"]))
        kind = "latency-caused unsafe entry" if c["original"]["outcome"][1] and not c["on_time"]["outcome"][1] else "false rejection removed"
        outcome_changes.append(dict(**event, change=kind,
                                    first_unsafe_time_s=c["original"]["first_unsafe_time_s"]))
    source_summary = json.loads((ROOT / "result/short_delay_latency.json").read_text())
    baselines = {"No Recovery": dict(unsafe=sum(c["original"]["outcome"][1] for c in cases),
                                     safe_completion=sum(c["original"]["outcome"][3] for c in cases))}
    baselines["Always-Recover"] = dict(
        unsafe=sum(r["policies"]["Always-Recover"]["unsafe"] for r in source_summary["per_delay"]),
        safe_completion=sum(r["policies"]["Always-Recover"]["safe_completion"] for r in source_summary["per_delay"]))
    result = dict(experiment="Experiment B: frozen-runtime availability stress", considered=20,
                  feasible=19, excluded_seeds=[16], paired_cases=114,
                  measured_prediction_timing=source_summary["prediction_timing"],
                  frozen_reception_predictions=list(receptions.values()),
                  availability_rule="alpha * frozen computation_s <= dispatch delay_s",
                  D0_rule="Immediate original command in both arms; no zero-duration prediction",
                  mission_deadline_after_approval_s=b.MISSION_DEADLINE_S,
                  measured_baseline=samples[0], on_time_counterfactual=metrics(cases, 1, on_time=True),
                  existing_baselines=baselines, readiness_transitions=events,
                  execution_changes=outcome_changes, simulated_sweep=samples[1:],
                  reused_original_trajectories=114, reused_dispatch_fallback_trajectories=5,
                  scientific_note="Alpha>1 changes simulated availability only; it is not measured slower hardware.")
    validation_path = OUTPUT / "validation.json"
    if validation_path.exists():
        result["native_boundary_validation"] = json.loads(validation_path.read_text())
    RESULT.write_text(json.dumps(result, indent=2) + "\n")
    freeze_and_check()
    print(f"Analysed {len(cases)} paired cases at {len(alphas)} alpha values; "
          f"{len(events)} readiness transitions and {len(outcome_changes)} execution changes.")


def validation_plan(cases):
    events = transitions(cases)
    representatives = [e for e in events if e["changes_execution"]]
    for steps in DELAY_STEPS:
        candidates = [e for e in events if e["steps"] == steps and not e["changes_execution"]]
        if candidates:
            representatives.append(min(candidates, key=lambda e: e["seed"]))
    plans = []
    for e in representatives:
        for phase, alpha in (("before", math.nextafter(e["alpha_last_ready"], -math.inf)),
                             ("at", e["alpha_last_ready"]), ("after", e["alpha_first_late"])):
            plans.append(dict(seed=e["seed"], steps=e["steps"], phase=phase, alpha=alpha))
    terminal = max(e["alpha_first_late"] for e in events)
    zero_cases = [c for c in cases if c["steps"] == 0]
    zero_seeds = {min(c["seed"] for c in zero_cases)} | {c["seed"] for c in zero_cases if c["would_reject"]}
    plans += [dict(seed=seed, steps=0, phase="D0 terminal stress", alpha=terminal) for seed in sorted(zero_seeds)]
    return plans


def validate():
    cases, _ = load_cases()
    inputs = load_inputs()
    plans = validation_plan(cases)
    path = OUTPUT / "boundary_executions.jsonl"
    completed = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    assert [r["plan"] for r in completed] == plans[:len(completed)]
    expiry = b.ttl_expired_at_dispatch
    with path.open("a") as stream:
        for plan in plans[len(completed):]:
            check_memory()
            c = next(c for c in cases if (c["seed"], c["steps"]) == (plan["seed"], plan["steps"]))
            ready, rejected, reference = selected(c, plan["alpha"])
            dispatch, deadline = c["original"]["dispatch_step"], c["original"]["deadline_step"]
            scenario = b.Scenario(**c["original"]["scenario"])
            snapshot = inputs["snapshots"][c["seed"]]
            trace, physics, checks = [], [], []
            def checked_expiry(prediction, waited):
                assert ready and waited == c["steps"] and trace[-1][0] == dispatch
                checks.append(dispatch)
                return expiry(prediction, waited)
            availability = plan["alpha"] * c["computation_s"]
            with patch.object(b, "ttl_expired_at_dispatch", checked_expiry):
                outcome = b.run_actual(scenario, snapshot, dispatch, dispatch, deadline, False,
                                       recovery_mode="safety-only", trace=trace, physics_trace=physics,
                                       ttl_prediction=(c["prediction"], availability))
            assert checks == ([dispatch] if ready else [])
            assert outcome[2] == rejected and list(outcome) == reference["outcome"]
            observed_hash = trace_hash(trace, physics)
            assert observed_hash == reference["trace_sha256"]
            assert trace[dispatch][2][b.STATE_Z] == scenario.usual_height
            assert trace[dispatch+1][2][b.STATE_Z] == (scenario.usual_height if rejected else scenario.low_height)
            row = dict(plan=plan, simulated_availability_s=availability, prediction_ready=ready,
                       outcome=outcome, trace_sha256=observed_hash, expiry_check_steps=checks,
                       dispatch_step=dispatch, deadline_step=deadline,
                       first_unsafe_time_s=reference["first_unsafe_time_s"],
                       worker_peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
            stream.write(json.dumps(row) + "\n"); stream.flush()
            completed.append(row)
            del trace, physics
            gc.collect()
            LIBC.malloc_trim(0)
            print(f"Validated {len(completed)}/{len(plans)}: seed={c['seed']} "
                  f"D={c['delay_s']:.2f} {plan['phase']}, ready={ready}, outcome={outcome}", flush=True)
    freeze_and_check()
    metadata = dict(actual_boundary_flights=len(completed), all_trace_matches=True,
                    representatives="Every execution-changing transition; lowest eligible seed per delay for availability-only transitions; D0 ordinary/invalid TTL",
                    recorded_worker_peak_rss_mib=max(r["worker_peak_rss_mib"] for r in completed),
                    memory_limits_mib=dict(max_process_rss=2048, min_host_available=2048))
    (OUTPUT / "validation.json").write_text(json.dumps(metadata, indent=2) + "\n")
    if RESULT.exists():
        result = json.loads(RESULT.read_text())
        result["native_boundary_validation"] = metadata
        RESULT.write_text(json.dumps(result, indent=2) + "\n")
    print("PASS: boundary executions, original dispatch times, no late cancellation, and preserved inputs/results.")


def plot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    result = json.loads(RESULT.read_text())
    samples = [result["measured_baseline"]] + result["simulated_sweep"]
    x = [r["alpha"] for r in samples]
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 3.2), layout="constrained")
    for index, steps in enumerate(DELAY_STEPS):
        axes[0].step(x, [r["per_delay"][index]["ready"] for r in samples], where="post",
                     label=f"D={steps*20} ms", linewidth=1.5)
    axes[0].set(xlabel="Simulated slowdown α", ylabel="Predictions ready (out of 19)",
                title="Availability at dispatch", ylim=(-.5, 19.5), yticks=[0, 5, 10, 15, 19])
    axes[0].legend(fontsize=8, ncol=2)
    axes[1].step(x, [r["totals"]["latency_unsafe"] for r in samples], where="post",
                 color="tab:red", label="Stressed fail-open", linewidth=2)
    axes[1].scatter([x[-1]], [samples[-1]["totals"]["latency_unsafe"]], color="tab:red", zorder=5)
    axes[1].axhline(result["on_time_counterfactual"]["unsafe"], color="tab:blue",
                   linestyle="--", label="Paired on-time counterfactual")
    axes[1].scatter([1], [result["measured_baseline"]["totals"]["unsafe"]],
                    color="black", zorder=5, label="Recorded timing baseline (α=1)")
    axes[1].set(xlabel="Simulated slowdown α", ylabel="Latency-caused unsafe cases (out of 114)",
                title="Safety effect of unavailable TTLs", ylim=(-.2, 4.3), yticks=range(5))
    axes[1].legend(fontsize=8, loc="upper left")
    for ax in axes:
        ax.set_xlim(1, x[-1]+.08)
        ax.grid(alpha=.2)
    for extension in ("png", "pdf"):
        path = ROOT / f"result/computation_latency_stress.{extension}"
        fig.savefig(path, dpi=180)
        print(path)
    plt.close(fig)
    freeze_and_check()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("analyse", "validate", "plot"), required=True)
    args = parser.parse_args()
    with (ROOT / "results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        freeze_and_check()
        {"analyse": analyse, "validate": validate, "plot": plot}[args.stage]()
