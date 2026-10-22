"""Fixed-TTL development sweep using verified trajectories and native timing checks.

No optimization or independent final-test scenes. A cancelled descent preserves
the nominal reference, making its physical path identical to the saved safety-only
fallback; validation checks the new dispatch event rather than assuming its timing.
"""
import argparse
import ast
import csv
import fcntl
import gc
import json
from pathlib import Path
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_corrected_experiment_b import ROOT, b, check_memory, digest, load_inputs, trace_hash, LIBC

OUTPUT = ROOT / "results/fixed_ttl"
RESULT = ROOT / "result/fixed_ttl_sweep.json"
SOURCE = ROOT / "results/use_time_recheck/executions.jsonl"
TTL_STEPS = tuple(range(0, 51, 5))


def check_preserved():
    for name, expected in json.loads((OUTPUT / "preserved_files.json").read_text()).items():
        if name != "staleguard_benefit.py":
            assert digest(ROOT / name) == expected, name
    # All five existing execution paths are structurally unchanged.
    before = ast.parse((OUTPUT / "staleguard_benefit.before.py").read_text())
    after = ast.parse((ROOT / "staleguard_benefit.py").read_text())
    old = next(n for n in before.body if isinstance(n, ast.FunctionDef) and n.name == "run_actual")
    new = next(n for n in after.body if isinstance(n, ast.FunctionDef) and n.name == "run_actual")
    assert [a.arg for a in new.args.args[-2:]] == ["fixed_ttl_steps", "fixed_ttl_log"]
    del new.args.args[-2:]
    del new.args.defaults[-2:]
    class StripFixedTTL(ast.NodeTransformer):
        def visit_If(self, node):
            if any(isinstance(n, ast.Name) and n.id == "fixed_ttl_steps" for n in ast.walk(node.test)):
                return None
            return self.generic_visit(node)
    assert ast.dump(old) == ast.dump(StripFixedTTL().visit(new))


def load_cases():
    rows = [json.loads(line) for line in SOURCE.read_text().splitlines()]
    saved = {(r["seed"], round(r["delay_s"]*b.CTRL_FREQ_HZ), r["policy"]): r for r in rows}
    calibrated = json.loads(b.CALIBRATION_FILE.read_text())
    seeds = sorted(int(seed) for seed, r in calibrated.items() if r["command"] is not None)
    assert len(seeds) == 19 and 16 not in seeds
    cases = []
    for seed in seeds:
        for steps in TTL_STEPS:  # The same 0–1 s grid is used for dispatch delay.
            original = saved[seed, steps, "No Recovery"]
            fallback = saved[seed, steps, "Always-Recover"]
            assert original["scenario"] == calibrated[str(seed)]["command"] == fallback["scenario"]
            assert original["dispatch_step"] == fallback["dispatch_step"]
            assert original["deadline_step"] == fallback["deadline_step"]
            if steps:
                assert fallback["outcome"] == [False, False, True, False]
            cases.append(dict(seed=seed, steps=steps, original=original, fallback=fallback))
    assert len(cases) == 209
    return cases


def selected(case, ttl_steps):
    expired = b.fixed_ttl_expired_at_dispatch(ttl_steps, case["steps"])
    return expired, case["fallback"] if expired else case["original"]


def counts(rows):
    n = len(rows)
    result = dict(N=n, unsafe=sum(r["outcome"][1] for r in rows),
                  safe_completion=sum(r["outcome"][3] for r in rows),
                  safe_non_completion=sum(not r["outcome"][0] and not r["outcome"][1] for r in rows),
                  fallbacks=sum(r["outcome"][2] for r in rows))
    assert result["unsafe"] + result["safe_completion"] + result["safe_non_completion"] == n
    for key in ("unsafe", "safe_completion", "safe_non_completion", "fallbacks"):
        result[key+"_rate"] = result[key]/n
    return result


def analyse():
    cases = load_cases()
    sweep, executions = [], []
    for ttl in TTL_STEPS:
        rows = []
        for case in cases:
            expired, reference = selected(case, ttl)
            assert reference["outcome"][2] == expired
            row = dict(ttl_s=ttl/b.CTRL_FREQ_HZ, seed=case["seed"],
                       delay_s=case["steps"]/b.CTRL_FREQ_HZ, expired=expired,
                       outcome=reference["outcome"], reference_policy=reference["policy"],
                       trace_sha256=reference["trace_sha256"],
                       dispatch_step=reference["dispatch_step"], deadline_step=reference["deadline_step"])
            rows.append(row)
        summary = dict(ttl_s=ttl/b.CTRL_FREQ_HZ, ttl_steps=ttl, **counts(rows))
        assert summary["fallbacks"] == 19*(10-ttl//5)
        summary["per_delay"] = [dict(delay_s=steps/b.CTRL_FREQ_HZ,
            **counts([r for r in rows if r["delay_s"] == steps/b.CTRL_FREQ_HZ])) for steps in TTL_STEPS]
        sweep.append(summary)
        executions.extend(rows)
    assert len(executions) == 2299
    assert all(a["unsafe"] <= z["unsafe"] for a, z in zip(sweep, sweep[1:]))
    assert all(a["safe_completion"] <= z["safe_completion"] for a, z in zip(sweep, sweep[1:]))
    # This comparison is added only after the complete prespecified sweep.
    comparison = json.loads((ROOT / "result/fbs.json").read_text())["totals"]
    result = dict(experiment="Experiment B: Fixed TTL development sweep", development_seeds=19,
        excluded_seeds=[16], pairs_per_ttl=209, reused_policy_outcomes=2299,
        mission_deadline_after_approval_s=b.MISSION_DEADLINE_S,
        rule="Execute iff waited_steps <= ttl_steps; otherwise safety-only fallback at dispatch",
        trajectory_reuse="Saved No Recovery when accepted; equivalent nominal safety-only fallback when expired. Decision timing verified separately by native executions.",
        sweep=sweep, existing_policy_comparison=comparison, selected_ttl=None,
        independent_final_test_executed=False)
    validation = OUTPUT / "validation.json"
    if validation.exists():
        result["native_validation"] = json.loads(validation.read_text())
    RESULT.write_text(json.dumps(result, indent=2)+"\n")
    (OUTPUT / "sweep_executions.jsonl").write_text("".join(json.dumps(r)+"\n" for r in executions))
    keys = [k for k in sweep[0] if k != "per_delay"]
    with (ROOT / "result/fixed_ttl_sweep.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, extrasaction="ignore")
        writer.writeheader(); writer.writerows(sweep)
    print("TTL(s) | Unsafe | Safe completion | Safe non-completion | Fallbacks; N=209")
    for row in sweep:
        print(f"{row['ttl_s']:.1f} | {row['unsafe']} ({row['unsafe_rate']:.2%}) | "
              f"{row['safe_completion']} ({row['safe_completion_rate']:.2%}) | "
              f"{row['safe_non_completion']} ({row['safe_non_completion_rate']:.2%}) | {row['fallbacks']}")
    check_preserved()


def validation_plan(cases):
    seeds = sorted({c["seed"] for c in cases})
    # Prespecified timing coverage; no selection by safety or completion outcome.
    plans = [dict(seed=seeds[0], ttl_steps=ttl, delay_steps=delay)
             for ttl in TTL_STEPS for delay in TTL_STEPS if abs(delay-ttl) <= 5]
    shifted = next(seed for seed in seeds if next(c for c in cases if c["seed"] == seed)["original"]["scenario"]["approval_time"] > 0)
    plans += [dict(seed=shifted, ttl_steps=25, delay_steps=delay) for delay in (20, 25, 30)]
    assert len(plans) == 34
    return plans


def validate():
    cases, inputs = load_cases(), load_inputs()
    plans = validation_plan(cases)
    sources = [ROOT / "staleguard_benefit.py", Path(__file__), OUTPUT / "protocol.json"]
    hashes = {str(path.relative_to(ROOT)): digest(path) for path in sources}
    hash_path = OUTPUT / "implementation_hashes.json"
    if hash_path.exists():
        assert json.loads(hash_path.read_text()) == hashes, "Source changed since timing checkpoints"
    else:
        hash_path.write_text(json.dumps(hashes, indent=2)+"\n")
    path = OUTPUT / "boundary_executions.jsonl"
    completed = [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []
    assert [r["plan"] for r in completed] == plans[:len(completed)]
    expiry = b.fixed_ttl_expired_at_dispatch
    with path.open("a") as stream:
        for plan in plans[len(completed):]:
            rss, available = check_memory()
            case = next(c for c in cases if c["seed"] == plan["seed"] and c["steps"] == plan["delay_steps"])
            expired, reference = selected(case, plan["ttl_steps"])
            current = inputs["snapshots"][case["seed"]]
            dispatch, deadline = reference["dispatch_step"], reference["deadline_step"]
            approval = round(current.approval_time*b.CTRL_FREQ_HZ)
            trace, physics, checks, event = [], [], [], {}
            def checked(ttl, waited):
                checks.append(trace[-1][0])
                assert ttl == plan["ttl_steps"] and waited == plan["delay_steps"]
                assert trace[-1][0] == dispatch
                return expiry(ttl, waited)
            with patch.object(b, "fixed_ttl_expired_at_dispatch", side_effect=checked):
                outcome = b.run_actual(b.Scenario(**reference["scenario"]), current,
                    dispatch, dispatch, deadline, False, trace=trace, physics_trace=physics,
                    fixed_ttl_steps=plan["ttl_steps"], fixed_ttl_log=event)
            assert event["approval_step"] == approval
            assert event["expiry_step"] == approval+plan["ttl_steps"]
            assert checks == [dispatch] and outcome[2] == expired
            assert event["application_step"] == (None if expired else dispatch)
            assert event["fallback_step"] == (dispatch if expired else None)
            assert deadline == round((current.approval_time+b.MISSION_DEADLINE_S)*b.CTRL_FREQ_HZ)
            assert trace_hash(trace[:dispatch+1], []) == case["original"]["prefix_sha256"]
            assert trace[dispatch+1][2][b.STATE_Z] == (reference["scenario"]["usual_height"] if expired else current.low_height)
            assert physics[dispatch*20][0] == dispatch*20
            assert list(outcome) == reference["outcome"]
            observed_hash = trace_hash(trace, physics)
            assert observed_hash == reference["trace_sha256"]
            row = dict(plan=plan, decision=event, outcome=outcome, trace_sha256=observed_hash,
                       reference_trace_match=True, deadline_step=deadline,
                       worker_rss_mib=rss, host_available_mib=available)
            stream.write(json.dumps(row)+"\n"); stream.flush()
            completed.append(row)
            del trace, physics
            gc.collect(); LIBC.malloc_trim(0)
            print(f"Validated {len(completed)}/34: seed={case['seed']}, "
                  f"TTL={plan['ttl_steps']/50:.1f}s, D={plan['delay_steps']/50:.1f}s, expired={expired}", flush=True)
    metadata = dict(actual_pybullet_flights=len(completed), all_trace_matches=True,
        accepted=sum(not r["decision"]["expired"] for r in completed),
        fallbacks=sum(r["decision"]["expired"] for r in completed),
        max_worker_rss_mib=max(r["worker_rss_mib"] for r in completed),
        min_host_available_mib=min(r["host_available_mib"] for r in completed),
        memory_limits_mib=dict(max_process_rss=2048, min_host_available=2048),
        existing_executor_ast_unchanged=True,
        coverage="Adjacent development-grid delays before/at/after every TTL, where in range; shifted-approval seed at TTL=0.5s. No independent final-test scenes.")
    (OUTPUT / "validation.json").write_text(json.dumps(metadata, indent=2)+"\n")
    check_preserved()
    print("PASS: all 34 native dispatch-boundary flights and preserved previous files.")


def plot():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    result = json.loads(RESULT.read_text())
    sweep = result["sweep"]
    fig, ax = plt.subplots(figsize=(7.4, 4.1), layout="constrained")
    ax.plot([100*r["unsafe_rate"] for r in sweep],
            [100*r["safe_completion_rate"] for r in sweep], "o-", label="Fixed TTL sweep", linewidth=1.5)
    for row in sweep:
        ax.annotate(f"{row['ttl_s']:.1f} s", (100*row["unsafe_rate"], 100*row["safe_completion_rate"]),
                    xytext=(5, -13 if row["ttl_s"] >= .5 else 5), textcoords="offset points", fontsize=8)
    staleguard = result["existing_policy_comparison"]["StaleGuard"]
    ax.scatter([100*staleguard["unsafe"]/staleguard["N"]],
               [100*staleguard["safe_completion"]/staleguard["N"]],
               marker="*", s=150, color="tab:red", label="Saved StaleGuard result", zorder=5)
    ax.set(xlabel="Unsafe execution (%) — lower is safer",
           ylabel="Safe mission completion (%)", xlim=(-2, 67), ylim=(6, 33),
           title="Fixed TTL: safety–completion trade-off\nDevelopment scenarios; 209 scenario–delay pairs per TTL")
    ax.grid(alpha=.2)
    ax.legend(loc="lower right", fontsize=9)
    for extension in ("png", "pdf"):
        path = ROOT / f"result/fixed_ttl_tradeoff.{extension}"
        fig.savefig(path, dpi=180)
        print(path)
    plt.close(fig)
    check_preserved()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("analyse", "validate", "plot"), required=True)
    args = parser.parse_args()
    check_preserved()
    with (ROOT / "results/always_recover/.validation.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        {"analyse": analyse, "validate": validate, "plot": plot}[args.stage]()
