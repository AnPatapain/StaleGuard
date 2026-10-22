"""Scenario-clustered statistics and six-policy reports for the locked test set."""
import csv
import json
import math
from pathlib import Path
from statistics import NormalDist

import numpy as np

from independent_evaluation import (ROOT, SEEDS, POLICIES, DELAY_STEPS, atomic_json,
                                    journal, check_memory)
import staleguard_benefit as b

FEATURES = ("inspection_completed", "unsafe", "fallbacks", "safe_completion",
            "safe_non_completion", "safe_fallbacks")


def cluster_bootstrap(values, replicates=10000, random_seed=1729):
    """Use identical scenario multiplicities for every delay, policy and metric."""
    values = np.asarray(values, dtype=float)
    n = len(values)
    if n == 0:
        return None
    rng = np.random.default_rng(random_seed)
    result = np.empty((replicates,)+values.shape[1:])
    for start in range(0, replicates, 128):
        count = min(128, replicates-start)
        weights = rng.multinomial(n, np.full(n, 1/n), size=count)
        result[start:start+count] = (weights @ values.reshape(n, -1) / n).reshape((count,)+values.shape[1:])
        if start % 1024 == 0:
            check_memory()
    return result


def interval(draws):
    if draws is None:
        return None
    finite = np.asarray(draws)[np.isfinite(draws)]
    return np.quantile(finite, [.025, .975]).tolist() if len(finite) else None


def wilson(successes, total):
    if total == 0:
        return None
    z = NormalDist().inv_cdf(.975)
    p = successes/total
    denominator = 1+z*z/total
    center = (p+z*z/(2*total))/denominator
    radius = z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/denominator
    return [max(0.0, center-radius), min(1.0, center+radius)]


def paired_sign_test(values):
    """Exact two-sided sign test across scenarios; zero differences are ties."""
    values = np.asarray(values)
    positive, negative = int(np.sum(values > 0)), int(np.sum(values < 0))
    n = positive+negative
    p = min(1.0, 2*sum(math.comb(n, k) for k in range(min(positive, negative)+1))/2**n) if n else 1.0
    return dict(p_value=p, positive_scenarios=positive, negative_scenarios=negative,
                tied_scenarios=len(values)-n)


def holm_adjust(rows):
    running = 0
    for rank, row in enumerate(sorted(rows, key=lambda r: r["p_value"])):
        running = max(running, min(1.0, (len(rows)-rank)*row["p_value"]))
        row["p_value_holm"] = running


def distribution(values):
    if not values:
        return dict(N=0, mean_s=None, median_s=None, p95_s=None, min_s=None, max_s=None)
    return dict(N=len(values), mean_s=float(np.mean(values)), median_s=float(np.median(values)),
                p95_s=float(np.percentile(values, 95)), min_s=float(min(values)), max_s=float(max(values)))


def metric_summary(values, draws):
    n = len(values)
    sums = values.reshape(-1, len(FEATURES)).sum(axis=0)
    denominator = values.size//len(FEATURES)
    result = dict(N=denominator)
    for index, key in enumerate(FEATURES):
        sample = draws[..., index].reshape(len(draws), -1).mean(axis=1) if draws is not None else None
        result[key] = dict(count=int(sums[index]), rate=float(sums[index]/denominator) if denominator else None,
                           ci95=interval(sample))
    fallback, safe = result["fallbacks"]["count"], result["safe_fallbacks"]["count"]
    ratio = None
    if draws is not None:
        triggers = draws[..., 2].reshape(len(draws), -1).mean(axis=1)
        successes = draws[..., 5].reshape(len(draws), -1).mean(axis=1)
        ratio = np.divide(successes, triggers, out=np.full_like(triggers, np.nan), where=triggers > 0)
    result["fallback_safety"] = dict(safe_count=safe, triggered=fallback,
        rate=safe/fallback if fallback else None, ci95=interval(ratio))
    any_unsafe = np.any(values[..., 1].reshape(n, -1), axis=1) if n else np.array([])
    result["scenario_any_unsafe"] = dict(count=int(any_unsafe.sum()), N=n,
        rate=float(any_unsafe.mean()) if n else None, ci95=wilson(int(any_unsafe.sum()), n),
        method="Wilson; one independent indicator per feasible scenario")
    return result


def write_csv(path, rows):
    if not rows:
        path.write_text("")
        return
    with path.open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def count_text(count, total):
    return b.format_count(count, total) if total else "N/A (0/0)"


def terminal_block(stream, delay, summaries, data, comparisons, feasible):
    n, width = feasible, 23
    print("="*171, file=stream)
    print(f"D = {delay:.1f}s | T = 4.2s | N = {n}", file=stream)
    print("="*171, file=stream)
    print("\nMISSION OUTCOMES", file=stream)
    print(f"{'':31}"+"".join(f"{policy:>{width}}" for policy in POLICIES), file=stream)
    for title, key in (("Safe mission completion", "safe_completion"),
                       ("Safe mission non-completion", "safe_non_completion"), ("Unsafe execution", "unsafe")):
        print(f"{title:<31}"+"".join(f"{count_text(summaries[p][key]['count'], n):>{width}}" for p in POLICIES), file=stream)
    print("\nSAFETY FALLBACK", file=stream)
    print(f"{'Fallback triggered':<31}"+"".join(f"{count_text(summaries[p]['fallbacks']['count'], n):>{width}}" for p in POLICIES), file=stream)
    print(f"{'Fallback safe (no unsafe entry)':<31}"+"".join(
        f"{count_text(summaries[p]['safe_fallbacks']['count'], summaries[p]['fallbacks']['count']):>{width}}" for p in POLICIES), file=stream)
    sg_index = POLICIES.index("StaleGuard")
    for policy in POLICIES:
        if policy == "StaleGuard":
            continue
        baseline = POLICIES.index(policy)
        sg, other = data[:, sg_index], data[:, baseline]
        sg_only = int(np.sum((sg[:, 3] == 1) & (other[:, 3] == 0)))
        other_only = int(np.sum((sg[:, 3] == 0) & (other[:, 3] == 1)))
        print(f"\nSTALEGUARD vs {policy.upper()}", file=stream)
        print("Paired safe mission completion", file=stream)
        if policy == "No Recovery":
            benefits = int(np.sum((sg[:, 2] == 1) & (sg[:, 3] == 1) & (other[:, 3] == 0)))
            harms = int(np.sum((sg[:, 2] == 1) & (sg[:, 3] == 0) & (other[:, 3] == 1)))
            print(f"{'Completion benefit':<36}{count_text(benefits, n)}", file=stream)
            print(f"{'Completion harm':<36}{count_text(harms, n)}", file=stream)
        print(f"{'StaleGuard-only successes':<36}{count_text(sg_only, n)}", file=stream)
        print(f"{policy+'-only successes':<36}{count_text(other_only, n)}", file=stream)
        for metric, title in (("safe_completion", "Safe mission completion difference"),
                              ("unsafe", "Unsafe execution difference"),
                              ("fallbacks", "Fallback frequency difference")):
            row = next(r for r in comparisons if r["baseline"] == policy and r["metric"] == metric)
            delta, ci = row["difference"], row["ci95"]
            text = (f"{100*delta:+.2f} pp; 95% CI [{100*ci[0]:+.2f}, {100*ci[1]:+.2f}]; "
                    f"Holm p={row['p_value_holm']:.4g}") if delta is not None else "N/A"
            print(f"{title:<38}{text}", file=stream)


def generate_report(output):
    protocol = json.loads((output / "protocol.json").read_text())
    calibrated = [json.loads((output / "scenarios" / str(seed) / "calibration.json").read_text()) for seed in SEEDS]
    feasible = [r["seed"] for r in calibrated if r["command"] is not None]
    infeasible = [r["seed"] for r in calibrated if r["command"] is None]
    assert len(calibrated) == 300 and len(feasible)+len(infeasible) == 300
    n = len(feasible)
    values = np.zeros((n, 11, 6, len(FEATURES)))
    seen = set()
    indices = {seed: index for index, seed in enumerate(feasible)}
    latency = {p: [] for p in ("StaleGuard", "Use-Time Recheck", "FBS")}
    latency_by_delay = {p: [[] for _ in range(11)] for p in latency}
    timing_sums = {p: np.zeros((n, 2)) for p in latency}
    availability = {p: [dict(ready=0, unavailable=0, dispatch_unreached=0) for _ in range(11)] for p in latency}
    decision_delays, application_delays, fbs_monitoring = [], [], []
    fbs_cold_seeds, fbs_ttls, check_to_use = [], {}, []
    execution_kinds = dict(native=0, verified_reference_reuse=0)
    with (output / "executions.jsonl").open("w") as merged:
        for seed in feasible:
            rows = journal(output / "scenarios" / str(seed) / "flights.jsonl")
            assert len(rows) == 66, (seed, len(rows))
            original = {r["delay_steps"]: r for r in rows if r["policy"] == "No Recovery"}
            received = json.loads((output / "scenarios" / str(seed) / "reception.json").read_text())
            latency["StaleGuard"].append(received["computation_s"])
            timing_sums["StaleGuard"][indices[seed]] = received["computation_s"], 1
            for row in rows:
                key = seed, row["delay_steps"], row["policy"]
                assert key not in seen
                seen.add(key)
                i, d, p = indices[seed], DELAY_STEPS.index(row["delay_steps"]), POLICIES.index(row["policy"])
                inspected, unsafe, fallback, complete = row["outcome"]
                assert complete == (inspected and not unsafe)
                values[i, d, p] = inspected, unsafe, fallback, complete, not inspected and not unsafe, fallback and not unsafe
                execution_kinds[row["execution_kind"]] += 1
                policy = row["policy"]
                if policy in availability:
                    state = availability[policy][d]
                    if row["prediction_ready"] is None:
                        state["dispatch_unreached"] += 1
                    elif row["prediction_ready"]:
                        state["ready"] += 1
                    else:
                        state["unavailable"] += 1
                duration = row["prediction_computation_s"]
                if policy in latency and duration is not None:
                    latency_by_delay[policy][d].append(duration)
                if policy in latency and policy != "StaleGuard" and duration is not None:
                    latency[policy].append(duration)
                    timing_sums[policy][i] += duration, 1
                if policy == "Use-Time Recheck" and row["event"]:
                    event = row["event"]
                    decision_delays.append((event["decision_step"]-row["dispatch_step"])/50)
                    if event["application_step"] is not None:
                        application_delays.append((event["application_step"]-row["dispatch_step"])/50)
                    if event["prediction"] == "SAFE" and unsafe and not original[row["delay_steps"]]["outcome"][1]:
                        check_to_use.append(dict(seed=seed, delay_s=row["delay_s"], first_unsafe_time_s=row["first_unsafe_time_s"]))
                if policy == "FBS":
                    event = row["event"]
                    prediction = event["prediction"]
                    if seed in fbs_ttls:
                        assert fbs_ttls[seed] == prediction
                    fbs_ttls[seed] = prediction
                    fbs_monitoring.extend(event["observation_update_s"][:-1])
                    if event["cold_start"] and row["delay_steps"] == 0:
                        fbs_cold_seeds.append(seed)
                merged.write(json.dumps(row)+"\n")
    assert len(seen) == n*66
    check_memory()
    boots = cluster_bootstrap(values, **{k: protocol["bootstrap"][k] for k in ("replicates", "random_seed")})
    aggregate = {policy: metric_summary(values[:, :, p], boots[:, :, p] if boots is not None else None)
                 for p, policy in enumerate(POLICIES)}
    per_delay = [dict(delay_s=steps/50, N=n, policies={policy: metric_summary(values[:, d, p], boots[:, d, p] if boots is not None else None)
                 for p, policy in enumerate(POLICIES)}) for d, steps in enumerate(DELAY_STEPS)]
    paired = []
    sg = POLICIES.index("StaleGuard")
    for baseline, policy in enumerate(POLICIES):
        if baseline == sg:
            continue
        for key in ("unsafe", "safe_completion", "fallbacks"):
            feature = FEATURES.index(key)
            diff = values[:, :, sg, feature]-values[:, :, baseline, feature]
            draws = boots[:, :, sg, feature]-boots[:, :, baseline, feature] if boots is not None else None
            for d in [None]+list(range(11)):
                scenario_values = diff.sum(axis=1) if d is None else diff[:, d]
                samples = draws.mean(axis=1) if d is None and draws is not None else draws[:, d] if draws is not None else None
                paired.append(dict(baseline=policy, metric=key, delay_s=None if d is None else d/10,
                    difference=float(diff.mean() if d is None else diff[:, d].mean()) if n else None,
                    ci95=interval(samples), **paired_sign_test(scenario_values)))
    holm_adjust(paired)
    computation = {}
    for p, policy in enumerate(latency):
        samples = cluster_bootstrap(timing_sums[policy], random_seed=1729+p)
        mean_draws = None
        if samples is not None:
            mean_draws = np.divide(samples[:, 0], samples[:, 1], out=np.full(len(samples), np.nan), where=samples[:, 1] > 0)
        computation[policy] = dict(**distribution(latency[policy]), mean_s_ci95=interval(mean_draws),
            availability=availability[policy],
            per_delay=[dict(delay_s=d/10, **distribution(samples)) for d, samples in enumerate(latency_by_delay[policy])],
            availability_clock="result applied at first control step after checking; command withheld" if policy == "Use-Time Recheck" else "scheduled dispatch; fail open if unavailable")
    computation["Use-Time Recheck"]["decision_delay"] = distribution(decision_delays)
    computation["Use-Time Recheck"]["safe_prediction_application_delay"] = distribution(application_delays)
    computation["FBS"]["earlier_observation_update_time"] = distribution(fbs_monitoring)
    feasibility = dict(attempted=300, feasible=n, infeasible=len(infeasible), feasibility_rate=n/300,
        ci95=wilson(n, 300), infeasible_seeds=infeasible, feasible_seeds=feasible,
        candidates=sum(r["candidates"] for r in calibrated),
        method="Wilson 95% interval over 300 scenario-level calibration outcomes")
    statistics = dict(bootstrap=protocol["bootstrap"],
        paired_tests="Exact two-sided sign tests of scenario-level differences; aggregate uses per-scenario counts across all 11 delays; ties excluded, all ties p=1",
        multiplicity=f"Holm adjustment jointly across all {len(paired)} aggregate/per-delay baseline×metric tests",
        interpretation="Outcome CIs condition on feasible calibration; a scenario, not a scenario-delay pair, is the resampling unit. Percentile CIs can be degenerate for zero observed events; complementary scenario-any-unsafe Wilson intervals are included.")
    result = dict(calibration=feasibility, aggregate=aggregate, per_delay=per_delay, paired=paired,
        computation=computation, statistics=statistics, execution_kinds=execution_kinds,
        fbs_cold_start_seeds=fbs_cold_seeds, fbs_ttls=fbs_ttls, check_to_use_unsafe=check_to_use,
        scientific_configuration=protocol)
    atomic_json(output / "summary.json", result)
    atomic_json(output / "calibration_feasibility.json", feasibility)
    atomic_json(output / "paired_comparisons.json", paired)
    atomic_json(output / "computation_times.json", computation)
    aggregate_rows, delay_rows = [], []
    for delay, policies in [(None, aggregate)]+[(r["delay_s"], r["policies"]) for r in per_delay]:
        for policy, metrics in policies.items():
            row = dict(policy=policy, delay_s=delay, N=metrics["N"])
            for feature in FEATURES:
                m = metrics[feature]
                row.update({feature+"_count": m["count"], feature+"_rate": m["rate"],
                            feature+"_ci_low": m["ci95"][0] if m["ci95"] else None,
                            feature+"_ci_high": m["ci95"][1] if m["ci95"] else None})
            (aggregate_rows if delay is None else delay_rows).append(row)
    write_csv(output / "aggregate.csv", aggregate_rows)
    write_csv(output / "per_delay.csv", delay_rows)
    write_csv(output / "paired_comparisons.csv", paired)
    with (output / "six_policy_report.log").open("w") as stream:
        print("INDEPENDENT EXPERIMENT B — SEEDS 20–319; FIXED TTL = 0.3s", file=stream)
        print(f"Calibration feasible: {n}/300 ({n/300:.2%}); Wilson 95% CI {feasibility['ci95']}", file=stream)
        print(f"Infeasible seeds (not replaced): {infeasible}", file=stream)
        print("Safety-only recovery; 1000 Hz actual monitoring; T=4.2s after approval.", file=stream)
        print("CIs resample entire scenarios, retaining all delays and policies together.", file=stream)
        print("Use-Time readiness is at its delayed application tick; StaleGuard/FBS readiness is at scheduled dispatch.", file=stream)
        for d, block in enumerate(per_delay):
            comparisons = [r for r in paired if r["delay_s"] == block["delay_s"]]
            terminal_block(stream, block["delay_s"], block["policies"], values[:, d], comparisons, n)
            for policy in latency:
                print(f"{policy} availability: {availability[policy][d]}", file=stream)
        print("\nAGGREGATE SIX-POLICY COMPARISON WITH SCENARIO-CLUSTER 95% CIs", file=stream)
        for policy, metrics in aggregate.items():
            print(policy, file=stream)
            for feature in ("safe_completion", "safe_non_completion", "unsafe", "fallbacks"):
                m = metrics[feature]
                print(f"  {feature}: {m['count']}/{metrics['N']}; rate={m['rate']}; 95% CI={m['ci95']}", file=stream)
            print(f"  fallback_safety: {metrics['fallback_safety']}", file=stream)
        print("\nCOMPUTATION-TIME SUMMARY (seconds)", file=stream)
        for policy in latency:
            print(f"{policy}: {distribution(latency[policy])}; clustered mean CI={computation[policy]['mean_s_ci95']}", file=stream)
        print("\nPAIRED AGGREGATE COMPARISONS (StaleGuard minus baseline)", file=stream)
        for row in paired:
            if row["delay_s"] is None:
                print(json.dumps(row), file=stream)
        print("\n"+json.dumps(statistics), file=stream)
    # Concise interpretation is descriptive, with no test-set parameter selection.
    lines = [f"Attempted exactly 300 new scenarios; {n} feasible and {len(infeasible)} infeasible; none replaced.",
             f"Each policy has {n*11} operational pairs from {n} scenario clusters."]
    for policy, metrics in aggregate.items():
        lines.append(f"{policy}: unsafe={metrics['unsafe']['count']}, safe completions={metrics['safe_completion']['count']}, safe non-completions={metrics['safe_non_completion']['count']}, fallbacks={metrics['fallbacks']['count']}.")
    lines += ["Intervals resample scenario clusters and preserve paired policies/delays; they condition on feasibility.",
              "These are noiseless deterministic-simulator results; computation timing is serial measured-time simulation, not concurrent onboard execution.",
              "No scientific parameter, fixed TTL, calibration rule, threshold or exclusion was changed using these outcomes."]
    (output / "interpretation.txt").write_text("\n".join(lines)+"\n")
    check_memory()
    plot_tradeoff(output, result)


def plot_tradeoff(output, result):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8.2, 4.8), layout="constrained")
    for policy, metrics in result["aggregate"].items():
        unsafe, complete = metrics["unsafe"], metrics["safe_completion"]
        if unsafe["rate"] is None:
            continue
        x, y = 100*unsafe["rate"], 100*complete["rate"]
        xerr = np.maximum(0, [[x-100*unsafe["ci95"][0]], [100*unsafe["ci95"][1]-x]])
        yerr = np.maximum(0, [[y-100*complete["ci95"][0]], [100*complete["ci95"][1]-y]])
        ax.errorbar([x], [y], xerr=xerr, yerr=yerr, fmt="o", capsize=3, label=policy)
    ax.set(xlabel="Unsafe execution (%) — lower is safer", ylabel="Safe mission completion (%)",
           title=f"Independent Experiment B: {result['calibration']['feasible']} feasible / 300 attempted scenarios\n95% scenario-cluster bootstrap intervals")
    ax.grid(alpha=.2)
    ax.legend(fontsize=8, loc="best")
    for extension in ("png", "pdf"):
        fig.savefig(output / f"safety_completion_tradeoff.{extension}", dpi=180)
    plt.close(fig)
