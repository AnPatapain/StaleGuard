"""Recompute E01 tables and matched whole-scenario uncertainty from primary records."""

from collections import Counter, defaultdict
import csv
import json
import time

import numpy as np

from staleguard.reproduce.common import profile_metrics
from staleguard.underpass_inspection.baselines.staleguard import first_ready_step
from .protocol import (
    ROOT,
    CONDITIONS,
    POLICIES,
    LABELS,
    BOOTSTRAP_SEED,
    atomic_json,
    load_record,
    digest,
    file_hash,
    perturbation,
    plant_name,
    environment,
)


def csv_table(name, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with (ROOT / (name + '.csv')).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    lines = ['| ' + ' | '.join(fields) + ' |', '| ' + ' | '.join('---' for _ in fields) + ' |']
    for row in rows:
        lines.append(
            '| '
            + ' | '.join('N/M' if row.get(k) is None else str(row.get(k, '')) for k in fields)
            + ' |'
        )
    (ROOT / (name + '.md')).write_text('\n'.join(lines) + '\n')


def collect(pattern):
    return [load_record(p) for p in sorted(ROOT.glob(pattern))]


def quantile(values):
    return [float(v) for v in np.quantile(values, [0.025, 0.975])]


def summarize(labels=None):
    labels = LABELS if labels is None else labels
    spec = json.loads((ROOT / 'specification.json').read_text())
    forecasts = collect('forecasts/*/*.json')
    physical = collect('physical/*/*/*.json')
    decisions = collect('decisions/*/*/*.json')
    outcomes = collect('outcomes/*/*/*.json')
    truth_records = collect('h2/*/*.json')
    f_index = {(r['condition'], r['seed']): r['forecast'] for r in forecasts}
    o_index = {
        (r['condition'], r['seed'], round(r['delay_s'] * 50), r['policy']): r for r in outcomes
    }
    if len(o_index) != len(outcomes) or len(f_index) != len(forecasts):
        raise ValueError('Duplicate scientific keys')
    valid_seeds = set(spec['selected_seeds'])
    forecast_keys = {(c, s) for c in CONDITIONS for s in valid_seeds}
    if not set(f_index) <= forecast_keys:
        raise ValueError('Unexpected forecast scenario/condition')
    for row in forecasts:
        forecast = row['forecast']
        if forecast['grid_s'] != spec['prediction_delays']:
            raise ValueError('Forecast candidate grid changed')
        if [r['delay_s'] for r in forecast['results']] != spec['prediction_delays']:
            raise ValueError('Forecast labels do not match candidate grid')
        if row['perturbation'] != perturbation(spec, row['condition'], row['seed']):
            raise ValueError('Forecast perturbation differs from frozen assignment')
    decision_index = {
        (r['condition'], r['seed'], round(r['delay_s'] * 50), r['policy']): r for r in decisions
    }
    if len(decision_index) != len(decisions):
        raise ValueError('Duplicate policy decision key')
    delays = [round(d * 50) for d in spec['policy_delays']]
    expected = {
        (c, s, d, p) for c in CONDITIONS for s in valid_seeds for d in delays for p in POLICIES
    }
    if not set(o_index) <= expected or not set(decision_index) <= expected:
        raise ValueError('Unexpected scenario/delay/policy/condition')
    for row in outcomes:
        key = row['condition'], row['seed'], round(row['delay_s'] * 50), row['policy']
        decision = decision_index[key]
        forecast_path = ROOT / 'forecasts' / row['condition'] / f'{row["seed"]}.json'
        if decision['forecast_sha256'] != file_hash(forecast_path):
            raise ValueError('Decision does not use the recorded forecast')
        if row['safe_completion'] != (row['safe'] and row['completed']):
            raise ValueError('Inconsistent physical outcome')
        source = load_record(ROOT / row['physical_cache_id'])
        assignment = perturbation(spec, row['condition'], row['seed'])
        if (
            source['key']['action'] != decision['action']
            or source['key']['scenario']['seed'] != row['seed']
            or source['key']['mass_factor'] != assignment['mass_factor']
            or not row['physical_cache_id'].startswith(
                'physical/' + plant_name(row['condition']) + '/'
            )
        ):
            raise ValueError('Policy action/physical input mapping mismatch')
        if source['record']['physics_sha256'] != row['physics_sha256']:
            raise ValueError('Outcome/physical hash mismatch')
        if source['record']['safe'] != row['safe']:
            raise ValueError('Outcome/physical label mismatch')
        if source['record']['safe_completion'] != row['safe_completion']:
            raise ValueError('Outcome/physical completion mismatch')
    truth_keys = {(r['condition'], r['seed']) for r in truth_records}
    if len(truth_keys) != len(truth_records) or not truth_keys <= set(f_index):
        raise ValueError('Duplicate, unexpected or forecast-unmatched H2 record')
    h2_rows, h2_details = [], {}
    for condition in CONDITIONS:
        records = [r for r in truth_records if r['condition'] == condition]
        seeds = sorted(r['seed'] for r in records)
        if not seeds:
            continue
        truth = {}
        for record in records:
            keys = set()
            for row in record['comparisons']:
                key = (
                    record['seed'],
                    'command' if row['mode'] == 'dive' else 'recovery',
                    round(row['delay_s'] * 50),
                )
                if key in keys:
                    raise ValueError('Duplicate H2 physical key')
                keys.add(key)
                source = load_record(ROOT / row['physical_cache_id'])
                expected_plant = condition if condition in ('A1', 'A2') else 'N0'
                if not row['physical_cache_id'].startswith('physical/' + expected_plant + '/'):
                    raise ValueError('Wrong condition-specific H2 ground truth')
                assert source['record']['safe'] == row['safe']
                assert source['record']['safe_completion'] == row['safe_completion']
                assert source['record']['physics_sha256'] == row['physics_sha256']
                assert (
                    source['key']['mass_factor']
                    == perturbation(spec, condition, record['seed'])['mass_factor']
                )
                assert source['key']['action'] == [
                    row['mode'],
                    source['record']['approval_step'] + round(row['delay_s'] * 50),
                ]
                truth[key] = row
            assert keys == {
                (record['seed'], mode, round(delay * 50))
                for mode in ('command', 'recovery')
                for delay in spec['prediction_delays']
            }
        all_seeds = seeds
        h2_details[condition] = {}
        for sign in ('combined', '+', '-') if condition in ('A1', 'A2') else ('combined',):
            seeds = (
                all_seeds
                if sign == 'combined'
                else [
                    seed
                    for seed in all_seeds
                    if (spec['mass_signs'][str(seed)] > 0) == (sign == '+')
                ]
            )
            if not seeds:
                continue
            fs = {str(s): f_index[condition, s] for s in seeds}
            metrics, ready, profiles = profile_metrics(fs, truth, spec['prediction_delays'], seeds)
            available = sum(
                first_ready_step(fs[str(s)]['computation_s']) <= d for s in seeds for d in delays
            )
            for opportunity, m in metrics.items():
                h2_rows.append(
                    dict(
                        condition=condition,
                        mass_sign=sign,
                        opportunity=opportunity,
                        scenarios=len(seeds),
                        comparisons=m['physical_comparisons'],
                        accuracy_pct=100 * m['accuracy'],
                        false_feasible=m['counts']['fp'],
                        false_feasible_pct=(
                            None
                            if m['false_feasible_rate'] is None
                            else 100 * m['false_feasible_rate']
                        ),
                        false_reject=m['counts']['fn'],
                        false_reject_pct=(
                            None if m['false_reject_rate'] is None else 100 * m['false_reject_rate']
                        ),
                        boundary_mae_ms=(
                            None if m['boundary_mae_s'] is None else 1000 * m['boundary_mae_s']
                        ),
                        boundary_median_ae_ms=(
                            None
                            if m['boundary_median_absolute_error_s'] is None
                            else 1000 * m['boundary_median_absolute_error_s']
                        ),
                        boundary_pairs=m['boundary_pairs'],
                        physical_censored=m['right_censored'].get('physical', 0),
                        predicted_censored=m['right_censored'].get('predicted', 0),
                        physical_initially_infeasible=m['initial_cases'].get('physical', 0),
                        predicted_initially_infeasible=m['initial_cases'].get('predicted', 0),
                        physical_nonmonotonic=m['non_monotonic'].get('physical', 0),
                        predicted_nonmonotonic=m['non_monotonic'].get('predicted', 0),
                        runtime_median_ms=1000 * ready['runtime_median_s'],
                        runtime_p95_ms=1000 * ready['runtime_p95_s'],
                        ready_by_dispatch=available,
                        dispatch_comparisons=len(seeds) * 13,
                        observed_recovery_losses=ready['observed_physical_recovery_boundaries'],
                        late_at_recovery_loss=ready['physical_readiness'].get(
                            'late_at_or_after_first_unsafe_sample', 0
                        ),
                        ready_by_last_safe_sample=ready['physical_readiness'].get(
                            'ready_by_last_safe_sample', 0
                        ),
                        status=(
                            'COMPLETE'
                            if len(seeds) == (64 if sign == 'combined' else 32)
                            else 'PARTIAL'
                        ),
                    )
                )
            h2_details[condition][sign] = dict(metrics=metrics, readiness=ready, scenario_ids=seeds)
    csv_table('table_E01_A', h2_rows)
    atomic_json(ROOT / 'h2_metrics.json', h2_details)

    policy_rows = []
    for condition in CONDITIONS:
        for policy in POLICIES:
            group = [r for r in outcomes if r['condition'] == condition and r['policy'] == policy]
            for sign in ('combined', '+', '-') if condition in ('A1', 'A2') else ('combined',):
                rows = (
                    group
                    if sign == 'combined'
                    else [r for r in group if (r['mass_sign'] > 0) == (sign == '+')]
                )
                if not rows:
                    continue
                n = len(rows)
                seeds = sorted(set(r['seed'] for r in rows))
                tested = sorted(set(r['delay_s'] for r in rows))
                policy_rows.append(
                    dict(
                        condition=condition,
                        policy=policy,
                        label=labels[policy],
                        mass_sign=sign,
                        scenarios=len(seeds),
                        scenario_ids=json.dumps(seeds),
                        delays=json.dumps(tested),
                        outcomes=n,
                        unsafe=sum(not r['safe'] for r in rows),
                        unsafe_pct=100 * sum(not r['safe'] for r in rows) / n,
                        safe_completions=sum(r['safe_completion'] for r in rows),
                        safe_completion_pct=100 * sum(r['safe_completion'] for r in rows) / n,
                        safe_non_completion=sum(r['safe_non_completion'] for r in rows),
                        recovery_attempts=sum(r['recovery_attempted'] for r in rows),
                        unsafe_recoveries=sum(r['unsafe_recovery'] for r in rows),
                        recovery_failure_categories=json.dumps(
                            dict(
                                Counter(
                                    r['recovery_failure_category']
                                    for r in rows
                                    if r['recovery_failure_category']
                                )
                            )
                        ),
                        zero_delay_unsafe=sum(r['delay_s'] == 0 and not r['safe'] for r in rows),
                        zero_delay_incomplete=sum(
                            r['delay_s'] == 0 and not r['completed'] for r in rows
                        ),
                        pre_approval_violations=sum(r['violation_before_approval'] for r in rows),
                        pending_violations=sum(r['violation_while_pending'] for r in rows),
                        status=(
                            'COMPLETE'
                            if n == (64 if sign == 'combined' else 32) * 13
                            else 'PARTIAL'
                        ),
                    )
                )
    csv_table('table_E01_B', policy_rows)

    comparisons = []
    full = 'staleguard_dual_horizon'
    for comparator in POLICIES[:-1]:
        available = [
            c
            for c in CONDITIONS
            if any(r['condition'] == c and r['policy'] == comparator for r in outcomes)
            and any(r['condition'] == c and r['policy'] == full for r in outcomes)
        ]
        if not available:
            continue
        complete_pairs = {
            (s, d)
            for s in spec['execution_order']
            for d in delays
            if all((c, s, d, p) in o_index for c in available for p in (full, comparator))
        }
        tested = [d for d in delays if any((s, d) in complete_pairs for s in valid_seeds)]
        seeds = [
            s
            for s in spec['execution_order']
            if tested and all((s, d) in complete_pairs for d in tested)
        ]
        if not seeds:
            continue
        sampled = np.random.default_rng(BOOTSTRAP_SEED).integers(0, len(seeds), (2000, len(seeds)))
        advantages = {}
        for c in available:
            unsafe = np.array(
                [
                    [
                        float(not o_index[c, s, d, comparator]['safe'])
                        - float(not o_index[c, s, d, full]['safe'])
                        for d in tested
                    ]
                    for s in seeds
                ]
            )
            completion = np.array(
                [
                    [
                        float(o_index[c, s, d, full]['safe_completion'])
                        - float(o_index[c, s, d, comparator]['safe_completion'])
                        for d in tested
                    ]
                    for s in seeds
                ]
            )
            advantages[c] = (unsafe.mean(axis=1), completion.mean(axis=1))
        for c in available:
            u, q = advantages[c]
            u_ci, q_ci = quantile(100 * u[sampled].mean(axis=1)), quantile(
                100 * q[sampled].mean(axis=1)
            )
            baseline = np.mean(
                [not o_index[c, s, d, comparator]['safe'] for s in seeds for d in tested]
            )
            matched_full = [o_index[c, s, d, full] for s in seeds for d in tested]
            matched_other = [o_index[c, s, d, comparator] for s in seeds for d in tested]
            delta_u = u - advantages['N0'][0] if 'N0' in advantages else None
            delta_q = q - advantages['N0'][1] if 'N0' in advantages else None
            comparisons.append(
                dict(
                    condition=c,
                    comparator=comparator,
                    scenarios=len(seeds),
                    scenario_ids=json.dumps(seeds),
                    delays=json.dumps([d / 50 for d in tested]),
                    matched_trials=len(seeds) * len(tested),
                    full_unsafe_pct=100 * np.mean([not r['safe'] for r in matched_full]),
                    comparator_unsafe_pct=100 * baseline,
                    full_safe_completion_pct=100
                    * np.mean([r['safe_completion'] for r in matched_full]),
                    comparator_safe_completion_pct=100
                    * np.mean([r['safe_completion'] for r in matched_other]),
                    unsafe_reduction_pp=100 * float(u.mean()),
                    unsafe_reduction_ci_low=u_ci[0],
                    unsafe_reduction_ci_high=u_ci[1],
                    completion_change_pp=100 * float(q.mean()),
                    completion_ci_low=q_ci[0],
                    completion_ci_high=q_ci[1],
                    relative_unsafe_reduction_pct=(
                        None if baseline == 0 else 100 * float(u.mean()) / baseline
                    ),
                    unsafe_prevented=sum(
                        not b['safe'] and a['safe'] for a, b in zip(matched_full, matched_other)
                    ),
                    unsafe_introduced=sum(
                        b['safe'] and not a['safe'] for a, b in zip(matched_full, matched_other)
                    ),
                    safe_completions_sacrificed=sum(
                        b['safe_completion'] and not a['safe_completion']
                        for a, b in zip(matched_full, matched_other)
                    ),
                    safe_completions_gained=sum(
                        a['safe_completion'] and not b['safe_completion']
                        for a, b in zip(matched_full, matched_other)
                    ),
                    advantage_change_vs_N0_pp=(
                        None if delta_u is None else 100 * float(delta_u.mean())
                    ),
                    advantage_change_vs_N0_ci=(
                        None
                        if delta_u is None
                        else json.dumps(quantile(100 * delta_u[sampled].mean(axis=1)))
                    ),
                    completion_advantage_change_vs_N0_pp=(
                        None if delta_q is None else 100 * float(delta_q.mean())
                    ),
                    completion_advantage_change_vs_N0_ci=(
                        None
                        if delta_q is None
                        else json.dumps(quantile(100 * delta_q[sampled].mean(axis=1)))
                    ),
                    status='COMPLETE' if len(seeds) == 64 and len(tested) == 13 else 'PARTIAL',
                )
            )
    csv_table('table_E01_C', comparisons)
    atomic_json(ROOT / 'paired_comparisons.json', comparisons)

    attempts = collect('attempts/*/*.json')
    execution = collect('execution/*.json')
    statistics = []
    for c in CONDITIONS:
        rows = [r for r in outcomes if r['condition'] == c]
        refs = {r['physical_cache_id'] for r in rows}
        for truth in truth_records:
            if truth['condition'] == c:
                refs.update(r['physical_cache_id'] for r in truth['comparisons'])
        calls = [r for r in attempts if r['uncertainty']['condition'] == c]
        predictions = [r for r in forecasts if r['condition'] == c]
        complete_seeds = [
            s
            for s in spec['execution_order']
            if all((c, s, d, p) in o_index for d in delays for p in POLICIES)
        ]
        stats = [r for r in execution if r['condition'] == c]
        statistics.append(
            dict(
                condition=c,
                scenarios_with_any_outcome=len({r['seed'] for r in rows}),
                scenarios_completed_full_13_delays=len(complete_seeds),
                complete_scenario_ids=json.dumps(complete_seeds),
                policy_outcomes=len(rows),
                predictions_computed=len(predictions),
                unique_physical_continuations_referenced=len(refs),
                actual_new_physical_rollouts=len(calls),
                physical_rollouts_reused=sum(r['physical_reused'] for r in rows),
                cache_efficiency_pct=(
                    None if not rows else 100 * sum(r['physical_reused'] for r in rows) / len(rows)
                ),
                simulation_runtime_s=sum(r['record']['physical_runtime_s'] for r in calls),
                mean_physical_rollout_s=(
                    None
                    if not calls
                    else float(np.mean([r['record']['physical_runtime_s'] for r in calls]))
                ),
                worker_peak_rss_mb=max(
                    (r['record']['worker_peak_rss_mb'] for r in calls), default=None
                ),
                prediction_runtime_s=sum(r['forecast']['computation_s'] for r in predictions),
                h2_physical_generation_wall_s=sum(r['h2_physical_runtime_s'] for r in stats),
                jit_warmup_full_forecasts=sum(r['warmup_full_forecasts'] for r in stats),
            )
        )
    csv_table('table_E01_D', statistics)

    nominal_comparisons = []
    for seed in sorted({r['seed'] for r in outcomes if r['condition'] == 'N0'}):
        archived = load_record(ROOT / 'archive' / f'{seed}.json')
        index = {(round(r['delay_s'] * 50), r['event']['policy']): r for r in archived['h3']}
        for row in [r for r in outcomes if r['condition'] == 'N0' and r['seed'] == seed]:
            old = index[round(row['delay_s'] * 50), row['policy']]
            physical_changed = row['physics_sha256'] != old['physics_sha256']
            decision_changed = (row['application_step'], row['fallback_step']) != (
                old['event']['application_step'],
                old['event']['fallback_step'],
            )
            nominal_comparisons.append(
                dict(
                    seed=seed,
                    policy=row['policy'],
                    delay_s=row['delay_s'],
                    archived_safe=old['safe'],
                    new_safe=row['safe'],
                    archived_safe_completion=old['safe_completion'],
                    new_safe_completion=row['safe_completion'],
                    action_changed=decision_changed,
                    trajectory_changed=physical_changed,
                    outcome_changed=(row['safe'], row['safe_completion'])
                    != (old['safe'], old['safe_completion']),
                    explanation=(
                        'Different measured readiness; E01 shares one freshly timed full forecast across StaleGuard ablations; archived command-aware used its command-only timing. Use-Time Recheck is freshly timed.'
                        if decision_changed
                        else (
                            'ERROR: same action but different physical trajectory'
                            if physical_changed
                            else 'Exact nominal physical equivalence'
                        )
                    ),
                )
            )
    csv_table('nominal_archive_comparison', nominal_comparisons)
    if any(r['trajectory_changed'] and not r['action_changed'] for r in nominal_comparisons):
        raise ValueError('N0 physical mismatch with identical action; investigate before reporting')
    for name, records in (
        ('forecasts', forecasts),
        ('physical_rollouts', physical),
        ('policy_decisions', decisions),
        ('policy_outcomes', outcomes),
    ):
        with (ROOT / (name + '.jsonl')).open('w') as stream:
            for row in records:
                stream.write(json.dumps(row, allow_nan=False) + '\n')
    complete = len(outcomes) == 24960 and len(forecasts) == 320 and len(truth_records) == 320
    validation = dict(
        status='COMPLETE' if complete else 'PARTIAL',
        outcomes=len(outcomes),
        expected_outcomes=24960,
        forecasts=len(forecasts),
        h2_scenario_conditions=len(truth_records),
        duplicate_keys=0,
        scenario_selection=spec['selected_seeds'],
        nominal_outcome_differences=sum(r['outcome_changed'] for r in nominal_comparisons),
        nominal_action_differences=sum(r['action_changed'] for r in nominal_comparisons),
        bootstrap_seed=BOOTSTRAP_SEED,
        bootstrap_resamples=2000,
    )
    atomic_json(ROOT / 'VALIDATION.json', validation)
    primary_paths = []
    for pattern in (
        'forecasts/*/*.json',
        'physical/*/*/*.json',
        'decisions/*/*/*.json',
        'outcomes/*/*/*.json',
        'h2/*/*.json',
        'snapshots/*/*.json',
        'snapshots/*/*.npz',
        'attempts/*/*.json',
        'archive/*.json',
        'invocations/*.json',
        'numerical_parameters.json',
    ):
        primary_paths.extend(sorted(ROOT.glob(pattern)))
    atomic_json(
        ROOT / 'manifest.json',
        dict(
            aggregation_provenance=environment(),
            specification_sha256=file_hash(ROOT / 'specification.json'),
            selection_sha256=file_hash(ROOT / 'selection.json'),
            implementation_sha256=file_hash(ROOT / 'implementation.json'),
            simulator_assets_sha256=(
                file_hash(ROOT / 'simulator_assets.json')
                if (ROOT / 'simulator_assets.json').exists()
                else None
            ),
            validation=validation,
            primary_files={
                str(p.relative_to(ROOT)): dict(bytes=p.stat().st_size, sha256=file_hash(p))
                for p in primary_paths
            },
        ),
    )
    pilot = (
        json.loads((ROOT / 'PILOT.json').read_text()) if (ROOT / 'PILOT.json').exists() else None
    )
    estimate = None
    if pilot:
        n = len(pilot['seeds'])
        estimate = dict(
            pilot_wall_s=pilot['elapsed_s'],
            pilot_scenarios=n,
            full_projected_wall_s=pilot['elapsed_s'] * 64 / n,
            method='Scenario-scaled pilot including full H2 grids; H3 pilot used 4/13 delays. Projection is approximate and may underestimate extra H3 continuations.',
            workers=pilot['workers'],
        )
        atomic_json(ROOT / 'runtime_estimate.json', estimate)
    report = [
        '# E01 — StaleGuard robustness evaluation',
        '',
        '**Status: '
        + validation['status']
        + '**. '
        + str(len(outcomes))
        + '/24,960 policy outcomes; '
        + str(len(forecasts))
        + '/320 production full forecasts. No incomplete subset is labeled as the full evaluation.',
        '',
        'The immutable protocol is in specification.json and selection.json. All 64 scenarios remain selected; no recalibration, outcome filtering, retuning, wind or extra conditions are permitted.',
        '',
        'Plant: 50 Hz PID / 1000 Hz PyBullet. Predictor: unchanged nominal 100 Hz model and original 19-point 0–1 s grid. H3 retains all 13 dispatch delays, including 1.2, 1.6 and 2 s.',
        '',
        'A1/A2 change only actual base-link mass after the final reset; local inertia, PID mass compensation and predictor remain nominal. Actual mass/inertia are verified with getDynamicsInfo. True perturbed approval/PID snapshots are captured without resetting at approval.',
        '',
        'B1/B2 add the same scenario-fixed world-frame position/velocity Gaussian bias at approval forecasts and Use-Time Recheck snapshots. Plant, PID observations/memory, geometry, references and physical maneuver anchors remain exact. Nominal H2 truth is reused only after snapshot and independent command/recovery replay equivalence checks.',
        '',
        'Timing is measured offline. Approval forecasts and dispatch rechecks are timed serially; only physical continuations run in parallel, and all physical futures are joined before timed batches. Fresh wall-clock measurements still have ordinary hardware/OS variation. Resumption reuses recorded timings and decisions. JIT warmups are setup computations, reported separately from 320 production forecasts.',
        '',
        'Workers retire between scenarios to bound native SCG/CasADi memory growth. Resumption checks simulator assets, numerical dependencies and the frozen experiment configuration. Historical source revisions are not execution requirements.',
        '',
        'All StaleGuard ablations share one full forecast per scenario/condition. The archived nominal command-aware benchmark used command-only forecast timing; N0 discrepancies are reported in nominal_archive_comparison.csv rather than hidden or tuned away.',
        '',
        'Table E01-A: prediction accuracy, conditional false-feasible/reject rates, sampled boundary errors and readiness. Censored errors remain N/M. Table E01-B: absolute policy outcomes, signs separately for A1/A2, zero-delay failures and violation timing. Table E01-C: matched paired differences and shared-index, whole-scenario 2,000-resample bootstrap. Table E01-D: actual execution/caching statistics.',
        '',
        'Partial tables explicitly include scenario IDs and tested delays. Comparative rows use a common completed rectangular scenario-delay subset across the available conditions for that comparator. This is completion-based accounting, never exclusion based on safety or success.',
        '',
        'Only 64 scenarios are planned; statistical precision is limited. Report safety gains together with completion losses. Neither favorable point estimates nor a lower unsafe rate alone establish dominance. If an advantage reverses, retain and report it.',
        '',
        'Scope: these results concern only ±5%/±10% mass mismatch and the two specified supervisory biases. They establish neither arbitrary-error/wind robustness, hardware safety, onboard timing nor continuous-time guarantees.',
        '',
        'Nominal archive outcome differences observed so far: '
        + str(validation['nominal_outcome_differences'])
        + '; action differences: '
        + str(validation['nominal_action_differences'])
        + '.',
        '',
    ]
    if estimate:
        report += [
            'Pilot runtime and approximate full-workload projection: `'
            + json.dumps(estimate)
            + '`.',
            '',
        ]
    if policy_rows:
        report += [
            '## Recorded outcomes',
            '',
            'These are the actually completed records, not estimates for unexecuted trials. Partial pilot values verify the experimental pathway and cannot establish robustness of the full cohort.',
            '',
            '| Condition | Policy | Trials | Unsafe | Safe completion | Recovery attempts | Unsafe recoveries | Status |',
            '| --- | --- | --- | --- | --- | --- | --- | --- |',
        ]
        for row in policy_rows:
            if row['mass_sign'] == 'combined':
                report.append(
                    '| '
                    + ' | '.join(
                        str(row[key])
                        for key in (
                            'condition',
                            'policy',
                            'outcomes',
                            'unsafe',
                            'safe_completions',
                            'recovery_attempts',
                            'unsafe_recoveries',
                            'status',
                        )
                    )
                    + ' |'
                )
        report.append('')
    if (ROOT / 'verification/TEST_RESULTS.json').exists():
        checks = json.loads((ROOT / 'verification/TEST_RESULTS.json').read_text())
        report += [
            'Executed verification: '
            + json.dumps(checks)
            + '. See verification/tests.log for the individual checks.',
            '',
        ]
    if (ROOT / 'verification/RESUME_PILOT.json').exists():
        report += [
            'Completed-pilot resume verification: '
            + (ROOT / 'verification/RESUME_PILOT.json').read_text().strip(),
            '',
        ]
    report += [
        '## Commands',
        '',
        '```bash',
        'python -m staleguard.robustness run --workers 2',
        '# Native host unattended launcher (tmux preferred; POSIX session fallback):',
        'python -m staleguard.robustness launch --workers 2',
        '# Resume: the identical command reuses completed primary records.',
        'python -m staleguard.robustness summarize',
        'python -m staleguard.robustness status',
        '```',
        '',
    ]
    (ROOT / 'REPORT.md').write_text('\n'.join(report))
    print('E01 summary:', validation, flush=True)
