"""Prediction accuracy from saved comparisons only; never runs a simulator."""
import argparse
from enum import Enum
import json
import math
from pathlib import Path
import statistics


class Metric(str, Enum):
    TOTAL_TESTS = 'total_tests'  # Comparisons with a directly available prediction.
    CORRECT_SAFE = 'correct_safe'  # Predicted SAFE; actual flight SAFE.
    CORRECT_UNSAFE = 'correct_unsafe'  # Predicted UNSAFE; actual flight UNSAFE.
    FALSE_SAFE = 'false_safe'  # Predicted SAFE; actual flight UNSAFE.
    FALSE_REJECTION = 'false_rejection'  # Predicted UNSAFE; actual flight SAFE.
    ACCURACY = 'accuracy'  # Correct classifications / evaluated comparisons.
    FALSE_SAFE_RATE = 'false_safe_rate'  # False-safe / actual unsafe in these comparisons.
    FALSE_REJECTION_RATE = 'false_rejection_rate'  # False-rejection / actual safe.


def read_records(path, repair_tail=False):
    """Reject duplicates; resume may discard only an unfinished final write."""
    path = Path(path)
    data = path.read_bytes() if path.exists() else b''
    rows, seen, offset = [], set(), 0
    for line in data.splitlines(keepends=True):
        if not line.endswith(b'\n'):
            if not repair_tail:
                raise ValueError('Unfinished final journal line; use --resume')
            with path.open('r+b') as stream:
                stream.truncate(offset)
            break
        row = json.loads(line)
        key = row['seed'], row['delay_s']
        if key in seen:
            raise ValueError(f'Duplicate comparison: {key}')
        seen.add(key)
        rows.append(row)
        offset += len(line)
    return rows


def ratio(numerator, denominator):
    return numerator/denominator if denominator else None


def confusion(rows, field):
    counts = dict(correct_safe=0, correct_unsafe=0, false_safe=0, false_rejection=0)
    for row in rows:
        predicted = row[field]
        if predicted is None:
            continue
        name = ('correct_safe' if row['safe'] else 'false_safe') if predicted else (
                'false_rejection' if row['safe'] else 'correct_unsafe')
        counts[name] += 1
    total = sum(counts.values())
    return dict(total_tests=total, **counts,
                accuracy=ratio(counts['correct_safe']+counts['correct_unsafe'],total),
                false_safe_rate=ratio(counts['false_safe'],counts['false_safe']+counts['correct_unsafe']),
                false_rejection_rate=ratio(counts['false_rejection'],counts['false_rejection']+counts['correct_safe']))


def timing_summary(samples):
    ordered = sorted(value*1000 for value in samples)
    return dict(samples=len(ordered), mean_ms=statistics.mean(ordered) if ordered else None,
                median_ms=statistics.median(ordered) if ordered else None,
                p95_ms=ordered[math.ceil(.95*len(ordered))-1] if ordered else None,
                max_ms=max(ordered) if ordered else None)


def summarize(journal, protocol=None):
    journal = Path(journal)
    if protocol is None:
        protocol = json.loads(journal.with_name('run.json').read_text())['protocol']
    rows = read_records(journal)
    expected = {(seed, delay) for seed in protocol['seeds'] for delay in protocol['delays_s']}
    grid = protocol['prediction_grid_s']
    by_seed = {seed:[] for seed in protocol['seeds']}
    for row in rows:
        key = row['seed'], row['delay_s']
        if key not in expected:
            raise ValueError(f'Comparison outside fixed protocol: {key}')
        if any(type(row[name]) is not bool for name in ('safe','completed','safe_completion','prediction_grid_point','predicted_safe_ttl')):
            raise ValueError('Scientific classifications must be booleans')
        if row['safe_completion'] != (row['safe'] and row['completed']):
            raise ValueError('Inconsistent safe_completion')
        direct = row['delay_s'] in grid
        if row['prediction_grid_point'] != direct or (type(row['predicted_safe_raw']) is bool) != direct:
            raise ValueError('Raw prediction must exist exactly at directly evaluated grid points')
        if not direct and row['predicted_safe_raw'] is not None:
            raise ValueError('Off-grid raw predictions must be null')
        expiry = row['predicted_first_unsafe_delay_s']
        if expiry is not None and expiry not in grid:
            raise ValueError('Predicted first unsafe delay is outside the original model grid')
        permits = expiry is None or round(row['delay_s']*protocol['control_hz']) < round(expiry*protocol['control_hz'])
        if row['predicted_safe_ttl'] != permits:
            raise ValueError('TTL classification differs from existing expiry semantics')
        computation = row['prediction_computation_s']
        if computation is not None and (not math.isfinite(computation) or computation < 0):
            raise ValueError('Invalid prediction computation time')
        by_seed[row['seed']].append(row)
    scenarios, errors, samples = [], [], []
    for seed, group in by_seed.items():
        if not group:
            continue
        group.sort(key=lambda row:row['delay_s'])
        for field in ('predicted_first_unsafe_delay_s','prediction_computation_s'):
            if any(row[field] != group[0][field] for row in group):
                raise ValueError(f'One frozen forecast per seed required: {seed}, {field}')
        predicted = group[0]['predicted_first_unsafe_delay_s']
        raw = [row for row in group if row['prediction_grid_point']]
        if {row['delay_s'] for row in raw} == set(grid):
            first_raw_unsafe = next((row['delay_s'] for row in raw if not row['predicted_safe_raw']),None)
            if first_raw_unsafe != predicted:
                raise ValueError('TTL does not match the first directly predicted unsafe delay')
        actual = next((row['delay_s'] for row in group if not row['safe']),None)
        error = abs(predicted-actual) if predicted is not None and actual is not None else None
        if error is not None:
            errors.append(error)
        if group[0]['prediction_computation_s'] is not None:
            samples.append(group[0]['prediction_computation_s'])
        seen_unsafe, nonmonotonic = False, False
        for row in group:
            nonmonotonic |= seen_unsafe and row['safe']
            seen_unsafe |= not row['safe']
        scenarios.append(dict(seed=seed, tested_delays=len(group), complete=len(group)==len(protocol['delays_s']),
            predicted_first_unsafe_delay_s=predicted, actual_first_unsafe_delay_s=actual,
            absolute_ttl_error_s=error, non_monotonic_actual_safety=nonmonotonic))
    by_delay = []
    for delay in protocol['delays_s']:
        group = [row for row in rows if row['delay_s']==delay]
        by_delay.append(dict(delay_s=delay,total_comparisons=len(group),
            raw_prediction=confusion(group,'predicted_safe_raw'),
            ttl_dispatch_decision=confusion(group,'predicted_safe_ttl'),
            actual_safe_completions=sum(row['safe_completion'] for row in group),
            actual_unsafe=sum(not row['safe'] for row in group)))
    return dict(total_comparisons=len(rows), expected_comparisons=len(expected),
        missing_comparisons=len(expected)-len(rows), scenarios_with_records=len(scenarios),
        raw_prediction=confusion(rows,'predicted_safe_raw'),
        ttl_dispatch_decision=confusion(rows,'predicted_safe_ttl'), per_delay=by_delay,
        ttl_false_safe_cases=[dict(seed=row['seed'],delay_s=row['delay_s'],
            prediction_grid_point=row['prediction_grid_point'],
            predicted_first_unsafe_delay_s=row['predicted_first_unsafe_delay_s'],
            min_clearance_m=row['min_clearance_m'])
            for row in rows if row['predicted_safe_ttl'] and not row['safe']],
        ttl=dict(per_scenario=scenarios, absolute_error_samples=len(errors),
            mean_absolute_error_s=statistics.mean(errors) if errors else None,
            median_absolute_error_s=statistics.median(errors) if errors else None,
            no_actual_unsafe_observed_seeds=[row['seed'] for row in scenarios if row['complete'] and row['actual_first_unsafe_delay_s'] is None],
            no_predicted_unsafe_observed_seeds=[row['seed'] for row in scenarios if row['predicted_first_unsafe_delay_s'] is None],
            non_monotonic_actual_safety_seeds=[row['seed'] for row in scenarios if row['non_monotonic_actual_safety']],
            interpretation='First unsafe tested grid points, not continuous thresholds; null is unobserved, not extrapolated.'),
        warmed_prediction_computation=timing_summary(samples),
        sampling_note='Repeated dispatch delays within 20 development scenarios are not independent scenarios.')


def report_markdown(metrics):
    """Keep the requested readable report inside run.json, without a fourth file."""
    raw, ttl, timing = (metrics[name] for name in ('raw_prediction','ttl_dispatch_decision','warmed_prediction_computation'))
    def number(value, digits=3):
        return 'unobserved' if value is None else f'{value:.{digits}f}'
    lines = [
        f'The accuracy experiment compares approval-time forecasts with {metrics["total_comparisons"]} saved actual PyBullet flights; no new ground-truth flights were run. '
        'The UAV patrols east, then makes its frozen left turn and flies north. Waiting shifts the turn east and can make it unsafe. '
        f'Direct model predictions were correct in {raw["correct_safe"]+raw["correct_unsafe"]}/{raw["total_tests"]} comparisons; '
        f'the TTL dispatch rule was correct in {ttl["correct_safe"]+ttl["correct_unsafe"]}/{ttl["total_tests"]}, with {ttl["false_safe"]} unsafe turns permitted and {ttl["false_rejection"]} safe turns rejected. '
        'These development measurements distinguish model errors from using a coarse expiry grid. The implementation is ready to measure an independent frozen dataset, but these results do not establish generalization or justify changing the predictor. No parameters were tuned.',
        '', '## Overall accuracy', '',
        '| Measurement | Tests | Accuracy | False-safe | False-rejection |',
        '|---|---:|---:|---:|---:|']
    for label, result in [('Raw prediction',raw),('TTL dispatch decision',ttl)]:
        lines.append(f'| {label} | {result["total_tests"]} | {number(result["accuracy"])} | {result["false_safe"]} | {result["false_rejection"]} |')
    lines += ['', 'SAFE means at least 5 cm body clearance throughout the entire flight, including waiting and after checkpoint passage. Completion also requires reaching the fixed checkpoint at the required velocity for the required dwell before the deadline; it is context, not the prediction target. Readiness and recovery are outside this experiment.',
        '', '## Per-delay accuracy', '',
        'Raw columns are unobserved where the original predictor did not evaluate that delay. Each TTL row contains 20 comparisons.', '',
        '| D (s) | Raw correct / tests | Raw false-safe / rejection | TTL correct / tests | TTL false-safe / rejection | Actual safe completion | Actual unsafe |',
        '|---:|---:|---:|---:|---:|---:|---:|']
    for row in metrics['per_delay']:
        a,b = row['raw_prediction'],row['ttl_dispatch_decision']
        correct = f'{a["correct_safe"]+a["correct_unsafe"]}/{a["total_tests"]}' if a['total_tests'] else 'unobserved'
        mistakes = f'{a["false_safe"]} / {a["false_rejection"]}' if a['total_tests'] else 'unobserved'
        lines.append(f'| {row["delay_s"]:.2f} | {correct} | {mistakes} | {b["correct_safe"]+b["correct_unsafe"]}/{b["total_tests"]} | {b["false_safe"]} / {b["false_rejection"]} | {row["actual_safe_completions"]}/20 | {row["actual_unsafe"]}/20 |')
    lines += ['', '## TTL prediction errors', '',
        '| Seed | Predicted first unsafe D (s) | Actual first unsafe tested D (s) | Absolute difference (s) |',
        '|---:|---:|---:|---:|']
    for row in metrics['ttl']['per_scenario']:
        lines.append(f'| {row["seed"]} | {number(row["predicted_first_unsafe_delay_s"],2)} | {number(row["actual_first_unsafe_delay_s"],2)} | {number(row["absolute_ttl_error_s"],2)} |')
    t = metrics['ttl']
    lines += ['', f'First-unsafe grid-point MAE: {number(t["mean_absolute_error_s"])} s; median: {number(t["median_absolute_error_s"])} s, over {t["absolute_error_samples"]} observable pairs. '
        f'No actual unsafe delay observed through 1 s: {t["no_actual_unsafe_observed_seeds"]}; no predicted unsafe delay: {t["no_predicted_unsafe_observed_seeds"]}. '
        f'Non-monotonic actual safety: {t["non_monotonic_actual_safety_seeds"]}. These are grid-resolved observations, not exact threshold errors; unobserved thresholds are excluded rather than extrapolated.',
        '', '## Computation performance', '',
        f'{timing["samples"]} approval forecasts, one per scenario: mean {number(timing["mean_ms"],2)} ms, median {number(timing["median_ms"],2)} ms, p95 {number(timing["p95_ms"],2)} ms, maximum {number(timing["max_ms"],2)} ms. '
        'The existing predictor measures elapsed call-to-result time, including model construction and all eleven delay rollouts. Its numerical kernels were warmed with one model control step before the measured forecasts; no extra approval forecast was run. Cold-start time and dispatch readiness are not measured here.',
        '', '## Interpretation and next step', '',
        'Raw accuracy applies only to directly evaluated delays. TTL accuracy also includes the gaps between them: an unsafe turn inside a gap can still be permitted until the next predicted unsafe grid point. '
        'Inspect the separate confusion matrices before attributing any error to the model. This experiment changes neither the 0.10 s prediction grid nor the expiry rule, and performs no recovery-policy comparison. '
        'Use the same frozen measurement procedure on independently calibrated evaluation scenarios; do not treat these 380 delay comparisons as 380 independent scenarios.',
        '', 'Technical note: the source flight journal stores violation time from flight start. This journal also retains that original value as first_violation_flight_s, and records first_violation_s relative to approval as required. Source and snapshot checks, cached forecasts, resumption state and this report are retained in run.json.']
    cases = metrics['ttl_false_safe_cases']
    if cases:
        row = cases[0]
        off_grid = sum(not case['prediction_grid_point'] for case in cases)
        explanation = (f'Observed: {off_grid}/{len(cases)} TTL false-safe decisions occurred at delays the model did not evaluate. '
            f'For example, in recorded seed {row["seed"]}, a {row["delay_s"]:.2f} s wait produced '
            f'{row["min_clearance_m"]*100:.2f} cm minimum body clearance, below the required 5 cm. '
            f'The TTL rule still permitted that turn because its first predicted unsafe delay was '
            f'{row["predicted_first_unsafe_delay_s"]:.2f} s. '
            'This establishes a gap in the existing TTL decisions; it does not establish what a direct model rollout at that untested delay would predict.')
        lines.insert(lines.index('## Interpretation and next step')+2,explanation+'\n')
    return '\n'.join(lines)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('journal',type=Path)
    args = parser.parse_args()
    result = summarize(args.journal)
    output = args.journal.with_name('metrics.json')
    temporary = output.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    temporary.replace(output)
    print(f'{result["total_comparisons"]} comparisons; metrics written to {output}')


if __name__ == '__main__':
    main()
