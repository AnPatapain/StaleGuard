"""Original safety classifications and TTL errors; saved records only."""
from pathlib import Path
import sys
from staleguard import inspection_descent
import argparse
import json
import numpy as np
from staleguard.geofence_turn.ttl_pred_accuracy.metrics import read_records,confusion
from staleguard.inspection_descent.baseline_benchmark.metrics import timing_summary


def summarize(journal,protocol=None):
    journal=Path(journal)
    if protocol is None:
        protocol=json.loads(journal.with_name('run.json').read_text())['protocol']
    rows=read_records(journal);expected={(s,d) for s in protocol['seeds'] for d in protocol['delays']}
    if any((r['seed'],r['delay_s']) not in expected for r in rows):
        raise ValueError('Accuracy record outside frozen protocol')
    observable=[r for r in rows if type(r['safe']) is bool]
    groups={seed:[r for r in rows if r['seed']==seed] for seed in protocol['seeds']}
    scenarios=[]; errors=[];times=[]
    invalid={'INVALID_AT_APPROVAL','INVALID_NOMINAL'}
    for seed,group in groups.items():
        if not group:
            continue
        first=group[0]
        for name in ('prediction','actual','computation_s'):
            if any(r[name]!=first[name] for r in group):
                raise ValueError('One frozen approval prediction/actual classification per seed required')
        pred,actual=first['prediction'],first['actual']
        error=None
        if (pred['first_bad_delay'] is not None and actual['first_bad_delay'] is not None
                and pred['classification'] not in invalid and actual['classification'] not in invalid):
            error=pred['first_bad_delay']-actual['first_bad_delay'];errors.append(error)
        times.append(first['computation_s'])
        scenarios.append(dict(seed=seed,prediction=pred,actual=actual,ttl_signed_error_s=error))
    timing=timing_summary(times)
    return dict(recorded_comparisons=len(rows),expected_comparisons=len(expected),
        selected_scenarios=len(groups),calibration_statistics=protocol['calibration_statistics'],
        raw_prediction=confusion(observable,'predicted_safe_raw'),
        ttl_dispatch_decision=confusion(observable,'predicted_safe_ttl'),
        classification_correct=sum(s['prediction']['classification']==s['actual']['classification'] for s in scenarios),
        classification_total=len(scenarios),
        ttl=dict(comparable_cases=len(errors),mean_absolute_error_s=float(np.mean(np.abs(errors))) if errors else None,
            mean_signed_error_s=float(np.mean(errors)) if errors else None,
            max_absolute_error_s=float(np.max(np.abs(errors))) if errors else None,per_scenario=scenarios),
        prediction_computation_s=timing,
        per_delay={str(d):dict(raw_prediction=confusion([r for r in observable if r['delay_s']==d],'predicted_safe_raw'),
            ttl_dispatch_decision=confusion([r for r in observable if r['delay_s']==d],'predicted_safe_ttl')) for d in protocol['delays']},
        note='Original 8.5 s flight-start safety horizon, not the 4.2 s operational deadline. Completion is unmeasured/null. Readiness is outside accuracy.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('executions',type=Path)
    parser.add_argument('--out',type=Path)
    args=parser.parse_args()
    from staleguard.inspection_descent.storage import write_json
    write_json(args.out or args.executions.with_name('metrics.json'),summarize(args.executions))


if __name__=='__main__':
    main()
