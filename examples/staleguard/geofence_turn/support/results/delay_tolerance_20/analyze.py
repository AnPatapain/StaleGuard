"""Recompute Task 1 tables from the saved actual-flight journal; no simulations."""
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path
import statistics

ROOT = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')


def write_csv(path, rows):
    with path.open('w') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def regained(rows, field):
    lost = False
    result = []
    for row in rows:
        if not row[field]:
            lost = True
        elif lost:
            result.append(row['delay_s'])
    return result


def analyze():
    protocol = read(ROOT/'protocol.json')
    data = read(ROOT/'calibrated_scenarios.json')
    validation = read(ROOT/'validation.json') if (ROOT/'validation.json').exists() else {}
    rows = [json.loads(line) for line in (ROOT/'flights.jsonl').read_text().splitlines()]
    grid = protocol['delay_steps']
    assert len(grid) == 19 and len(rows) == 380
    keys = {(row['seed'],round(row['delay_s']*50)) for row in rows}
    assert len(keys) == 380
    scenarios = []
    for entry in data['scenarios']:
        scenario = entry['scenario']
        seed = scenario['seed']
        flights = sorted((r for r in rows if r['seed']==seed),key=lambda r:r['delay_s'])
        assert [round(r['delay_s']*50) for r in flights] == grid
        assert flights[0]['safe_completion']
        assert all(r['event']['policy']=='no_recovery' for r in flights)
        assert all(r['physics_samples']==r['expected_physics_samples'] for r in flights)
        def first(predicate):
            return next((r for r in flights if predicate(r)),None)
        unsafe = first(lambda r:not r['safe'])
        missed = first(lambda r:not r['completed'])
        failed = first(lambda r:not r['safe_completion'])
        predispatch = first(lambda r:r['pre_dispatch_violation_s'] is not None)
        cause = None if failed is None else 'both' if not failed['safe'] and not failed['completed'] else 'checkpoint_only' if failed['safe'] else 'safety_only'
        observation = {} if missed is None else missed['checkpoint_observations']
        scenarios.append(dict(
            seed=seed,reused=seed in protocol['existing_seeds'],approval_s=scenario['approval_s'],
            speed_mps=scenario['speed'],radius_m=scenario['command']['radius'],deadline_s=scenario['deadline_s'],
            d0_body_clearance_m=flights[0]['min_clearance_m'],
            d0_clearance_above_requirement_m=flights[0]['min_clearance_m']-.05,
            last_safe_completable_delay_s=max(r['delay_s'] for r in flights if r['safe_completion']),
            first_unsafe_delay_s=None if unsafe is None else unsafe['delay_s'],
            first_completion_loss_s=None if missed is None else missed['delay_s'],
            first_safe_completion_loss_s=None if failed is None else failed['delay_s'],
            first_loss_kind=cause,
            first_predispatch_violation_delay_s=None if predispatch is None else predispatch['delay_s'],
            any_predispatch_violation=predispatch is not None,
            nonmonotonic_safety=bool(regained(flights,'safe')),
            nonmonotonic_completion=bool(regained(flights,'completed')),
            nonmonotonic_safe_completion=bool(regained(flights,'safe_completion')),
            positive_safe_completion_delays=sum(r['safe_completion'] for r in flights if round(r['delay_s']*50)>0),
            completion_at_or_above_0_1=any(r['safe_completion'] for r in flights if round(r['delay_s']*50)>=5),
            first_completion_loss_position_seen=observation.get('position_seen'),
            first_completion_loss_entire_pass_east=observation.get('entire_north_altitude_pass_east_of_box'),
            first_completion_loss_minimum_east_miss_m=observation.get('minimum_east_miss_m'),
            first_completion_loss_qualified_dwell_s=observation.get('longest_qualified_dwell_s')))
    scenarios.sort(key=lambda s:s['seed'])
    per_delay = []
    for step in grid:
        flights = [row for row in rows if round(row['delay_s']*50)==step]
        per_delay.append(dict(delay_s=step/50,scenarios=len(flights),
            safe_completion=sum(r['safe_completion'] for r in flights),
            safe_noncompletion=sum(r['safe'] and not r['completed'] for r in flights),
            unsafe=sum(not r['safe'] for r in flights),
            checkpoint_completed=sum(r['completed'] for r in flights),
            predispatch_violation=sum(r['pre_dispatch_violation_s'] is not None for r in flights)))
    calibration_rows = []
    calibration_flights = 0
    for directory in sorted(ROOT.glob('calibration_*')):
        calibration_rows.extend(json.loads(line) for line in (directory/'attempts.jsonl').read_text().splitlines())
        calibration_flights += read(directory/'finished.json')['trials']
    assert len(calibration_rows)+5 == data['generated_and_attempted']
    assert sum(r['accepted'] for r in calibration_rows)==15
    kind_counts = Counter(s['first_loss_kind'] for s in scenarios)
    maxima = [s['last_safe_completable_delay_s'] for s in scenarios]
    summary = dict(
        evidence='Actual SCG/PyBullet No-Recovery flights, monitored at 1000 Hz.',
        accepted_scenarios=20,reused=5,newly_calibrated=15,
        delays_s=[step/50 for step in grid],actual_sweep_flights=len(rows),
        new_calibration_flights=calibration_flights,
        preflight_validation_flights=validation.get('preflight_validation_flights',0),
        confirmation_flights=validation.get('confirmation_flights',0),
        total_new_mission_flights=len(rows)+calibration_flights+validation.get('preflight_validation_flights',0)+validation.get('confirmation_flights',0),
        new_seeds=[r['seed'] for r in calibration_rows],
        rejected_seeds=[r['seed'] for r in calibration_rows if not r['accepted']],
        generated_and_attempted=data['generated_and_attempted'],
        calibration_acceptance_rate=20/data['generated_and_attempted'],
        new_calibration_acceptance_rate=15/len(calibration_rows),
        positive_delay_completion_scenarios=sum(s['positive_safe_completion_delays']>0 for s in scenarios),
        positive_delay_completion_rate=sum(s['positive_safe_completion_delays']>0 for s in scenarios)/20,
        completion_at_or_above_0_1_scenarios=sum(s['completion_at_or_above_0_1'] for s in scenarios),
        completion_at_or_above_0_1_rate=sum(s['completion_at_or_above_0_1'] for s in scenarios)/20,
        no_positive_delay_completion_seeds=[s['seed'] for s in scenarios if s['positive_safe_completion_delays']==0],
        last_safe_delay=dict(min=min(maxima),median=statistics.median(maxima),max=max(maxima)),
        first_loss_kind_counts=dict(kind_counts),
        nonmonotonic_safety_seeds=[s['seed'] for s in scenarios if s['nonmonotonic_safety']],
        nonmonotonic_completion_seeds=[s['seed'] for s in scenarios if s['nonmonotonic_completion']],
        nonmonotonic_safe_completion_seeds=[s['seed'] for s in scenarios if s['nonmonotonic_safe_completion']],
        predispatch_violation_scenarios=sum(s['any_predispatch_violation'] for s in scenarios),
        predispatch_violation_flights=sum(r['pre_dispatch_violation_s'] is not None for r in rows),
        first_unsafe_already_predispatch=sum(next(r for r in rows if r['seed']==s['seed'] and r['delay_s']==s['first_unsafe_delay_s'])['pre_dispatch_violation_s'] is not None for s in scenarios if s['first_unsafe_delay_s'] is not None),
        first_completion_loss_east_miss_scenarios=sum(s['first_completion_loss_entire_pass_east'] is True for s in scenarios),
        first_completion_loss_position_seen_scenarios=sum(s['first_completion_loss_position_seen'] is True for s in scenarios),
        last_safe_delay_distribution=dict(Counter(str(s['last_safe_completable_delay_s']) for s in scenarios)),
        right_censored_safety_seeds=[s['seed'] for s in scenarios if s['first_unsafe_delay_s'] is None],
        analyzed_journal_sha256=hashlib.sha256((ROOT/'flights.jsonl').read_bytes()).hexdigest(),
        analysis_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    write_csv(ROOT/'scenario_metrics.csv',scenarios)
    write_csv(ROOT/'per_delay.csv',per_delay)
    write_json(ROOT/'summary.json',summary)
    print(json.dumps(summary,indent=2))
    print('\nSCENARIOS')
    for s in scenarios:
        print(s['seed'],s['last_safe_completable_delay_s'],s['first_unsafe_delay_s'],s['first_completion_loss_s'],s['first_loss_kind'],s['first_completion_loss_entire_pass_east'])


if __name__ == '__main__':
    analyze()
