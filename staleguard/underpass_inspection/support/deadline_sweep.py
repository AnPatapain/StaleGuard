"""Independent actual-flight deadline sweeps; resumable journal and metrics only."""
from pathlib import Path
import sys
from staleguard import underpass_inspection
import argparse
from collections import Counter
from dataclasses import asdict
import fcntl
import hashlib
import json
import csv
import time
import numpy as np

from staleguard.underpass_inspection.calibration import (ROOT, load_calibrated, manifest_path,
    experiment_provenance, check_provenance, release_memory, write_json, append_json, file_hash)
from staleguard.underpass_inspection.simulation import execute
from staleguard.underpass_inspection.pilot import intervals

GRID = tuple(i/50 for i in range(51))


def flight_key(row):
    return row['seed'], row['mode'], row['start_s']


def read_journal(path, repair=False):
    """Only an uncommitted unfinished final line may be discarded."""
    rows, seen = [], set()
    with Path(path).open('rb+' if repair else 'rb') as stream:
        while True:
            offset = stream.tell(); line = stream.readline()
            if not line:
                break
            try:
                row = json.loads(line)
                if not line.endswith(b'\n'):
                    raise ValueError('Unfinished last line')
            except (json.JSONDecodeError, ValueError):
                if not repair or stream.read():
                    raise ValueError('Invalid execution journal')
                stream.seek(offset); stream.truncate(); break
            key = flight_key(row)
            if key in seen:
                raise ValueError(f'Duplicate flight: {key}')
            if row['safe_completion'] != (row['safe'] and row['inspection_completed']):
                raise ValueError('Inconsistent safety/completion flags')
            seen.add(key); rows.append(row)
    return rows


def committed_journal(folder, run, repair=False):
    path = Path(folder)/'executions.jsonl'
    raw = path.read_bytes(); size = run['journal_bytes']
    if len(raw) < size or hashlib.sha256(raw[:size]).hexdigest() != run['journal_sha256']:
        raise ValueError('Previously committed journal changed')
    return read_journal(path,repair)


def profile(rows):
    rows = sorted(rows,key=lambda r:r['start_s'])
    safe = [r for r in rows if r['safe']]; bad = [r for r in rows if not r['safe']]
    nonmonotonic = any(not a['safe'] and b['safe'] for a,b in zip(rows,rows[1:]))
    first = bad[0] if bad else None
    completed = [r['start_s'] for r in rows if r['safe_completion']]
    raw_completed = [r['start_s'] for r in rows if r['inspection_completed']]
    return dict(evaluations=len(rows),safe_count=len(safe),unsafe_count=len(bad),
        first_unsafe_s=first['start_s'] if first else None,
        last_safe_s=safe[-1]['start_s'] if safe else None,
        finite_boundary_observed=bool(safe and bad),non_monotonic=nonmonotonic,
        safety_intervals=intervals(rows),inspection_intervals=intervals(rows,'inspection_completed'),
        last_safe_completion_s=max(completed) if completed else None,
        last_raw_inspection_completion_s=max(raw_completed) if raw_completed else None,
        first_raw_inspection_failure_s=next((r['start_s'] for r in rows if not r['inspection_completed']),None),
        safe_but_inspection_incomplete=sum(r['safe'] and not r['inspection_completed'] for r in rows),
        first_unsafe_obstacle=first['first_violation']['obstacle'] if first else None,
        first_unsafe_part=first['first_violation']['part'] if first else None,
        first_unsafe_violation_before_start=bool(first and first['first_violation']['time_flight_s'] <= first['initiation_step']/50))


def distribution(values):
    values = np.array(values,dtype=float)
    return dict(n=len(values),minimum=float(values.min()) if len(values) else None,
        median=float(np.median(values)) if len(values) else None,
        mean=float(values.mean()) if len(values) else None,
        q25=float(np.percentile(values,25)) if len(values) else None,
        q75=float(np.percentile(values,75)) if len(values) else None,
        maximum=float(values.max()) if len(values) else None)


def summarize(rows, protocol):
    scenarios = []
    for scenario in protocol['scenarios']:
        seed = scenario['seed']
        selected = [r for r in rows if r['seed']==seed]
        profiles = {mode:profile([r for r in selected if r['mode']==mode])
                    for mode in ('dive','sideways','climb')}
        cmd, side, climb = (profiles[m] for m in ('dive','sideways','climb'))
        t_cmd = cmd['first_unsafe_s']; last_side = side['last_safe_s']
        gap = round(last_side-t_cmd,10) if t_cmd is not None and last_side is not None else None
        climb_deadline = climb['last_safe_s'] if climb['finite_boundary_observed'] and not climb['non_monotonic'] else None
        climb_gap = round(climb_deadline-t_cmd,10) if t_cmd is not None and climb_deadline is not None else None
        first_side_bad = side['first_unsafe_s']
        comparison = next((r for r in selected if r['mode']=='climb' and r['start_s']==first_side_bad),None)
        command_at_side_failure = next((r for r in selected if r['mode']=='dive' and r['start_s']==first_side_bad),None)
        scenarios.append(dict(seed=seed,profiles=profiles,t_cmd_s=t_cmd,
            completed_evaluations=len(selected),
            t_rec_side_last_tested_safe_s=last_side,side_boundary_observed=side['finite_boundary_observed'],
            delta_side_grid_s=gap,
            contiguous_side_window_interpretation=not (cmd['non_monotonic'] or side['non_monotonic']),
            side_gap_right_censored=not side['finite_boundary_observed'] and last_side is not None,
            t_rec_climb_s=climb_deadline,delta_climb_s=climb_gap,
            climb_safe_at_first_unsafe_side_start=comparison['safe'] if comparison else None,
            command_safe_at_first_unsafe_side_start=command_at_side_failure['safe'] if command_at_side_failure else None,
            climb_safe_at_some_later_side_start=any(r['mode']=='climb' and r['safe'] and
                first_side_bad is not None and r['start_s']>=first_side_bad for r in selected)))
    n = len(scenarios)
    gaps = [x['delta_side_grid_s'] for x in scenarios if x['delta_side_grid_s'] is not None]
    positive = [g for g in gaps if g>0]
    finite_gaps = [x['delta_side_grid_s'] for x in scenarios
                   if x['delta_side_grid_s'] is not None and x['side_boundary_observed']]
    counts = dict(accepted_scenarios=n,
        finite_command_expiration=sum(x['profiles']['dive']['finite_boundary_observed'] for x in scenarios),
        no_command_safety_loss_observed=sum(x['t_cmd_s'] is None for x in scenarios),
        finite_sideways_recovery_expiration=sum(x['side_boundary_observed'] for x in scenarios),
        sideways_boundary_unobserved=sum(not x['side_boundary_observed'] for x in scenarios),
        positive_side_grid_gaps=sum(g>0 for g in gaps),zero_side_grid_gaps=sum(g==0 for g in gaps),
        negative_side_grid_gaps=sum(g<0 for g in gaps),undefined_side_grid_gaps=n-len(gaps),
        finite_climb_expiration=sum(x['t_rec_climb_s'] is not None for x in scenarios),
        non_monotonic_scenarios=sum(any(p['non_monotonic'] for p in x['profiles'].values()) for x in scenarios),
        command_safe_when_sideways_first_fails=sum(x['command_safe_at_first_unsafe_side_start'] is True for x in scenarios),
        positive_side_gaps_with_finite_recovery_boundary=sum(x['delta_side_grid_s'] is not None and x['delta_side_grid_s']>0 and x['side_boundary_observed'] for x in scenarios),
        climb_safe_at_first_sideways_failure=sum(x['climb_safe_at_first_unsafe_side_start'] is True for x in scenarios),
        climb_after_side_not_assessable=sum(x['climb_safe_at_first_unsafe_side_start'] is None for x in scenarios))
    obstacle_counts={mode:dict(Counter(x['profiles'][mode]['first_unsafe_part'] for x in scenarios
                             if x['profiles'][mode]['first_unsafe_part'] is not None))
                     for mode in ('dive','sideways','climb')}
    complete=len(rows)==protocol['expected_flights']
    return dict(status='complete' if complete else 'partial',flights=len(rows),expected_flights=protocol['expected_flights'],
        interpretation='Descriptive deterministic development data; denominator is all accepted scenarios. Repeated delays are not independent scenarios.',
        gap_definition='Last TESTED safe sideways start minus first TESTED unsafe dive delay. If sideways never fails, this is a demonstrated-start difference, not an observed expiry gap. Non-monotonic profiles require full intervals.',
        calibration_statistics=protocol['calibration_statistics'],counts=counts,
        all_side_grid_gaps_s=distribution(gaps),positive_side_grid_gaps_s=distribution(positive),
        side_gaps_with_observed_recovery_boundary_s=distribution(finite_gaps),
        climb_gaps_s=distribution([x['delta_climb_s'] for x in scenarios if x['delta_climb_s'] is not None]),
        expiration_parts=obstacle_counts,
        safe_but_inspection_incomplete_flights=sum(r['mode']=='dive' and r['safe'] and not r['inspection_completed'] for r in rows),
        safe_but_inspection_incomplete_scenarios=sum(x['profiles']['dive']['safe_but_inspection_incomplete']>0 for x in scenarios),
        by_scenario=scenarios)


def write_metrics(folder, run, rows):
    folder=Path(folder); metrics=summarize(rows,run['protocol']);write_json(folder/'metrics.json',metrics)
    fields=('seed','t_cmd_s','t_rec_side_last_tested_safe_s','side_boundary_observed',
            'delta_side_grid_s','side_gap_right_censored','t_rec_climb_s','delta_climb_s',
            'command_last_safe_s','command_last_safe_completion_s','command_expiration_part',
            'side_expiration_part','non_monotonic','climb_safe_at_first_unsafe_side_start')
    with (folder/'per_scenario.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in metrics['by_scenario']:
            entry={k:row[k] for k in fields if k in row}
            entry.update(command_last_safe_s=row['profiles']['dive']['last_safe_s'],
                command_last_safe_completion_s=row['profiles']['dive']['last_safe_completion_s'],
                command_expiration_part=row['profiles']['dive']['first_unsafe_part'],
                side_expiration_part=row['profiles']['sideways']['first_unsafe_part'],
                non_monotonic=any(p['non_monotonic'] for p in row['profiles'].values()))
            writer.writerow(entry)
    fields=('seed','mode','start_s','approval_step','initiation_step','horizon_step','safe',
            'inspection_completed','safe_completion','min_clearance_m','violation_flight_s',
            'violation_approval_s','obstacle','part')
    with (folder/'per_delay.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in rows:
            entry={k:row[k] for k in fields if k in row};hit=row['first_violation']
            entry.update(violation_flight_s=hit['time_flight_s'] if hit else None,
                violation_approval_s=hit['time_approval_s'] if hit else None,
                obstacle=hit['obstacle'] if hit else None,part=hit['part'] if hit else None)
            writer.writerow(entry)
    n=metrics['counts']['accepted_scenarios'];c=metrics['counts'];cal=metrics['calibration_statistics']
    lines=['# Underpass development deadline analysis','',
        f'Implemented and ran a bounded actual-PyBullet study: {cal["attempted"]} generated scenarios, '
        f'{n} accepted and {cal["rejected"]} rejected. The UAV flies North toward a bridge, dives to inspect underneath, '
        'then swerves East around a pier. Independent sideways recovery abandons inspection and goes beside the deck. '
        f'{c["positive_side_grid_gaps"]}/{n} scenarios have positive tested sideways gaps, '
        f'{c["negative_side_grid_gaps"]}/{n} negative, {c["zero_side_grid_gaps"]}/{n} zero and '
        f'{c["undefined_side_grid_gaps"]}/{n} undefined. '
        'Grid endpoints can leave expiration unobserved. This small simulated development cohort does not establish '
        'real-world prevalence or predictive-policy performance. Next, evaluate approval-time command predictions '
        'against the frozen physical outcomes, retaining unfavorable and unobserved cases.','',
        '## What was tested','',
        'Calibration uses only nominal flight and immediate maneuvers: nominal must become unsafe, dive must complete '
        'inspection safely, and initial sideways recovery must be safe. Initial climb is a separate diagnostic. '
        'Each seed gets one draw with no replacement or parameter search. Speed, approval time and deck front vary '
        'around seed 0; absolute pier North position is drawn independently. These are provisional design assumptions '
        '(ranges in [README](../../../README.md)), not sampled real-world frequencies.','',
        'After calibration, every accepted scenario is replayed independently for dive and sideways starts from '
        '0–1.00 s at 0.02 s spacing. The climb diagnostic extends to '
        f'{max(run["protocol"]["grids"]["climb"]):.2f} s on that grid. All times are after approval. '
        'Safety is measured from reset through approval plus the unchanged eight-second observation period at '
        '1,000 Hz, including before intervention and after violations. Obstacles are visual-only in PyBullet; '
        'the unchanged 3D body-clearance predicate detects violations. Safe completion requires inspection dwell '
        'and safety throughout the flight. No model prediction or recovery selector was used.','',
        '## Observed results','',
        f'Completed physical sweep flights: **{len(rows)}/{metrics["expected_flights"]}**. '
        'Counts below use all accepted scenarios; repeated delay trials are not additional independent scenarios.','',
        '| Measurement | Observed count / accepted scenarios |','|---|---:|']
    if metrics['status']!='complete':
        lines.insert(2,'**PARTIAL EXECUTION: pending scenario/delay cells are not scientific outcomes. Counts below are interim only.**\n')
    for title,key in [('Finite command expiration','finite_command_expiration'),('Finite sideways expiration','finite_sideways_recovery_expiration'),
                      ('Positive tested sideways gap','positive_side_grid_gaps'),('Zero tested sideways gap','zero_side_grid_gaps'),
                      ('Negative tested sideways gap','negative_side_grid_gaps'),('Undefined sideways gap','undefined_side_grid_gaps'),
                      ('Finite climb expiration','finite_climb_expiration'),('Non-monotonic safety scenarios','non_monotonic_scenarios'),
                      ('Climb safe at first sideways failure','climb_safe_at_first_sideways_failure')]:
        lines.append(f'| {title} | {c[key]} / {n} |')
    def number(v):
        return 'unobserved/undefined' if v is None else f'{v:.2f}'
    lines+=['','The grid gap is the last tested safe sideways start minus the first tested unsafe dive delay. '
        f'Only {c["positive_side_gaps_with_finite_recovery_boundary"]}/{n} positive gaps also have an observed sideways '
        f'expiration; {c["sideways_boundary_unobserved"]}/{n} sideways profiles never fail on the tested grid. '
        'For those, the grid difference records a demonstrated safe-start separation, not an observed expiry gap. '
        f'{c["no_command_safety_loss_observed"]}/{n} command profiles never fail through 1.00 s and have no finite '
        'measured command lifetime or scalar gap. No boundary is interpolated between samples.','',
        '| Gap distribution (s) | Cases | Minimum | Median | Mean | Maximum |',
        '|---|---:|---:|---:|---:|---:|']
    for label,key in [('All defined sideways grid gaps','all_side_grid_gaps_s'),
                      ('Positive sideways grid gaps','positive_side_grid_gaps_s'),
                      ('Sideways gaps with observed recovery expiration','side_gaps_with_observed_recovery_boundary_s'),
                      ('Climb gaps with observed recovery expiration','climb_gaps_s')]:
        d=metrics[key]
        lines.append(f'| {label} | {d["n"]} | {number(d["minimum"])} | {number(d["median"])} | '
                     f'{number(d["mean"])} | {number(d["maximum"])} |')
    lines+=['','| First unsafe sample location | Dive scenarios | Sideways scenarios | Climb scenarios |',
            '|---|---:|---:|---:|']
    parts=sorted({p for counts in metrics['expiration_parts'].values() for p in counts})
    for part in parts:
        values=[metrics['expiration_parts'][m].get(part,0) for m in ('dive','sideways','climb')]
        lines.append(f'| {part.replace("_"," ")} | {values[0]} | {values[1]} | {values[2]} |')
    lines+=['','Locations describe geometry at the first violating sample; they are not a controlled causal isolation. '
        f'Climb remains safe at the first unsafe sideways start in {c["climb_safe_at_first_sideways_failure"]}/{n} '
        f'scenarios; {c["climb_after_side_not_assessable"]}/{n} have no observed sideways failure to compare. '
        f'The dive is still safe when sideways first fails in {c["command_safe_when_sideways_first_fails"]}/{n} cases. '
        'Climb is always reported separately and never selected automatically.','',
        f'Safe commands miss inspection in **{metrics["safe_but_inspection_incomplete_flights"]} flights across '
        f'{metrics["safe_but_inspection_incomplete_scenarios"]}/{n} scenarios**. This is mission failure without '
        'physical safety loss. Per-scenario last safe and last safe-completable delays are in '
        '[per_scenario.csv](per_scenario.csv); every trial is in [per_delay.csv](per_delay.csv).','',
        '## Interpretation and next decision','',
        'Positive gaps demonstrate that a command can become unsafe while a specified recovery remains physically '
        'available. Negative gaps or a safe command at sideways failure show the opposite ordering; neither is removed. '
        'These separately replayed deadlines can arise from different structure encounters, but this study does not '
        'establish statistical independence. A positive gap alone does not guarantee successful early intervention: '
        'prediction error, computation time and readiness have not been evaluated.','',
        'The frozen cohort and approval snapshots are suitable for a bounded approval-time prediction-accuracy study, '
        'provided it treats grid resolution, unobserved boundaries, whole-flight safety and inspection failure explicitly. '
        'Inspect these findings before implementing any policy benchmark. Geometry remains idealized, obstacles exert '
        'no contact forces, and only one simulator/controller family is tested. No StaleGuard or other predictive policy '
        'was implemented. Validation, rejected draws, source provenance and reproduction are documented in the '
        '[technical appendix](TECHNICAL_APPENDIX.md).','',
        'Non-monotonic tested safety intervals (inclusive tested endpoints; scalar gaps do not imply continuous windows):']
    for row in metrics['by_scenario']:
        for mode,p in row['profiles'].items():
            if p['non_monotonic']:
                lines.append(f'- Seed {row["seed"]}, {mode}: {p["safety_intervals"]}')
    if not c['non_monotonic_scenarios']:
        lines.append('None observed on the tested grids.')
    (folder/'REPORT.md').write_text('\n'.join(lines)+'\n')
    return metrics


def run_sweep(calibration, out, resume=False, max_flights=0, climb_max=1.5):
    cases, calibration_data=load_calibrated(calibration)
    if calibration_data['status']!='complete' or not cases:
        raise ValueError('Use a completed calibration with at least one accepted scenario')
    grids=dict(dive=list(GRID),sideways=list(GRID),climb=[i/50 for i in range(round(climb_max*50)+1)])
    protocol=dict(scenarios=[asdict(s) for s in cases],grids=grids,
        calibration_sha256=file_hash(manifest_path(calibration)),calibration_statistics=calibration_data['statistics'],
        expected_flights=len(cases)*sum(len(g) for g in grids.values()),
        recovery='Primary independent Eastward sideways; climb separate diagnostic',
        horizon='Unchanged scenario observation_s after approval; monitor from reset',
        representative_seed=cases[0].seed, representative_starts=[0.,.5,1.,climb_max],
        control_hz=50,physics_hz=1000)
    protocol=json.loads(json.dumps(protocol))
    out=Path(out);run_path=out/'run.json';journal=out/'executions.jsonl'
    if run_path.exists():
        if not resume:
            raise FileExistsError('Sweep exists; use --resume or a fresh --out')
        saved=json.loads(run_path.read_text())
        if saved['protocol']!=protocol:
            raise ValueError('Sweep resume protocol differs')
        check_provenance(saved['provenance'])
        rows=committed_journal(out,saved,repair=True)
    else:
        if resume:
            raise FileNotFoundError(run_path)
        if out.exists() and any(out.iterdir()):
            raise FileExistsError(out)
        out.mkdir(parents=True,exist_ok=True);journal.touch();rows=[]
        saved=dict(format='underpass_deadline_sweep_v1',status='partial',protocol=protocol,
                   created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
                   provenance=experiment_provenance(),journal_bytes=0,
                   journal_sha256=hashlib.sha256(b'').hexdigest(),resumptions=0)
        write_json(run_path,saved)
    done={flight_key(r) for r in rows}
    expected={(s.seed,m,d) for s in cases for m,g in grids.items() for d in g}
    if not done<=expected:
        raise ValueError('Unexpected journal flight')
    saved['resumptions']+=int(resume)
    representatives={}
    rep_path=out/'representative_trajectories.npz'
    if rep_path.exists():
        with np.load(rep_path) as archive:
            representatives={k:archive[k].copy() for k in archive.files}
    added=0
    try:
        for s in cases:
            for mode,grid in grids.items():
                for delay in grid:
                    key=s.seed,mode,delay
                    if key in done:
                        continue
                    release_memory()
                    row,arrays=execute(s,mode,delay)
                    if mode=='dive' and delay==0.:
                        calibrated=next(c for c in calibration_data['scenarios'] if c['seed']==s.seed)
                        assert row['physics_sha256']==calibrated['d0_verification']['physics_sha256']
                    assert row['physics_samples']==row['horizon_step']*20+1
                    assert row['physical_state_unchanged_at_initiation'] and row['pid_unchanged_at_initiation']
                    assert row['geometry_unchanged']
                    assert row['max_physics_position_increment_m']<.01
                    if mode in ('dive','sideways'):
                        assert row['final_position_m'][1]<row['maneuver_anchor_m'][1]
                    if s.seed==protocol['representative_seed'] and delay in protocol['representative_starts']:
                        prefix=f'seed{s.seed}_{mode}_{round(delay*50):03d}'
                        for kind,value in arrays.items():
                            representatives[prefix+'__'+kind]=value
                        np.savez_compressed(rep_path,**representatives)
                    append_json(journal,row);rows.append(row);done.add(key);added+=1
                    raw=journal.read_bytes()
                    saved.update(journal_bytes=len(raw),journal_sha256=hashlib.sha256(raw).hexdigest(),completed_flights=len(done))
                    write_json(run_path,saved)
                    if added%10==0 or len(done)==len(expected):
                        print(f'{len(done)}/{len(expected)} seed={s.seed} {mode} +{delay:.2f}s safe={row["safe"]}',flush=True)
                    if max_flights and added>=max_flights:
                        return
    finally:
        raw=journal.read_bytes()
        saved.update(status='complete' if done==expected else 'partial',journal_bytes=len(raw),
                     journal_sha256=hashlib.sha256(raw).hexdigest(),completed_flights=len(done))
        write_json(run_path,saved);write_metrics(out,saved,rows)
        print(f'Saved {len(done)}/{len(expected)} flights; new this invocation {added}',flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--calibration',type=Path,default=ROOT/'results/development/calibration')
    p.add_argument('--out',type=Path,default=ROOT/'results/development/deadline_sweep')
    p.add_argument('--resume',action='store_true')
    p.add_argument('--max-flights',type=int,default=0)
    p.add_argument('--climb-max',type=float,choices=(1.,1.5),default=1.5)
    p.add_argument('--metrics-only',action='store_true')
    a=p.parse_args()
    if a.max_flights<0:
        p.error('--max-flights must be nonnegative')
    with (ROOT/'support/.simulation.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        if a.metrics_only:
            run=json.loads((a.out/'run.json').read_text())
            rows=committed_journal(a.out,run)
            print(write_metrics(a.out,run,rows)['counts'])
        else:
            run_sweep(a.calibration,a.out,a.resume,a.max_flights,a.climb_max)


if __name__=='__main__':
    main()
