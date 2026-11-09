"""Development-only profiling and reference/physical-label comparison, no flights."""
from pathlib import Path
import sys
if __package__ in (None,''):sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import underpass_inspection
import argparse
import time
import numpy as np
from model.scg import load_scg_platform_params
from underpass_inspection.storage import ROOT,load_calibrated_scenarios,load_ground_truth,write_json,metadata
from underpass_inspection.baselines.staleguard import predict_delays,segment_box_clearance
from underpass_inspection.baselines.recovery_horizons import predict,segment_safe


def run(out):
    cases=load_calibrated_scenarios(ROOT/'results/development/calibration');platform=load_scg_platform_params()
    truth,_=load_ground_truth(ROOT/'results/development/deadline_sweep')
    strategies={'reference_full':dict(backend='reference',early_exit=False,broadphase=False),
                'compiled_full':dict(early_exit=False,broadphase=False),
                'compiled_early_exact':dict(early_exit=True,broadphase=False),
                'compiled_early_broadphase':dict(early_exit=True,broadphase=True)}
    cold=time.perf_counter();predict(cases[0][1],platform);startup=time.perf_counter()-cold
    measurements={name:[] for name in strategies};predictions={};mismatches=[];max_state=max_pid=0.
    quality={k:dict(false_safe=0,false_reject=0,total=0) for k in ('command','completion','recovery')}
    for s,snapshot,_ in cases:
        original=predict_delays(snapshot,platform)
        reference=predict(snapshot,platform,**strategies['reference_full'],diagnostics=True)
        full=predict(snapshot,platform,**strategies['compiled_full'],diagnostics=True)
        for a,b in zip(reference['rollouts'],full['rollouts']):
            max_state=max(max_state,float(np.max(np.abs(np.array(a['terminal_state'])-b['terminal_state']))))
            max_pid=max(max_pid,float(np.max(np.abs(np.array(a['terminal_pid'])-b['terminal_pid']))))
        assert max_state<1e-12 and max_pid<1e-12
        assert [r['safe'] for r in original['results']]==[r['command_safe'] for r in reference['results']]
        for name,kwargs in strategies.items():
            for repeat in range(3):
                result=predict(snapshot,platform,**kwargs)
                measurements[name].append(dict(seed=s.seed,repeat=repeat,seconds=result['computation_s']))
                if result['results']!=reference['results']:mismatches.append((s.seed,name))
        assert not mismatches
        predictions[str(s.seed)]=result
        for row in result['results']:
            d=row['delay_s'];cmd=truth[s.seed,'dive',d];rec=truth[s.seed,'sideways',d]
            for key,predicted,actual in [('command',row['command_safe'],cmd['safe']),
                ('completion',row['command_complete'],cmd['safe_completion']),('recovery',row['recovery_safe'],rec['safe'])]:
                q=quality[key];q['total']+=1;q['false_safe']+=predicted and not actual;q['false_reject']+=actual and not predicted
    rng=np.random.default_rng(42);s=cases[0][0];boxes=np.asarray([[b.lower,b.upper] for b in s.obstacles])
    for _ in range(10000):
        a=rng.uniform(-2,6,12);b=rng.uniform(-2,6,12)
        assert segment_safe(a,b,boxes,s.clearance_m,True)==segment_safe(a,b,boxes,s.clearance_m,False)
    timing={name:dict(samples=len(values),p50_s=float(np.median([v['seconds'] for v in values])),
        p95_s=float(np.quantile([v['seconds'] for v in values],.95,method='higher'))) for name,values in measurements.items()}
    report=dict(provenance=metadata(),evidence='Development-only predictive diagnostic; physical labels from independent archived sweeps',
        startup_compile_warmup_s=startup,timing=timing,measurements=measurements,
        profile_disagreements=mismatches,max_state_error=max_state,max_pid_error=max_pid,
        original_command_label_disagreements=0,physical_label_quality=quality,broadphase_random_checks=10000,
        predictions=predictions,completion_vs_safety_differences=sum(r['command_safe']!=r['command_complete']
            for p in predictions.values() for r in p['results']))
    out.mkdir(parents=True,exist_ok=True);write_json(out/'optimization.json',report)
    print('TIMING',timing);print('CORRECTNESS',quality,'max errors',max_state,max_pid)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,default=ROOT/'results/development/recovery_horizons')
    run(p.parse_args().out)
