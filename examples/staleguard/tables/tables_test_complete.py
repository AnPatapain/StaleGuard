"""Focused core semantics and numerical adapter checks; no physical benchmark."""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import unittest,inspect,importlib
from tempfile import TemporaryDirectory
import numpy as np
from model.feasibility_profiles import finalize,safe_segment,residence,advance,kernel_parameters
from model.mission_evaluation import clock_action,cases,prediction
from model.scg import load_scg_platform_params
from model.lightweight import LightweightPredictiveModel,rk4
from underpass_inspection.baselines.policies import HorizonCommand


def reference_safety(mission,s,snapshot,platform):
    """Independent existing LightweightPID/model.step controller/checkpoint path."""
    a=importlib.import_module(mission+'.baselines.recovery_horizons').adapter(s,snapshot)
    model=LightweightPredictiveModel(*platform);checkpoints={};reference=snapshot.remaining_reference[:a['horizon_steps']+1].copy()
    try:
        ctrl,obs,info,_=model.start(snapshot);model.set_reference(ctrl,reference);prefix=a['prefix_safe']
        wanted={round(d*50) for d in a['grid']}
        for step in range(max(wanted)+1):
            if step in wanted:checkpoints[step]=(model.save_checkpoint(ctrl,obs,info),prefix)
            if step==max(wanted):break
            action=ctrl.select_action(obs,info);prev=obs;obs,_,info=model.step(action,obs,info)
            prefix=prefix and safe_segment.py_func(prev,obs,a['kind'],np.array(a['geometry']))
        rows=[]
        for d in a['grid']:
            result=[];step=round(d*50)
            for recovery in (True,False):
                checkpoint,prefix=checkpoints[step];obs,info=model.restore_checkpoint(ctrl,checkpoint)
                changed=a['reference'](reference,step,obs,recovery);model.set_reference(ctrl,changed);safe=prefix
                for tick in range(step,a['horizon_steps']):
                    prev=obs;obs,_,info=model.step(ctrl.select_action(obs,info),obs,info)
                    safe=safe and safe_segment.py_func(prev,obs,a['kind'],np.array(a['geometry']))
                result.append((safe,obs.copy(),np.array([ctrl.integral_pos_e,ctrl.last_rpy,ctrl.integral_rpy_e])))
            rows.append((d,result))
        return rows
    finally:model.close(checkpoints)


class CompletionTests(unittest.TestCase):
    def test_physical_reuse_keeps_independent_policy_event(self):
        import complete_experiments as workflow
        row=dict(seed=1,delay_s=.2,kind='recovery',safe=True,completed=False,safe_completion=False,
                 first_violation_s=None,horizon_s=4.2,runtime_s=.4,
                 event=dict(policy='always_recover',application_step=None,fallback_step=10))
        event=dict(policy='staleguard_dual_horizon',scheduled_dispatch_step=50,application_step=None,
                   fallback_step=10,computation_s=.041,command_canceled=True,events=[])
        result=workflow.policy_reuse(row,'inspection_descent',1.,event,('recovery',10),workflow.ROOT/'inspection_descent/results/evaluation/supplementary_feasibility_v1')
        self.assertEqual(workflow.key(result),(1,'staleguard_dual_horizon',50))
        self.assertEqual(result['event']['computation_s'],.041)
        self.assertTrue(result['event']['early_recovery'])
        self.assertEqual(result['physical_source']['policy'],'always_recover')
        self.assertIsNone(result['runtime_s'])
        with self.assertRaises(AssertionError):
            workflow.policy_reuse(row,'inspection_descent',1.,event,('recovery',11),workflow.ROOT)
    def test_journal_resume_integrity_and_duplicate_rejection(self):
        from underpass_inspection import storage
        import complete_experiments as workflow
        with TemporaryDirectory() as folder:
            path=Path(folder)
            run=storage.initialize_run(path,dict(protocol=dict(scenarios=[]),provenance=storage.metadata()))
            row=dict(seed=1,delay_s=0.,kind='recovery',event=dict(policy='always_recover'),safe=True,completed=False,safe_completion=False)
            digest=storage.committed_hasher(path,run)
            storage.commit_record(path,run,row,digest)
            committed=(path/'executions.jsonl').read_bytes()
            with (path/'executions.jsonl').open('ab') as f:f.write(b'{"unfinished":')
            resumed=storage.read_run(path,repair=True)
            self.assertEqual((path/'executions.jsonl').read_bytes(),committed)
            self.assertEqual(resumed['completed_flights'],1)
            storage.commit_record(path,resumed,row,storage.committed_hasher(path,resumed))
            with self.assertRaises(AssertionError):workflow.finish(path,resumed,{(1,'recovery',0)})
            with (path/'executions.jsonl').open('r+b') as f:f.write(b'X')
            with self.assertRaises(ValueError):storage.read_run(path,repair=True)
    def forecast(self):
        return finalize([dict(delay_s=d,command_safe=d<.8,command_complete=d<.4,recovery_safe=d<.6) for d in (0.,.2,.4,.6,.8,1.)],(0.,.2,.4,.6,.8,1.))
    def test_same_core_and_unknown_delay(self):
        self.assertNotIn('delay',inspect.signature(HorizonCommand.approve).parameters)
        snap=type('Snapshot',(),dict(approval_time=1.,horizon_steps=210))()
        f=self.forecast();f['computation_s']=.1
        a,e=clock_action('staleguard_recovery_aware',snap,1.,f)
        self.assertEqual(a,('recovery',70)) # last safe recovery sample = .4
        a,e=clock_action('staleguard_dual_horizon',snap,1.,f);self.assertEqual(a,('recovery',70))
        self.assertIsNone(e['application_step']);self.assertTrue(e['command_canceled'])
        for age in (0.,.4,1.):
            a,e=clock_action('always_recover',snap,age);self.assertEqual(a,('recovery',50));self.assertIsNone(e['application_step'])
        a,e=clock_action('fixed_early_recovery',snap,1.);self.assertEqual(a,('recovery',60))
        f['computation_s']=2.;a,e=clock_action('staleguard_dual_horizon',snap,.2,f);self.assertEqual(a,('command',60))
    def test_closed_safety_and_strict_completion(self):
        a=np.zeros(12);b=a.copy();g=np.array([1.,0.,0.,0.,0.,0.])
        a[0]=b[0]=1.;self.assertTrue(safe_segment(a,b,0,g))
        b[0]=1.001;self.assertFalse(safe_segment(a,b,0,g))
        lo=np.array([-1.,-1.,-1.]);hi=-lo;a=np.zeros(12);b=a.copy();a[0]=b[0]=1.
        self.assertEqual(residence(a,b,lo,hi,np.zeros(3),0.,False,True,0.,False,.2,.18),(0.,False))
        self.assertEqual(residence(a,b,lo,hi,np.zeros(3),0.,False,False,0.,False,.2,.18),(.2,True))
    def test_frozen_inputs_and_compiled_pid_equivalence(self):
        platform=load_scg_platform_params()
        for mission,n in [('inspection_descent',276),('geofence_turn',300)]:
            data=cases(mission);self.assertEqual(len(data),n);seed,s,snap,path=data[0]
            compiled=prediction(mission,s,snap,platform,early_exit=False,diagnostics=True)
            old=reference_safety(mission,s,snap,platform)
            for (d,continuations),r in zip(old,compiled['results']):
                self.assertEqual(r['delay_s'],d)
                for recovery,(safe,end,memory) in zip((True,False),continuations):
                    self.assertEqual(r['recovery_safe' if recovery else 'command_safe'],safe)
                    trace=next(t for t in compiled['diagnostics'] if t['delay_s']==d and t['recovery']==recovery)
                    np.testing.assert_allclose(trace['state'],end,rtol=0,atol=2e-12)
                    np.testing.assert_allclose(trace['memory'],memory,rtol=0,atol=2e-12)

if __name__=='__main__':unittest.main()
