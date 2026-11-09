"""Focused extension tests; no held-out flights and no original policies rerun."""
from copy import deepcopy
from dataclasses import replace
import inspect
import json
from pathlib import Path
import unittest
from tempfile import TemporaryDirectory
from unittest.mock import patch
import numpy as np
from model.scg import load_scg_platform_params
from underpass_inspection.storage import ROOT,physical_source_hash,REVIEWED,load_calibrated_scenarios
from underpass_inspection.baselines import recovery_horizons as predictor
from underpass_inspection.baselines.recovery_policies import HorizonCommand,POLICIES
from underpass_inspection.baselines.staleguard import TTL_GRID,first_ready_step
from underpass_inspection.scenario import Scenario


def forecast(command=10,recovery=30,completion=None,runtime=.02):
    completion=command if completion is None else completion
    def boundary(step):
        return dict(first_invalid_steps=step,last_safe_prefix_steps=None if step is None or step==0 else step-1,
                    last_tested_safe_steps=50 if step is None else step-1,right_censored=step is None,non_monotonic=False)
    b={k:boundary(v) for k,v in [('command',command),('recovery',recovery),('completion',completion)]}
    plans={}
    for p in POLICIES:
        candidates=[x for x in (command,None if recovery is None else max(0,recovery-1),completion if p==POLICIES[1] else None) if x is not None]
        plans[p]=dict(trigger_steps=min(candidates,default=None),reasons=['fixture'],
                      recovery_verified_through_steps=50 if recovery is None else b['recovery']['last_safe_prefix_steps'])
    return dict(computation_s=runtime,expiry_steps=command,boundaries=b,policy_plans=plans)


class HorizonPolicyTests(unittest.TestCase):
    def agent(self,policy=POLICIES[0],**kwargs):
        a=HorizonCommand(policy,Scenario(),forecast(**kwargs));a.approve();return a

    def advance(self,a,age,dispatch=False):
        step=a.approval+age;a.announce_readiness(step)
        if dispatch:a.dispatch(step,None,None)
        a.tick(step);return a.take_action()

    def test_no_future_schedule_interface(self):
        for call in (predictor.predict,HorizonCommand.__init__,HorizonCommand.tick,HorizonCommand.approve):
            self.assertFalse({'delay_s','dispatch_time','dispatch_step','ground_truth','recovery_deadline'}&set(inspect.signature(call).parameters))
        a=self.agent();self.assertNotIn('scheduled_dispatch_step',a.event)
        self.assertIsNone(a.event['dispatch_observed_step'])

    def test_recovery_boundary_earlier_than_command(self):
        a=self.agent(command=20,recovery=6)
        self.assertIsNone(self.advance(a,4));self.assertEqual(self.advance(a,5),'sideways')
        self.assertIsNone(self.advance(a,40,dispatch=True))
        self.assertIsNone(a.event['application_step']);self.assertTrue(a.event['command_canceled'])
        self.assertTrue(a.event['predicted_recovery_verified_at_start'])

    def test_dual_completion_trigger_and_admission(self):
        for dispatch in (False,True):
            a=self.agent(command=20,recovery=30,completion=5)
            b=self.agent(POLICIES[1],command=20,recovery=30,completion=5)
            self.assertEqual(self.advance(a,5,dispatch), 'dive' if dispatch else None)
            self.assertEqual(self.advance(b,5,dispatch),'sideways')

    def test_dispatch_wins_timer_tie(self):
        a=self.agent(command=20,recovery=6)
        self.assertEqual(self.advance(a,5,True),'dive')
        self.assertIsNone(a.event['fallback_step'])
        b=self.agent(command=5,recovery=20)
        self.assertEqual(self.advance(b,5,True),'sideways')

    def test_unavailable_is_fail_open_and_cannot_retroactively_recover(self):
        a=self.agent(command=5,runtime=.121)
        self.assertEqual(self.advance(a,5,True),'dive')
        self.assertIsNone(self.advance(a,10));self.assertIsNone(a.event['fallback_step'])
        b=self.agent(command=5,runtime=.121)
        self.assertIsNone(self.advance(b,6));self.assertEqual(self.advance(b,7),'sideways')
        self.assertEqual(b.event['prediction_ready_step'],b.approval+first_ready_step(.121))

    def test_missed_recovery_prefix_preserves_safe_command(self):
        a=self.agent(command=20,recovery=5,runtime=.2)
        self.assertIsNone(self.advance(a,10));self.assertTrue(a.event['recovery_prefix_missed_before_prediction_ready'])
        self.assertEqual(self.advance(a,11,True),'dive')
        b=self.agent(command=20,recovery=5,runtime=.2)
        self.assertIsNone(self.advance(b,10));self.assertEqual(self.advance(b,20),'sideways')
        self.assertTrue(b.event['best_effort_recovery'])

    def test_no_invented_horizon_expiration_and_profiles_atomic(self):
        for policy in POLICIES:
            a=self.agent(policy,command=None,recovery=None)
            self.assertIsNone(self.advance(a,50));self.assertEqual(self.advance(a,100,True),'dive')
            b=self.agent(policy,command=5,runtime=.3)
            self.assertIsNone(self.advance(b,10))

    def test_no_completion_only_abandonment_after_recovery_prefix(self):
        a=self.agent(POLICIES[1],command=30,recovery=5,completion=6,runtime=.2)
        self.assertEqual(self.advance(a,10,True),'dive')

    def test_identical_profiles_produce_identical_decisions(self):
        for delay in (0,4,10,30,100):
            agents=[self.agent(p) for p in POLICIES]
            for step in range(delay+1):
                actions=[self.advance(a,step,step==delay) for a in agents]
                self.assertEqual(*actions)


class HorizonPredictionTests(unittest.TestCase):
    def test_boundary_nonmonotonic_and_censored(self):
        for values,expected in [([True,False,True],(1,0,2,True,False)),([True]*3,(None,None,2,False,True)),([False]*3,(0,None,None,False,False))]:
            rows=[dict(delay_s=i/50,safe=v) for i,v in enumerate(values)]
            b=predictor._boundary(rows,'safe')
            self.assertEqual(tuple(b[k] for k in ('first_invalid_steps','last_safe_prefix_steps','last_tested_safe_steps','non_monotonic','right_censored')),expected)

    def test_inspection_dwell_resets_at_region_exit(self):
        lo=np.zeros(3);hi=np.ones(3);a=np.zeros(12);b=np.zeros(12)
        a[[0,2,4]]=.5;b[[0,2,4]]=.5
        dwell,done=predictor.inspection_segment(a,b,lo,hi,0.,False,.1,.18)
        self.assertFalse(done)
        dwell,done=predictor.inspection_segment(a,b,lo,hi,dwell,done,.1,.18)
        self.assertTrue(done)
        b[0]=2.;dwell,done=predictor.inspection_segment(a,b,lo,hi,.01,False,.1,.18)
        self.assertEqual(dwell,0.);self.assertFalse(done)

    def test_reference_compiled_and_early_exit_equivalence(self):
        s,snapshot,_=load_calibrated_scenarios(ROOT/'results/development/calibration')[0]
        platform=load_scg_platform_params()
        ref=predictor.predict(snapshot,platform,backend='reference',early_exit=False,diagnostics=True)
        full=predictor.predict(snapshot,platform,early_exit=False,broadphase=False,diagnostics=True)
        fast=predictor.predict(snapshot,platform)
        self.assertEqual(ref['grid_s'],list(TTL_GRID));self.assertEqual(len(ref['results']),19)
        self.assertEqual(ref['results'],full['results']);self.assertEqual(ref['results'],fast['results'])
        self.assertEqual(ref['boundaries'],fast['boundaries'])
        for a,b in zip(ref['rollouts'],full['rollouts']):
            np.testing.assert_array_equal(a['terminal_state'],b['terminal_state'])
            np.testing.assert_array_equal(a['terminal_pid'],b['terminal_pid'])
        self.assertLess(sum(r['control_steps'] for r in fast['rollouts']),sum(r['control_steps'] for r in full['rollouts']))
        self.assertLessEqual(fast['recovery_profile_finished_s'],fast['computation_s'])

    def test_unchanged_physical_source(self):
        for name,(_,checksum) in REVIEWED.items():self.assertEqual(physical_source_hash((ROOT/name).read_text(),name),checksum)


class HorizonAnalysisTests(unittest.TestCase):
    def test_metrics_regeneration_uses_only_selected_pairs(self):
        from underpass_inspection.baseline_benchmark import horizons
        run=dict(protocol=dict(scenarios=[dict(seed=1)],delays=[0.,.2],policies=[]),predictions_by_policy={})
        truth={(1,'dive',0.):{},(1,'dive',.2):{},(1,'dive',.4):{},(2,'dive',0.):{}}
        with TemporaryDirectory() as folder, patch.object(horizons.metrics,'journal_metrics',return_value=dict(predictor={})) as call, \
             patch.object(horizons.metrics,'write_outputs'),patch.object(horizons,'iter_committed',return_value=iter(())):
            horizons.write_results(Path(folder),run,truth)
            self.assertEqual(set(call.call_args.args[2]),{(1,'dive',0.),(1,'dive',.2)})

    def test_nine_policy_pairs_and_scenario_clustered_intervals(self):
        from underpass_inspection.baselines.policies import POLICIES as old
        from underpass_inspection.baseline_benchmark import metrics
        source=ROOT/'results/development/baseline_benchmark/executions.jsonl'
        with source.open() as f:template=json.loads(next(f))
        protocol=dict(scenarios=[dict(seed=s) for s in (1,2)],delays=[0.,.2],policies=list(old+POLICIES),
                      uncertainty=dict(unit='scenario',method='paired percentile bootstrap',confidence=.95,resamples=2000,seed=2027,complete_scenarios_only=True))
        rows=[]
        for seed in (1,2):
            for d in protocol['delays']:
                for name in protocol['policies']:
                    r=deepcopy(template);r.update(seed=seed,delay_s=d,safe=(name in POLICIES or seed==1),safe_completion=False)
                    r['event'].update(policy=name,observation_update_s=[]);rows.append(r)
        result=metrics.summarize(rows,dict(protocol=protocol,predictions={}))
        for seed,group in result['by_scenario'].items():
            local=[r for r in rows if r['seed']==int(seed)]
            group['paired_early_recovery']={p:metrics.paired_metrics(local,other=p,reference='staleguard_early_recovery')
                for p in ('staleguard','use_time_recheck','fixed_early_recovery')}
        uncertainty=metrics.scenario_uncertainty(result,protocol)
        self.assertEqual(uncertainty['complete_scenarios'],2)
        self.assertEqual(len(result['paired_extensions']),7)
        key=POLICIES[0]+'__vs__staleguard_early_recovery'
        self.assertEqual(uncertainty['paired_extensions'][key]['net_unsafe_reduction']['mean'],.5)
        self.assertEqual(uncertainty['paired_extensions'][key]['net_unsafe_reduction']['ci95'],[0.,1.])
        with self.assertRaises(ValueError):metrics.summarize(rows+[rows[0]],dict(protocol=protocol,predictions={}))


if __name__=='__main__':unittest.main()
