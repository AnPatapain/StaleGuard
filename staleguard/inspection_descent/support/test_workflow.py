"""Small migration/scheduler tests. Prescribed states below are not PyBullet results."""
from pathlib import Path
import sys
from staleguard import inspection_descent
from dataclasses import replace
import json
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch,MagicMock
import numpy as np
from staleguard.inspection_descent.scenario import Scenario,ApprovalSnapshot,make_forward_trajectory
from staleguard.model.common import PIDState,capture_pid_state
from staleguard.inspection_descent import simulation as core
from staleguard.inspection_descent.storage import (ROOT,load_calibrated_scenarios,historical_benchmark,
    normalize_historical,save_snapshot,check_approval_snapshot)
from staleguard.inspection_descent.baseline_benchmark.metrics import policy_metrics
from staleguard.inspection_descent.baselines import fixed_ttl,fixed_early_recovery,ppr


class WorkflowTests(unittest.TestCase):
    def test_canonical_architecture_and_pickle_classes(self):
        import ast
        from staleguard.inspection_descent import scenario
        from staleguard.inspection_descent.baselines import staleguard, use_time_recheck, fbs
        obsolete = {'experiment','geometry','ground_truth','mission','staleguard_benefit',
                    'max_dwell_calibration','independent_evaluation','independent_report'}
        for name in obsolete | {'staleguard','use_time_recheck','fbs','physics_monitor'}:
            self.assertFalse((ROOT/(name+'.py')).exists())
        for name in ('diagnostics','result','model','support/legacy_benefit.py'):
            self.assertFalse((ROOT/name).exists())
        for module,function in ((scenario,scenario.generate_scenario),(staleguard,staleguard.predict),
                                (fbs,fbs.update_volatility),(use_time_recheck,use_time_recheck.predict_immediate)):
            self.assertEqual(function.__module__,module.__name__)
        for path in ROOT.rglob('*.py'):
            if any(part in ('results','independent_evaluation_20_319') for part in path.parts):
                continue
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node,ast.ImportFrom):
                    self.assertFalse(any(n.name=='*' for n in node.names),path)
                    self.assertNotIn((node.module or '').split('.')[0],obsolete,path)
        frozen=load_calibrated_scenarios(ROOT/'results/evaluation/calibration')[0][2]
        self.assertEqual(type(frozen).__module__,'staleguard.inspection_descent.scenario')
        self.assertEqual(type(frozen.pid_state).__module__,'staleguard.model.common')

    def test_scientific_function_bodies_match_precleanup_evidence(self):
        import ast,hashlib
        evidence=json.loads((ROOT.parent/'source_relocation.json').read_text())
        count=0
        for filename,expected in evidence['scientific_nodes'].items():
            tree=ast.parse((ROOT.parent/filename).read_text())
            nodes={n.name:n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
            for name,checksum in expected.items():
                self.assertEqual(hashlib.sha256(ast.dump(nodes[name]).encode()).hexdigest(),checksum)
                count+=1
        self.assertEqual(count,39)

    def test_relocated_sources_reject_unknown_hashes_and_changes(self):
        from staleguard.geofence_turn import storage
        evidence=json.loads((ROOT.parent/'source_relocation.json').read_text())
        old='inspection_descent/model/common.py'
        checksum=evidence['sources'][old]['sha256']
        storage.check_source(ROOT.parent/old,checksum)
        with self.assertRaises(ValueError):
            storage.check_source(ROOT.parent/old,'0'*64)
        original=storage.file_hash
        def tampered(path):
            return '0'*64 if Path(path).resolve()==(ROOT.parent/'model/common.py').resolve() else original(path)
        with patch.object(storage,'file_hash',side_effect=tampered), self.assertRaises(ValueError):
            storage.check_source(ROOT.parent/old,checksum)

    def test_frozen_datasets_and_snapshots_are_readable(self):
        for kind,count,attempted in [('development',19,20),('evaluation',276,300)]:
            folder=ROOT/f'results/{kind}/calibration'
            rows=load_calibrated_scenarios(folder)
            self.assertEqual(len(rows),count)
            self.assertEqual(json.loads((folder/'calibrated_scenarios.json').read_text())['statistics']['attempted'],attempted)
        _,_,snapshot,_=rows[0]
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'copy.pkl';save_snapshot(path,snapshot)
            import pickle
            check_approval_snapshot(pickle.loads(path.read_bytes()),snapshot)
            changed=replace(snapshot,pid_state=PIDState(snapshot.pid_state.integral_pos_e+.01,
                snapshot.pid_state.last_rpy,snapshot.pid_state.integral_rpy_e))
            with self.assertRaises(AssertionError):
                check_approval_snapshot(changed,snapshot)

    def test_historical_counts_and_reuse_are_preserved(self):
        _,rows=historical_benchmark()
        self.assertEqual(len(rows),18216)
        self.assertEqual(sum(r['execution_kind']=='native' for r in rows),12980)
        expected={'no_recovery':(1841,930),'always_recover':(0,276),'fixed_ttl':(139,819),
                  'fbs':(1841,930),'use_time_recheck':(63,864),'staleguard':(22,917)}
        converted=[normalize_historical(r) for r in rows]
        for policy,(unsafe,completed) in expected.items():
            metrics=policy_metrics([r for r in converted if r['event']['policy']==policy],policy)
            self.assertEqual((metrics['unsafe'],metrics['safe_completion']),(unsafe,completed))

    def test_fixed_boundaries(self):
        self.assertFalse(fixed_ttl.fixed_ttl_expired_at_dispatch(15,15))
        self.assertTrue(fixed_ttl.fixed_ttl_expired_at_dispatch(15,16))
        self.assertFalse(fixed_early_recovery.expired(14))
        self.assertTrue(fixed_early_recovery.expired(15))

    def fixture(self,policy,delay_steps,computation=.001,prediction_safe=True):
        s=Scenario(.5,1.2,10.,11.,.35,1.,.04,.6,.4)
        states=np.zeros((61,12));states[:,0]=np.arange(61)*.01;states[:,1]=.5;states[:,4]=1.2
        env=SimpleNamespace(index=0,X_GOAL=np.zeros((451,12)),CTRL_FREQ=50,
            reset=lambda **kw:(states[0].copy(),{}),close=lambda:None)
        def advance(action):
            env.index+=1;return states[env.index].copy(),0,False,{}
        env.step=advance
        ctrl=SimpleNamespace(integral_pos_e=np.zeros(3),last_rpy=np.zeros(3),integral_rpy_e=np.zeros(3),
            reset_before_run=lambda **kw:None,select_action=lambda *a:None,close=lambda:None)
        reference=make_forward_trajectory(env,s)
        snapshot=ApprovalSnapshot(states[2].copy(),capture_pid_state(ctrl),reference[2:].copy(),
            s.low_height,20,s.box_x_min,s.box_x_max,s.box_y_limit,s.unsafe_below_z,s.approval_time,423)
        event=dict(policy=policy,scheduled_dispatch_step=2+delay_steps,application_step=None,
                   fallback_step=None,early_recovery=False,prediction_ready=None)
        received=dict(classification='FINITE_TTL',results=None,first_bad_delay=.04,last_safe_delay=.02)
        kwargs=dict(early_policy=policy,event=event)
        if policy=='staleguard_early_recovery':
            kwargs['ttl_prediction']=(received,computation)
        values=iter([v for _ in range(100) for v in (0.,computation)])
        with patch.object(core,'make_scg_env_factory',return_value=lambda **kw:env),\
             patch.object(core,'make',return_value=ctrl),\
             patch.object(core,'monitor_physics_steps',return_value=lambda:None),\
             patch.object(core,'time',SimpleNamespace(perf_counter=lambda:next(values))),\
             patch('staleguard.inspection_descent.storage.check_memory'),\
             patch.object(ppr,'predict_recovery',return_value=dict(safe=prediction_safe)) as predict:
            trace=[]
            outcome=core.run_actual(s,snapshot,2+delay_steps,2+delay_steps,60,False,
                                    trace=trace,**kwargs)
        return event,trace,predict

    def test_early_ttl_waits_for_availability_and_cancels(self):
        for duration,step in [(.001,4),(.061,6)]:
            event,trace,_=self.fixture('staleguard_early_recovery',20,duration)
            self.assertEqual(event['fallback_step'],step)
            self.assertIsNone(event['application_step'])
            self.assertTrue(event['early_recovery'])
            # Forward flight is retained; no stop, hover, reset or mission retry.
            self.assertTrue(all(row[2][1]>.0 for row in trace[4:]))
            self.assertTrue(all(row[2][4]==1.2 for row in trace))

    def test_fixed_early_uses_inspection_timeout_and_no_model(self):
        for delay in (15,16,30):
            event,trace,predict=self.fixture('fixed_early_recovery',delay)
            self.assertEqual(event['fallback_step'],17)
            self.assertEqual(event['early_recovery'],delay>15)
            self.assertIsNone(event['application_step']);predict.assert_not_called()

    def test_ppr_captures_current_state_and_respects_latency(self):
        for duration,offset in [(.001,1),(.020,2),(.041,3)]:
            event,trace,predict=self.fixture('ppr',20,duration,False)
            check=event['ppr_checks'][0]
            self.assertEqual(check['observation_step'],2)
            self.assertEqual(event['fallback_step'],2+offset)
            self.assertIsNone(event['application_step'])
            self.assertEqual(predict.call_count,1)
            self.assertFalse(hasattr(predict.call_args.args[0],'dispatch_step'))
            np.testing.assert_array_equal(predict.call_args.args[0].state,trace[2][1])
            self.assertEqual(check['action_step'],check['available_step'])

    def test_ppr_safe_checks_repeat_without_turn_check_or_future_delay(self):
        event,trace,predict=self.fixture('ppr',20)
        self.assertEqual([c['observation_step'] for c in event['ppr_checks']],[2,7,12,17])
        self.assertEqual(event['application_step'],22)
        self.assertIsNone(event['fallback_step'])
        self.assertEqual(predict.call_count,4)
        for call in predict.call_args_list:
            snapshot,usual,platform=call.args
            np.testing.assert_array_equal(snapshot.remaining_reference[:,4],usual)
        event,_,predict=self.fixture('ppr',0)
        self.assertEqual(event['application_step'],2);predict.assert_not_called()

    def test_ppr_late_result_never_retroactively_cancels(self):
        event,trace,predict=self.fixture('ppr',10,.3,False)
        self.assertEqual(predict.call_count,1)
        self.assertIsNone(event['fallback_step']);self.assertEqual(event['application_step'],12)
        self.assertTrue(event['ppr_checks'][0]['discarded_at_dispatch'])
        self.assertTrue(event['ppr_checks'][0]['timing_bound_exceeded'])

    def test_accuracy_reproduces_saved_development_classifications(self):
        from staleguard.inspection_descent.ttl_pred_accuracy.run import historical_truth
        from staleguard.inspection_descent.ttl_pred_accuracy.metrics import summarize
        cases=load_calibrated_scenarios(ROOT/'results/development/calibration')
        truth,_=historical_truth(cases)
        self.assertEqual(len(truth),19)
        metrics=summarize(ROOT/'results/development/ttl_pred_accuracy/executions.jsonl')
        self.assertEqual(metrics['recorded_comparisons'],209)
        for r in metrics['ttl']['per_scenario']:
            self.assertEqual(r['actual'],truth[r['seed']])
        for field in ('raw_prediction','ttl_dispatch_decision'):
            c=metrics[field]
            self.assertEqual(c['total_tests'],sum(c[k] for k in
                ('correct_safe','correct_unsafe','false_safe','false_rejection')))

    def test_benchmark_resume_and_duplicate_checks(self):
        from staleguard.inspection_descent.baseline_benchmark import run as runner
        from staleguard.inspection_descent.storage import metadata
        case=load_calibrated_scenarios(ROOT/'results/evaluation/calibration')[0]
        seed,s,snapshot,path=case
        _,history=historical_benchmark()
        original=normalize_historical(next(r for r in history if r['seed']==seed and
            r['delay_steps']==0 and r['policy']=='No Recovery'))
        warm=MagicMock();warm.start.return_value=(SimpleNamespace(select_action=lambda *a:None),None,None,None)
        forecast=dict(classification='FINITE_TTL',results=None,first_bad_delay=.5,last_safe_delay=.4)
        def flown(s,delay,policy,**kwargs):
            row=json.loads(json.dumps(original));row['origin']='test_fixture';row['event']['policy']=policy
            if policy=='staleguard':
                row['event'].update(computation_s=.001,prediction_ready=False,expiry_steps=25)
            return row,snapshot,None
        with tempfile.TemporaryDirectory() as folder:
            out=Path(folder)/'benchmark'
            args=SimpleNamespace(calibration=ROOT/'results/evaluation/calibration',out=out,seeds=[seed],
                delays=[0.],policies=['no_recovery','staleguard'],max_flights=1)
            with patch.object(runner,'load_scg_platform_params',return_value=(None,None)),\
                 patch.object(runner,'LightweightPredictiveModel',return_value=warm),\
                 patch.object(runner,'predict_at_reception',return_value=(forecast,.001)) as predicted,\
                 patch.object(runner,'execute',side_effect=flown) as executed,\
                 patch.object(runner,'check_memory'):
                runner.run(args);self.assertEqual(executed.call_count,1)
                args.max_flights=0;runner.run(args);self.assertEqual(executed.call_count,2)
                self.assertEqual(predicted.call_count,1)
                before=(out/'executions.jsonl').read_bytes();runner.run(args)
                self.assertEqual(executed.call_count,2);self.assertEqual((out/'executions.jsonl').read_bytes(),before)
                self.assertEqual({p.name for p in out.iterdir()},{'run.json','metrics.json','executions.jsonl'})
                with (out/'executions.jsonl').open('ab') as stream:
                    stream.write(before.splitlines(keepends=True)[0])
                with self.assertRaisesRegex(ValueError,'Duplicate'):
                    runner.run(args)

    def test_calibration_completed_seed_resume_uses_original_planner(self):
        from staleguard.inspection_descent import calibration
        from staleguard.inspection_descent.storage import load_calibrated_scenarios
        seed,selected,snapshot,_=load_calibrated_scenarios(ROOT/'results/development/calibration')[0]
        with tempfile.TemporaryDirectory() as folder,\
             patch.object(calibration,'command_candidates',return_value=[selected]) as candidates,\
             patch.object(calibration,'inspection_dwell_ticks',return_value=261),\
             patch.object(calibration,'run_actual',return_value=(True,False,False,True)),\
             patch.object(calibration,'capture_approval',return_value=snapshot),\
             patch.object(calibration,'execute',return_value=(dict(safe_completion=True),snapshot,None)),\
             patch.object(calibration,'check_memory'):
            output=Path(folder)/'calibration'
            data=calibration.calibrate_scenarios([seed],output)
            self.assertEqual(data['statistics'],dict(attempted=1,accepted=1,rejected=0))
            calibration.calibrate_scenarios([seed],output,resume=True)
            self.assertEqual(candidates.call_count,1)
            self.assertEqual(load_calibrated_scenarios(output)[0][0],seed)


if __name__=='__main__':
    unittest.main()
