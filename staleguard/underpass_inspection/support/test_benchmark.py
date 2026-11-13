"""Unified benchmark storage, selection and semantic regression tests; no flights."""
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from copy import deepcopy
import hashlib,json,inspect,unittest
from unittest.mock import patch
import numpy as np
from staleguard.underpass_inspection import storage
from staleguard.underpass_inspection.baselines.policies import POLICIES,HORIZON_POLICIES,make_policy,HorizonCommand,PendingCommand
from staleguard.underpass_inspection.baselines.staleguard import TTL_GRID,first_ready_step,staleguard_status,predict_delays
from staleguard.underpass_inspection.baseline_benchmark.run import select_cases,DISPATCH_DELAYS,SAMPLE_SEED
from staleguard.underpass_inspection.baseline_benchmark import metrics
from staleguard.underpass_inspection.support.test_recovery_horizons import forecast
from staleguard.underpass_inspection.scenario import Scenario


class BenchmarkTests(unittest.TestCase):
    def config(self):return dict(protocol=dict(scenarios=[]),provenance=storage.metadata(),inputs_sha256='fixture')

    def test_nine_policy_registry_and_factory(self):
        self.assertEqual(len(POLICIES),9);self.assertEqual(len(set(POLICIES)),9)
        for policy in POLICIES:
            received=forecast() if policy in HORIZON_POLICIES else dict(computation_s=.05,expiry_steps=10) if policy.startswith('staleguard') else None
            agent=make_policy(policy,Scenario(),received)
            self.assertEqual(agent.event['policy'],policy)
            self.assertEqual(isinstance(agent,HorizonCommand),policy in HORIZON_POLICIES)
        for call in (make_policy,PendingCommand.approve,PendingCommand.tick,predict_delays):
            self.assertFalse({'delay_s','dispatch_time','dispatch_step','ground_truth'}&set(inspect.signature(call).parameters))

    def test_disjoint_sampling_is_deterministic_and_order_independent(self):
        cases=[(SimpleNamespace(seed=i),None,None) for i in range(1001,1866)]
        excluded=set(range(1001,1101));eligible=list(range(1101,1866))
        actual=[s.seed for s,_,_ in select_cases(cases,excluded,400,SAMPLE_SEED)]
        expected=sorted(np.random.default_rng(20261109).choice(eligible,400,replace=False).tolist())
        self.assertEqual(actual,expected);self.assertFalse(set(actual)&excluded)
        self.assertEqual(actual,[s.seed for s,_,_ in select_cases(cases[::-1],excluded,400,SAMPLE_SEED)])
        self.assertEqual(400*len(DISPATCH_DELAYS)*len(POLICIES),46800)
        self.assertEqual(DISPATCH_DELAYS,(0.,.04,.08,.12,.16,.20,.4,.6,.8,1.,1.2,1.6,2.))
        self.assertEqual(TTL_GRID,tuple(i/50 for i in range(11))+tuple(i/10 for i in range(3,11)))

    def test_committed_flights_and_predictions_resume_after_torn_writes(self):
        for kind,filename in [('flight','executions.jsonl'),('prediction','predictions.jsonl')]:
            with TemporaryDirectory() as d:
                out=Path(d);run=storage.initialize_run(out,self.config())
                row=dict(policy='staleguard',seed=1,forecast=dict(computation_s=.123,expiry_steps=10)) if kind=='prediction' else dict(seed=1,delay_s=0.,event=dict(policy='no_recovery'))
                digest=storage.committed_hasher(out,run,kind=kind)
                storage.commit_record(out,run,row,digest,kind=kind)
                committed=(out/filename).read_bytes()
                with patch.object(storage,'write_progress',side_effect=OSError('interruption')):
                    with self.assertRaises(OSError):storage.commit_record(out,run,row,digest,kind=kind)
                with (out/filename).open('ab') as f:f.write(b'{"incomplete":')
                with self.assertRaises(ValueError):storage.read_run(out)
                loaded=storage.read_run(out,repair=True)
                self.assertEqual((out/filename).read_bytes(),committed)
                if kind=='prediction':self.assertEqual(loaded['predictions']['1'],row['forecast'])
                else:self.assertEqual(loaded['completed_flights'],1)
                with (out/filename).open('r+b') as f:f.write(b'X')
                with self.assertRaises(ValueError):storage.read_run(out,repair=True)

    def test_source_config_and_duplicate_prediction_detection(self):
        with TemporaryDirectory() as d:
            out=Path(d);run=storage.initialize_run(out,self.config())
            row=dict(policy='staleguard',seed=1,forecast=dict(computation_s=.1,expiry_steps=5))
            digest=storage.committed_hasher(out,run,kind='prediction')
            storage.commit_record(out,run,row,digest,kind='prediction')
            storage.commit_record(out,run,row,digest,kind='prediction')
            with self.assertRaises(ValueError):storage.read_run(out)
        with TemporaryDirectory() as d:
            out=Path(d);run=storage.initialize_run(out,self.config());config=json.loads((out/'run.json').read_text())
            config['protocol']['new_setting']=True;storage.write_json(out/'run.json',config)
            with self.assertRaises(ValueError):storage.read_run(out)
        changed=deepcopy(storage.metadata());changed['source_sha256']['baselines/policies.py']='0'*64
        with self.assertRaises(ValueError):storage.check_provenance(changed)

    def test_ready_equality_and_fail_open_beyond_grid(self):
        for step in range(51):
            self.assertEqual(first_ready_step(step/50),step)
            self.assertEqual(staleguard_status(dict(computation_s=step/50,expiry_steps=step),step),(True,True))
        for name in ('staleguard','staleguard_early_recovery'):
            a=make_policy(name,Scenario(),dict(computation_s=.1,expiry_steps=None));a.approve()
            for age in range(101):
                step=a.approval+age;a.announce_readiness(step)
                if age==100:a.dispatch(step,None,None)
                a.tick(step);action=a.take_action()
                if age<100:self.assertIsNone(action)
                else:self.assertEqual(action,'dive')

    def test_each_policy_can_be_prepared_alone(self):
        from staleguard.underpass_inspection.baseline_benchmark.run import prepare
        from staleguard.underpass_inspection.support.validate_baseline_benchmark import validate
        with TemporaryDirectory() as d:
            for policy in POLICIES:
                out=Path(d)/policy
                args=SimpleNamespace(out=out,calibration=storage.ROOT/'results/development/calibration',
                    policies=[policy],delays=[0.,1.],sample_count=None,smoke=False,seeds=[1])
                prepare(args);result=validate(out)
                self.assertEqual(result['expected_flights'],2);self.assertEqual(result['actual_flights'],0)
                self.assertNotIn('predictions',json.loads((out/'run.json').read_text()))
                self.assertLess((out/'progress.json').stat().st_size,2048)

    def test_duplicate_flight_rejected_by_validator(self):
        from staleguard.underpass_inspection.baseline_benchmark.run import prepare
        from staleguard.underpass_inspection.support.validate_baseline_benchmark import validate
        source=storage.ROOT/'results/development/baseline_benchmark/executions.jsonl'
        with source.open() as f:row=json.loads(next(f))
        with TemporaryDirectory() as d:
            out=Path(d)/'run';args=SimpleNamespace(out=out,calibration=storage.ROOT/'results/development/calibration',
                policies=['no_recovery'],delays=[0.],sample_count=None,smoke=False,seeds=[1])
            prepare(args);run=storage.read_run(out);digest=storage.committed_hasher(out,run)
            storage.commit_record(out,run,row,digest);storage.commit_record(out,run,row,digest)
            run['status']='complete';storage.write_progress(out,run)
            with self.assertRaises(AssertionError):validate(out)

    def test_archived_development_metrics_remain_identical(self):
        out=storage.ROOT/'results/development/baseline_benchmark';run=json.loads((out/'run.json').read_text())
        truth,_=storage.load_ground_truth(storage.ROOT/'results/development/deadline_sweep')
        expected=json.loads((out/'metrics.json').read_text())
        self.assertEqual(metrics.summarize_journal(out,run,truth),expected)
        self.assertEqual(expected['predictor']['forecast_false_safe_labels'],17)
        self.assertEqual(expected['predictor']['direct_grid_false_safe'],1)


if __name__=='__main__':unittest.main()
