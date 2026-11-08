"""Focused freeze, timing, labels and crash-recovery checks; no held-out scores."""
import copy
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
import numpy as np
from underpass_inspection import storage
from underpass_inspection.baselines import staleguard
from underpass_inspection.baselines.policies import PendingCommand
from underpass_inspection.baseline_benchmark import metrics
from underpass_inspection.baseline_benchmark.run import protocol_hash,verify_freeze
from underpass_inspection.scenario import Scenario
from underpass_inspection.support.test_baselines import KinematicModel

ROOT=storage.ROOT


class FinalAuditTests(unittest.TestCase):
    def test_fixed_grid_and_exact_ready_expiry_ties(self):
        self.assertEqual(staleguard.TTL_GRID,tuple(i/50 for i in range(11))+tuple(i/10 for i in range(3,11)))
        for step in range(51):
            runtime=step/50
            self.assertEqual(staleguard.first_ready_step(runtime),step)
            forecast=dict(computation_s=runtime,expiry_steps=step)
            self.assertEqual(staleguard.staleguard_status(forecast,step),(True,True))
            if step:self.assertEqual(staleguard.staleguard_status(forecast,step-1),(False,False))
            a=PendingCommand('staleguard',Scenario(),forecast);a.approve()
            a.announce_readiness(a.approval+step)
            self.assertEqual(a.finish(a.approval+50,a.approval+400)['prediction_ready_control_step'],a.approval+step)

    def test_first_unsafe_cutoff_keeps_nonmonotonic_tail(self):
        _,snapshot,_=storage.load_calibrated_scenarios(ROOT/'results/development/calibration')[0]
        active=[-1]
        from underpass_inspection.scenario import apply_maneuver
        def maneuver(*args):
            active[0]+=1
            return apply_maneuver(*args)
        def clearance(*args):return 0. if active[0]==1 else 1.
        with patch.object(staleguard,'LightweightPredictiveModel',KinematicModel), \
             patch.object(staleguard,'apply_maneuver',side_effect=maneuver), \
             patch.object(staleguard,'segment_clearance',side_effect=clearance):
            result=staleguard.predict_delays(snapshot,(None,None),(0.,.2,1.))
        self.assertEqual([r['safe'] for r in result['results']],[True,False,True])
        self.assertEqual(result['expiry_steps'],10);self.assertTrue(result['non_monotonic'])

    def test_atomic_prefix_rolls_back_full_and_torn_uncommitted_records(self):
        with (ROOT/'results/development/baseline_benchmark/executions.jsonl').open() as stream:
            row=json.loads(next(stream))
        with TemporaryDirectory() as temp:
            out=Path(temp);journal=out/'executions.jsonl';journal.touch()
            run=dict(journal_bytes=0,journal_sha256=hashlib.sha256(b'').hexdigest(),completed_flights=0)
            storage.write_json(out/'run.json',run);digest=storage.committed_hasher(out,run)
            storage.commit_execution(out,run,row,digest)
            committed=journal.read_bytes()
            with patch.object(storage,'write_json',side_effect=OSError('simulated interrupted metadata commit')):
                with self.assertRaises(OSError):storage.commit_execution(out,run,row,digest)
            with journal.open('ab') as stream:stream.write(b'{"seed":')
            saved=json.loads((out/'run.json').read_text())
            with self.assertRaises(ValueError):storage.committed_hasher(out,saved)
            storage.committed_hasher(out,saved,repair=True)
            self.assertEqual(journal.read_bytes(),committed)
            self.assertEqual(list(storage.iter_committed(out,saved)),[row])
            journal.write_bytes(b'X'+committed[1:])
            with self.assertRaises(ValueError):storage.committed_hasher(out,saved,repair=True)

    def test_development_stream_metrics_and_off_grid_false_safes(self):
        out=ROOT/'results/development/baseline_benchmark';run=json.loads((out/'run.json').read_text())
        truth,_=storage.load_ground_truth(ROOT/'results/development/deadline_sweep')
        expected=json.loads((out/'metrics.json').read_text())
        self.assertEqual(metrics.summarize_journal(out,run,truth),expected)
        run=copy.deepcopy(run);run['protocol']['ground_truth_source']='no_recovery'
        actual=metrics.summarize_journal(out,run)
        for key in ('by_policy','by_delay','by_scenario','paired_against_no_recovery','paired_early_recovery'):
            self.assertEqual(actual[key],expected[key])
        self.assertEqual(actual['predictor']['forecast_false_safe_labels'],17)
        self.assertEqual(actual['predictor']['direct_grid_false_safe'],1)

    def test_no_recovery_labels_keep_intervals_and_censoring(self):
        forecast=dict(expiry_steps=1,computation_s=.01,non_monotonic=True,
                      results=[dict(delay_s=i/50,safe=s) for i,s in enumerate((True,False,True))])
        run=dict(protocol=dict(delays=[0.,.02,.04],ground_truth_source='no_recovery'),predictions={'1':forecast})
        rows=[dict(seed=1,delay_s=i/50,safe=s,safe_completion=s,event=dict(policy='no_recovery'))
              for i,s in enumerate((True,False,True))]
        result=metrics.predictor_metrics(run,metrics.command_truth(rows))['by_scenario'][0]
        self.assertEqual(result['true_first_unsafe_s'],.02)
        self.assertEqual(result['last_tested_safe_s'],.04);self.assertTrue(result['non_monotonic'])
        self.assertEqual(len(result['safety_intervals']),3)
        result=metrics.predictor_metrics(run,{})['by_scenario'][0]
        self.assertFalse(result['observations_complete']);self.assertFalse(result['truth_right_censored'])
        for row in rows:row.update(safe=True,safe_completion=True)
        result=metrics.predictor_metrics(run,metrics.command_truth(rows))['by_scenario'][0]
        self.assertTrue(result['truth_right_censored']);self.assertIsNone(result['ttl_error_s'])

    def test_freeze_rejects_protocol_and_source_changes(self):
        protocol=dict(scenarios=[dict(seed=1)],snapshot_sha256={'1':'example'},stage='evaluation_benchmark')
        frozen=dict(protocol=protocol,protocol_sha256=protocol_hash(protocol),provenance=storage.metadata())
        frozen['provenance_sha256']=protocol_hash(frozen['provenance'])
        with TemporaryDirectory() as temp:
            path=Path(temp)/'freeze.json';storage.write_json(path,frozen)
            verify_freeze(path,protocol)
            with self.assertRaises(ValueError):verify_freeze(path,dict(protocol,stage='different'))
            with patch('underpass_inspection.storage.metadata',return_value={}):
                with self.assertRaises(ValueError):verify_freeze(path,protocol)


if __name__=='__main__':unittest.main()
