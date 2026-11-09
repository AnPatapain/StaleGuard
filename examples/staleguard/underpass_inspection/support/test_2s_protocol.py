"""2-second scheduling/analysis tests; synthetic fixtures are never physics scores."""
import copy
import csv
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from underpass_inspection.baseline_benchmark import metrics
from underpass_inspection.baseline_benchmark.run import DISPATCH_DELAYS
from underpass_inspection.baselines import staleguard,fbs
from underpass_inspection.baselines.policies import PendingCommand,POLICIES
from underpass_inspection.scenario import Scenario
from underpass_inspection.storage import ROOT,physical_source_hash,REVIEWED,ADDITIVE_SOURCE_HASHES

SETTINGS=dict(unit='scenario',method='paired percentile bootstrap',confidence=.95,
              resamples=2000,seed=2027,complete_scenarios_only=True)
WINDOWS=dict(full=[0.,2.],within_prediction_horizon=[0.,1.],beyond_prediction_horizon=[1.02,2.])


class TwoSecondTests(unittest.TestCase):
    def test_grid_algorithms_and_physical_horizon_unchanged(self):
        self.assertEqual(DISPATCH_DELAYS,tuple(i/50 for i in range(101)))
        self.assertEqual(len(staleguard.TTL_GRID),19);self.assertEqual(max(staleguard.TTL_GRID),1.)
        original=json.loads((ROOT/'results/evaluation/preparation/freeze.json').read_text())
        for name,digest in original['provenance']['source_sha256'].items():
            if name=='simulation.py':
                self.assertIn(digest,ADDITIVE_SOURCE_HASHES[name])
                self.assertEqual(physical_source_hash((ROOT/name).read_text(),name),REVIEWED[name][1])
            elif name.startswith('baselines/') or name=='scenario.py':
                self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),digest,name)
        s=Scenario()
        self.assertEqual(s.observation_s,8.)
        self.assertAlmostEqual(2+s.swerve_start_s+s.swerve_s,5.8)
        self.assertAlmostEqual(s.observation_s-(2+s.swerve_start_s+s.swerve_s),2.2)
        self.assertLess(2+.02+s.swerve_start_s+s.swerve_s,s.observation_s)

    def test_beyond_prediction_grid_has_no_new_rejection_rule(self):
        for name in ('staleguard','staleguard_early_recovery'):
            for runtime,expiry in ((.06,None),(.06,20),(3.,20)):
                agent=PendingCommand(name,Scenario(),dict(computation_s=runtime,expiry_steps=expiry))
                agent.approve();actions=[]
                for age in range(151):
                    step=agent.approval+age;agent.announce_readiness(step)
                    if age==100:agent.dispatch(step,None,None)
                    agent.tick(step);action=agent.take_action()
                    if action is not None:actions.append((age,action))
                expected=(100,'dive') if expiry is None or runtime>2. else (
                    (20,'sideways') if name=='staleguard_early_recovery' else (100,'sideways'))
                self.assertEqual(actions,[expected])
                self.assertEqual(agent.event['dispatch_observed_step'],agent.approval+100)
        prediction=dict(margin_m=1.,ttl_steps=57)
        self.assertFalse(fbs.dispatch_status(prediction,0.,50)['recover'])
        self.assertFalse(fbs.dispatch_status(prediction,0.,57)['recover'])
        self.assertTrue(fbs.dispatch_status(prediction,0.,100)['recover'])

    def test_uncertainty_resamples_whole_paired_scenarios(self):
        results=[]
        for delays in (3,101):
            groups={}
            for seed,unsafe in ((1,delays),(2,0)):
                policy=dict(flights=delays,unsafe=unsafe,safe_completion=delays-unsafe,recovery_attempts=0)
                groups[str(seed)]=dict(by_policy={p:policy for p in ('no_recovery','staleguard')},
                    paired_against_no_recovery={'staleguard':dict(net_unsafe_reduction=0,net_completion_benefit=0)},
                    paired_early_recovery={})
            result=dict(by_scenario=groups,paired_against_no_recovery={'staleguard':{}},paired_early_recovery={})
            ci=metrics.scenario_uncertainty(result,dict(policies=['no_recovery','staleguard'],
                delays=list(range(delays)),uncertainty=SETTINGS))
            self.assertEqual(ci['complete_scenarios'],2)
            self.assertEqual(ci['by_policy']['staleguard']['unsafe']['ci95'],[0.,1.])
            self.assertEqual(ci['paired_against_no_recovery']['staleguard']['net_unsafe_reduction']['ci95'],[0.,0.])
            results.append(ci)
        self.assertEqual(results[0],results[1])

    def test_one_journal_three_analysis_windows_and_completion_labels(self):
        with (ROOT/'results/development/baseline_benchmark/executions.jsonl').open() as stream:
            template=json.loads(next(stream))
        rows=[]
        for seed in (1,3):
            for delay in (0.,1.,2.):
                for p in POLICIES:
                    row=copy.deepcopy(template);row.update(seed=seed,delay_s=delay,
                        physics_sha256='synthetic grouping fixture; not physical evidence',
                        inspection_observation='completed',maneuver_profile_truncated_by_horizon=False)
                    row['event'].update(policy=p,computation_s=.01,prediction_available_before_dispatch=delay>=.01,
                                        observation_update_s=[.001])
                    rows.append(row)
        run=dict(protocol=dict(scenarios=[dict(seed=1),dict(seed=3)],policies=list(POLICIES),
            delays=[0.,1.,2.],ground_truth_source='no_recovery',analysis_subsets=WINDOWS,
            uncertainty=SETTINGS),predictions={})
        with TemporaryDirectory() as folder,patch.object(metrics,'iter_committed',side_effect=lambda *a:iter(rows)):
            result=metrics.write_journal_outputs(Path(folder),run)
            self.assertEqual(result['recorded_flights'],42)
            self.assertEqual(result['analysis_subsets']['within_prediction_horizon']['recorded_flights'],28)
            self.assertEqual(result['analysis_subsets']['beyond_prediction_horizon']['recorded_flights'],14)
            self.assertEqual(result['uncertainty']['complete_scenarios'],2)
            with (Path(folder)/'per_delay.csv').open() as stream:data=list(csv.DictReader(stream))
            self.assertEqual(len(data),42)
            self.assertEqual(sum(r['prediction_grid_region']=='beyond_prediction_horizon' for r in data),14)
            with (Path(folder)/'per_scenario.csv').open() as stream:data=list(csv.DictReader(stream))
            self.assertEqual({r['analysis_subset'] for r in data},set(WINDOWS))
        row=copy.deepcopy(rows[0]);row.update(completed=False,safe_completion=False,
            inspection_observation='not_observed_by_fixed_deadline',maneuver_profile_truncated_by_horizon=True)
        m=metrics.policy_metrics([row],'no_recovery',{})
        self.assertEqual(m['inspections_not_observed_by_deadline'],1)
        self.assertEqual(m['maneuver_profiles_truncated_by_horizon'],1)


if __name__=='__main__':unittest.main()
