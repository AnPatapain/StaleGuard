"""Focused proof against outputs captured before moving any mission code.

These tests never recalibrate scenarios or fly a new benchmark. Recorded physics
are compared at 1 kHz; generation, references, selection and forecasts are also
checked against the live functions. Original source is archived as .py.txt so it
cannot be imported or collected as another test suite.
"""
import ast
import contextlib
import io
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import tempfile
import unittest

import numpy as np
from geofence_turn.scenario import (Config, HZ, generate_scenario, scenario_dict,
    scenario_from_dict, make_forward_trajectory, apply_turn_command)
from geofence_turn.calibration import approval_candidates, calibrate_scenarios, verify_calibration
from geofence_turn.storage import load_development, check_provenance
from geofence_turn.baselines.staleguard import predict_delays
from geofence_turn.simulation import check_memory, start_work

ROOT = Path(__file__).resolve().parents[2]
RECORDS = ROOT/'support/results/refactor_equivalence'
BEFORE = RECORDS/'reference'
AFTER = RECORDS/'refactored'
DEV = ROOT/'support/logs/northbound_review/development'


def read(path):
    return json.loads(path.read_text())


def scientific_result(result):
    """Wall clocks may vary; decisions, checks and applied steps may not."""
    result = json.loads(json.dumps(result))
    result.pop('runtime_s', None)
    result['event'].pop('computation_s', None)
    return result


def function_tree(path, name):
    return next(node for node in ast.parse(path.read_text()).body
                if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name == name)


class RefactorEquivalenceTests(unittest.TestCase):
    def test_reference_is_complete_and_configuration_unchanged(self):
        before, after = read(BEFORE/'complete.json'), read(AFTER/'complete.json')
        self.assertEqual(before['flight_count'], 21)
        self.assertEqual(after['flight_count'], 21)
        self.assertEqual(before['config'], asdict(Config()))
        self.assertEqual(before['config'], after['config'])

    def test_generated_scenarios_and_rejections_are_identical(self):
        envelope = read(ROOT/'support/logs/northbound_review/characterization/envelope.json')
        actual = []
        for seed in range(10):
            try:
                actual.append(scenario_dict(generate_scenario(seed, envelope)))
            except ValueError as error:
                actual.append(dict(seed=seed, rejection=str(error)))
        self.assertEqual(json.loads(json.dumps(actual)), read(BEFORE/'generated.json'))
        self.assertEqual(read(BEFORE/'generated.json'), read(AFTER/'generated.json'))

    def test_same_approval_times_from_all_33_candidate_records(self):
        for record in read(BEFORE/'calibration.json'):
            seed = record['seed']
            rows = record['candidates']
            attempt = next(row for row in map(json.loads, (DEV/'attempts.jsonl').read_text().splitlines())
                           if row['seed'] == seed)
            by_step = {round(row['approval_s']*HZ):row for row in rows}
            selected = next(candidate for candidate in approval_candidates(
                scenario_from_dict(attempt['generated']),attempt['search_upper_s'])
                if by_step[round(candidate.approval_s*HZ)]['outcome']['safe_completion'])
            self.assertEqual(json.loads(json.dumps(scenario_dict(selected))), record['selected'])
        self.assertEqual(read(BEFORE/'calibration.json'), read(AFTER/'calibration.json'))

    def test_all_60_nominal_and_dispatched_reference_arrays_are_identical(self):
        for path in sorted(DEV.glob('seed_*/scenario.json')):
            scenario = scenario_from_dict(read(path))
            expected = np.load(BEFORE/f'references_{scenario.seed}.npz')
            reference = make_forward_trajectory(len(expected['nominal']), scenario)
            np.testing.assert_array_equal(reference, expected['nominal'])
            for i in range(11):
                actual = apply_turn_command(reference, round(scenario.approval_s*HZ)+5*i, scenario.command)
                np.testing.assert_array_equal(actual, expected[f'delay_{i}'])

    def test_all_55_live_prediction_results_match_the_original_model(self):
        from model.scg import load_scg_platform_params
        check_memory(Config())
        platform = load_scg_platform_params()
        for scenario, snapshot, _ in load_development(DEV):
            actual = predict_delays(snapshot, platform)
            expected = read(BEFORE/f'prediction_{scenario.seed}.json')
            self.assertEqual(actual['results'], expected['results'])
            self.assertEqual(actual['expiry_steps'], expected['expiry_steps'])
            recorded = read(AFTER/f'prediction_{scenario.seed}.json')
            self.assertEqual(actual['results'], recorded['results'])

    def test_21_actual_flights_and_policy_steps_match(self):
        for path in sorted(BEFORE.glob('*.npz')):
            if path.name.startswith('references_'):
                continue
            before, after = np.load(path), np.load(AFTER/path.name)
            self.assertEqual(before.files, after.files)
            for field in before.files:
                np.testing.assert_allclose(before[field], after[field], rtol=0, atol=1e-10,
                                           err_msg=f'{path.name}:{field}')
            old = scientific_result(read(path.with_suffix('.json')))
            new = scientific_result(read((AFTER/path.name).with_suffix('.json')))
            self.assertEqual(old, new, path.name)

    def test_old_snapshots_and_exported_dataset_load_without_conversion(self):
        original = load_development(DEV)
        exported = load_development(RECORDS/'calibrated_export/calibrated_scenarios.json')
        self.assertEqual(len(original), 5)
        for (s, snapshot, path), (other, loaded, target) in zip(original, exported):
            self.assertEqual(s, other)
            self.assertEqual(path.resolve(), target.resolve())
            np.testing.assert_array_equal(snapshot.state, loaded.state)
            np.testing.assert_array_equal(snapshot.remaining_reference, loaded.remaining_reference)
        with self.assertRaisesRegex(ValueError, 'configuration'):
            load_development(DEV, replace(Config(), d_min_m=.051))


    def test_checksum_and_old_source_resume_checks_are_not_disabled(self):
        exported = RECORDS/'calibrated_export/calibrated_scenarios.json'
        data = read(exported)
        data['provenance']['path'] = str((exported.parent/data['provenance']['path']).resolve())
        for row in data['scenarios']:
            row['approval_snapshot'] = str((exported.parent/row['approval_snapshot']).resolve())
        data['scenarios'][0]['snapshot_sha256'] = '0'*64
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder)/'tampered.json'
            target.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, 'snapshot checksum'):
                load_development(target)
        with self.assertRaisesRegex(ValueError, 'source/layout changed'):
            check_provenance(read(DEV/'metadata.json'),Config())
        with self.assertRaises(ValueError):
            start_work(201, 60)

    def test_grouped_archives_keep_every_file_byte_unchanged(self):
        manifest = read(ROOT/'support/layout_reorganization.json')
        self.assertEqual(len(manifest['files']), 486)
        for record in manifest['files']:
            self.assertFalse((ROOT/record['old']).exists(), record['old'])
            self.assertEqual(hashlib.sha256((ROOT/record['new']).read_bytes()).hexdigest(),
                             record['sha256'], record['new'])
        for name in ('diagnostics.py','validation.py','mission.py','tests','logs'):
            self.assertFalse((ROOT/name).exists(), name)

    def test_original_data_and_shared_model_files_are_untouched(self):
        records = read(BEFORE/'preservation.json')
        for group in ('data_sha256', 'shared_model_sha256'):
            for name, checksum in records[group].items():
                moved_name = name.replace('geofence_turn/logs/','geofence_turn/support/logs/',1)
                path = ROOT.parent/moved_name
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), checksum, name)

    def test_moved_scientific_functions_and_physics_loop_are_unchanged(self):
        archived = BEFORE/'source'
        moves = [
            ('mission.py.txt', 'scenario.py', name) for name in
            ('Config','Command','Scenario','scenario_dict','scenario_from_dict','make_forward_trajectory',
             'turn_sample','apply_turn_command','clearance','in_checkpoint','safety_only_reference')]
        moves += [('scenario.py.txt','scenario.py','generate_scenario')]
        moves += [('prediction.py.txt','baselines/staleguard.py','predict_delays'),
                  ('prediction.py.txt','baselines/use_time_recheck.py','predict_immediate'),
                  ('policies.py.txt','baselines/staleguard.py','staleguard_status'),
                  ('policies.py.txt','baselines/fixed_ttl.py','fixed_ttl_expired'),
                  ('policies.py.txt','baselines/use_time_recheck.py','first_step_after_check'),
                  ('simulation.py.txt','simulation.py','env_factory')]
        for old, new, name in moves:
            self.assertEqual(ast.dump(function_tree(archived/old,name)),
                             ast.dump(function_tree(ROOT/new,name)), name)
        class RemoveImports(ast.NodeTransformer):
            def visit_ImportFrom(self,node):
                return None
        old = function_tree(archived/'simulation.py.txt','execute')
        new = function_tree(ROOT/'simulation.py','execute')
        for node_type, name in ((ast.For,'step'), (ast.FunctionDef,'observe')):
            def select(function):
                return next(node for node in ast.walk(function) if isinstance(node,node_type)
                            and (getattr(node,'name',None)==name or
                                 isinstance(getattr(node,'target',None),ast.Name) and node.target.id==name))
            self.assertEqual(ast.dump(RemoveImports().visit(select(old))),
                             ast.dump(RemoveImports().visit(select(new))), name)


class ArtifactWorkflowTests(unittest.TestCase):
    """Output/resume fixtures use saved results, never new physics flights."""
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)





    def recorded_flight(self, candidate, delay=0., **kwargs):
        if delay is None:
            return read(DEV/f'seed_{candidate.seed}/nominal_bound.json'),None,None
        cases = {s.seed:(s,snapshot,path) for s,snapshot,path in load_development(DEV)}
        rows = list(map(json.loads,(DEV/f'seed_{candidate.seed}/candidates.jsonl').read_text().splitlines()))
        result = next(row['outcome'] for row in rows if row['approval_s']==candidate.approval_s)
        return result,cases[candidate.seed][1],None

    def test_simple_calibration_matches_saved_candidates_and_minimal_files(self):
        out = self.folder/'simple'
        with patch('geofence_turn.calibration.execute',side_effect=self.recorded_flight) as flown, contextlib.redirect_stdout(io.StringIO()):
            data = calibrate_scenarios(range(10),5,out)
        self.assertEqual(flown.call_count,43)
        self.assertEqual([s.approval_s for s,_,_ in load_development(out)],[2.,1.8,1.8,1.4,1.4])
        self.assertEqual(data['statistics']['attempted'],5)
        self.assertEqual(data['statistics']['generated'],10)
        self.assertEqual({p.name for p in out.iterdir()},{'snapshots','calibrated_scenarios.json'})
        self.assertNotIn('progress',data)
        before = (out/'calibrated_scenarios.json').read_bytes()
        with patch('geofence_turn.calibration.execute') as flown:
            calibrate_scenarios(range(10),5,out,resume=True)
        flown.assert_not_called()
        self.assertEqual((out/'calibrated_scenarios.json').read_bytes(),before)

    def test_resume_keeps_completed_seed_and_restarts_only_unfinished_seed(self):
        out = self.folder/'resume'
        def interrupted(candidate,delay=0.,**kwargs):
            if candidate.seed==1 and delay is not None:
                raise RuntimeError('fixture interruption')
            return self.recorded_flight(candidate,delay,**kwargs)
        with patch('geofence_turn.calibration.execute',side_effect=interrupted),contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'fixture interruption'):
                calibrate_scenarios([0,1],2,out)
        saved = read(out/'calibrated_scenarios.json')
        self.assertEqual(saved['statistics']['accepted'],1)
        self.assertEqual(saved['statistics']['attempted'],1)
        snapshot = (out/'snapshots/seed_0.pkl').read_bytes()
        with patch('geofence_turn.calibration.execute',side_effect=self.recorded_flight) as flown,contextlib.redirect_stdout(io.StringIO()):
            calibrate_scenarios([0,1],2,out,resume=True)
        self.assertTrue(all(call.args[0].seed==1 for call in flown.call_args_list))
        self.assertIsNone(flown.call_args_list[0].kwargs['delay'])
        self.assertEqual((out/'snapshots/seed_0.pkl').read_bytes(),snapshot)
        self.assertEqual(read(out/'calibrated_scenarios.json')['statistics']['accepted'],2)
        with self.assertRaisesRegex(ValueError,'inputs/settings'):
            calibrate_scenarios([1,0],2,out,resume=True)
        with self.assertRaisesRegex(FileExistsError,'Output contains data'):
            calibrate_scenarios([0,1],2,out)

    def test_exhausted_pool_and_generator_rejection_counts(self):
        out = self.folder/'rejected'
        def failure(candidate,delay=0.,**kwargs):
            result,snapshot,trace = self.recorded_flight(candidate,delay,**kwargs) if delay is None else ({},None,None)
            if delay is not None:
                result = dict(safe=True,completed=False,safe_completion=False)
            return result,snapshot,trace
        with patch('geofence_turn.calibration.execute',side_effect=failure),contextlib.redirect_stdout(io.StringIO()):
            data = calibrate_scenarios([0,5],1,out)
        self.assertEqual(data['calibration']['status'],'exhausted')
        self.assertEqual(data['statistics']['attempted'],2)
        self.assertEqual(data['statistics']['rejected'],2)
        self.assertEqual(data['calibration']['attempts'][1]['candidate_count'],0)
        self.assertTrue(data['calibration']['attempts'][1]['rejection_reason'].startswith('checkpoint_geometry:'))
        self.assertEqual(load_development(out),[])

    def test_verification_checks_reference_and_command_as_well_as_state(self):
        from copy import deepcopy
        scenario,snapshot,_ = load_development(DEV)[0]
        bad_reference = replace(snapshot,remaining_reference=snapshot.remaining_reference+.01)
        bad_command = replace(snapshot,command=replace(snapshot.command,radius=.4))
        for changed in (bad_reference,bad_command):
            with patch('geofence_turn.calibration.execute',return_value=(dict(safe_completion=True),changed,None)):
                with self.assertRaises(AssertionError):
                    verify_calibration(scenario,snapshot)

    def test_all_frozen_datasets_load_and_reject_tampered_snapshot(self):
        source = ROOT/'results/development/calibration/calibrated_scenarios.json'
        self.assertEqual(len(load_development(source)),20)
        self.assertEqual(len(load_development(ROOT/'results/evaluation/calibration')),300)
        data = read(source)
        for row in data['scenarios']:
            row['approval_snapshot'] = str((source.parent/row['approval_snapshot']).resolve())
        data['scenarios'][0]['snapshot_sha256'] = '0'*64
        path = self.folder/'tampered.json'
        path.write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError,'snapshot checksum'):
            load_development(path)

    def test_real_record_counts_match_previous_summary_and_partial_pairs(self):
        from geofence_turn.baseline_benchmark.metrics import summarize
        rows = [read(p) for p in sorted(BEFORE.glob('baseline_*.json'))]
        journal = self.folder/'executions.jsonl'
        journal.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        report = summarize(journal)
        expected = read(RECORDS/'comparison.json')['baseline_counts']['reference']
        for policy,counts in expected.items():
            actual = report['by_policy'][policy]
            for name in ('flights','safe_completion','safe_noncompletion','unsafe'):
                self.assertEqual(actual[name],counts[name])
            self.assertEqual(actual['recovery_attempts'],counts['fallback'])
            self.assertEqual(actual['recovery_successes']+actual['recovery_failures'],counts['fallback'])
        self.assertEqual(report['paired_comparison']['complete_pairs'],4)
        self.assertEqual(report['paired_comparison']['net_completion_benefit'],0)
        self.assertEqual(report['by_policy']['staleguard']['prediction_computation_s']['samples'],2)
        self.assertEqual(report['by_policy']['use_time_recheck']['prediction_computation_s']['samples'],4)
        self.assertEqual(report['by_policy']['no_recovery']['prediction_computation_s']['mean_s'],None)
        removed = next(r for r in rows if r['event']['policy']=='use_time_recheck')
        rows.remove(removed)
        journal.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        partial = summarize(journal)
        self.assertEqual(partial['paired_comparison']['complete_pairs'],3)
        self.assertEqual(partial['paired_comparison']['incomplete_pairs'],1)
        self.assertEqual(sum(x['flights'] for x in partial['by_policy'].values()),15)

    def test_metric_formulas_missing_timings_and_recovery_failures(self):
        from geofence_turn.baseline_benchmark.metrics import summarize
        def row(seed,policy,safe,completed,recovery=False,duration=None,ready=None,delay=.1,action=55):
            dispatch = 50+round(delay*50)
            return dict(seed=seed,delay_s=delay,safe=safe,completed=completed,safe_completion=safe and completed,
                        event=dict(policy=policy,scheduled_dispatch_step=dispatch,
                        application_step=None if recovery else action,fallback_step=action if recovery else None,
                        computation_s=duration,prediction_ready=ready,decision_step=200))
        rows = [row(100,'staleguard',True,True,duration=.04,ready=True),
                row(100,'use_time_recheck',False,False,recovery=True,duration=.005,action=57),
                row(101,'staleguard',False,False,recovery=True,duration=.02,ready=True),
                row(101,'use_time_recheck',True,True,action=56),
                row(102,'staleguard',True,True,ready=False),
                row(102,'use_time_recheck',True,False,recovery=True,duration=.002,action=56),
                row(103,'staleguard',True,False),row(103,'use_time_recheck',False,False,action=None),
                row(100,'staleguard',True,True,duration=.04,ready=True,delay=.2,action=60)]
        journal = self.folder/'executions.jsonl'
        journal.write_text(''.join(json.dumps(r)+'\n' for r in rows))
        report = summarize(journal)
        sg,ut = report['by_policy']['staleguard'],report['by_policy']['use_time_recheck']
        self.assertEqual((sg['flights'],sg['safe_completion'],sg['safe_noncompletion'],sg['unsafe']),(5,3,1,1))
        self.assertEqual(sg['safe_completion_rate'],.6)
        self.assertEqual((sg['prediction_ready'],sg['prediction_unavailable'],sg['prediction_readiness_unrecorded']),(3,1,1))
        self.assertEqual(sg['prediction_computation_s'],dict(samples=2,mean_s=.03,median_s=.03,p95_s=.04))
        self.assertEqual(sg['prediction_computation_ms'],dict(samples=2,mean_ms=30.,median_ms=30.,p95_ms=40.))
        self.assertEqual(sg['unverified_executions'],1)
        self.assertEqual((ut['recovery_attempts'],ut['recovery_successes'],ut['recovery_failures']),(2,1,1))
        self.assertEqual(ut['dispatch_to_action_s']['samples'],3)
        self.assertEqual(ut['dispatch_to_action_s']['p95_s'],.04)
        self.assertEqual(ut['prediction_computation_s']['samples'],2)
        pair = report['paired_comparison']
        self.assertEqual((pair['complete_pairs'],pair['incomplete_pairs']),(4,1))
        self.assertEqual((pair['completion_benefits'],pair['completion_harms'],pair['net_completion_benefit']),(2,1,1))
        self.assertEqual((pair['unsafe_prevented'],pair['unsafe_introduced'],pair['net_unsafe_reduction']),(2,1,1))
        for group in report['by_policy'].values():
            for key,value in group.items():
                if key.endswith('_rate') and value is not None:
                    self.assertTrue(0 <= value <= 1)
        journal.write_text('')
        empty = summarize(journal,dict(policies=['staleguard'],delays=[0.],control_hz=50))
        self.assertIsNone(empty['by_policy']['staleguard']['unsafe_rate'])
        self.assertIsNone(empty['paired_comparison']['completion_benefits'])

    def benchmark_fixture(self,out,args,execute_fixture,prediction=None):
        from geofence_turn.baseline_benchmark.run import run_comparison
        received = prediction or read(RECORDS/'baseline_entrypoint/prediction_seed_0.json')
        with patch('model.scg.load_scg_platform_params',return_value=(None,None)), \
             patch('geofence_turn.baselines.staleguard.predict_delays',return_value=received), \
             patch('geofence_turn.baselines.use_time_recheck.predict_immediate',return_value=received['results'][0]), \
             patch('geofence_turn.baseline_benchmark.run.execute',side_effect=execute_fixture) as flown, \
             patch('geofence_turn.baseline_benchmark.run.check_memory',return_value=(100,3000)), \
             contextlib.redirect_stdout(io.StringIO()):
            run_comparison(out,args,start_work(20,120),Config())
        return flown

    def test_two_policy_selection_keeps_only_selected_pairs(self):
        _,snapshot,_ = load_development(DEV)[0]
        originals = {row['event']['policy']:row for row in map(json.loads,(RECORDS/'baseline_entrypoint/executions.jsonl').read_text().splitlines())}
        args = SimpleNamespace(development=str(DEV),seeds=[0],delays=(0.,),
                               policies=('staleguard','use_time_recheck'),resume=False,diagnostics=False)
        def recorded(s,delay,policy,**kwargs):
            return originals[policy],snapshot,np.empty((0,))
        out = self.folder/'paired'
        flown = self.benchmark_fixture(out,args,recorded)
        self.assertEqual([call.kwargs['policy'] for call in flown.call_args_list],list(args.policies))
        self.assertEqual(read(out/'run.json')['protocol']['policies'],list(args.policies))
        self.assertEqual(read(out/'metrics.json')['paired_comparison']['complete_pairs'],1)
        self.assertEqual(read(out/'metrics.json')['recorded_flights'],2)

    def test_benchmark_three_files_resume_cache_and_duplicate_prevention(self):
        from geofence_turn.baseline_benchmark.metrics import read_executions, summarize
        scenario,snapshot,_ = load_development(DEV)[0]
        originals = {row['event']['policy']:row for row in map(json.loads,(RECORDS/'baseline_entrypoint/executions.jsonl').read_text().splitlines())}
        args = SimpleNamespace(development=str(DEV),seeds=[0],delays=(0.,),resume=False,diagnostics=False)
        def recorded(s,delay,policy,**kwargs):
            self.assertFalse(kwargs['save_trace'])
            self.assertEqual(kwargs['received']['computation_s'],originals['staleguard']['event']['computation_s'])
            return originals[policy],snapshot,np.empty((0,))
        def interrupted(s,delay,policy,**kwargs):
            if policy=='use_time_recheck':
                raise RuntimeError('fixture interruption')
            return recorded(s,delay,policy,**kwargs)
        out = self.folder/'benchmark'
        with self.assertRaisesRegex(RuntimeError,'fixture interruption'):
            self.benchmark_fixture(out,args,interrupted)
        self.assertEqual({p.name for p in out.iterdir()},{'run.json','executions.jsonl','metrics.json'})
        self.assertEqual(len(read_executions(out/'executions.jsonl')),2)
        self.assertEqual(read(out/'run.json')['status'],'interrupted')
        before = (out/'executions.jsonl').read_bytes()
        args.resume = True
        prediction = read(RECORDS/'baseline_entrypoint/prediction_seed_0.json')
        prediction['computation_s'] = 99.  # Warm-up must not replace cached timing.
        self.benchmark_fixture(out,args,recorded,prediction)
        self.assertTrue((out/'executions.jsonl').read_bytes().startswith(before))
        self.assertEqual(len(read_executions(out/'executions.jsonl')),4)
        self.assertEqual(read(out/'run.json')['status'],'complete')
        self.assertEqual(read(out/'metrics.json'),summarize(out/'executions.jsonl'))
        self.assertEqual(read(out/'run.json')['predictions']['0']['computation_s'],originals['staleguard']['event']['computation_s'])
        before = (out/'executions.jsonl').read_bytes()
        flown = self.benchmark_fixture(out,args,recorded,prediction)
        flown.assert_not_called()
        self.assertEqual((out/'executions.jsonl').read_bytes(),before)
        self.assertEqual({p.name for p in out.iterdir()},{'run.json','executions.jsonl','metrics.json'})
        for row in read_executions(out/'executions.jsonl'):
            self.assertEqual(row,originals[row['event']['policy']])
        saved_run = read(out/'run.json')
        tampered = read(out/'run.json')
        tampered['provenance']['source_hashes']['simulation.py'] = 'changed'
        (out/'run.json').write_text(json.dumps(tampered))
        with self.assertRaisesRegex(ValueError,'source/layout changed'):
            self.benchmark_fixture(out,args,recorded)
        (out/'run.json').write_text(json.dumps(saved_run))
        tampered = read(out/'run.json')
        tampered['predictions']['0']['computation_s'] = 99.
        (out/'run.json').write_text(json.dumps(tampered))
        with self.assertRaisesRegex(ValueError,'frozen approval prediction'):
            self.benchmark_fixture(out,args,recorded)
        (out/'run.json').write_text(json.dumps(saved_run))
        args.delays = (.2,)
        with self.assertRaisesRegex(ValueError,'protocol differ'):
            self.benchmark_fixture(out,args,recorded)
        args.delays = (0.,)
        with (out/'executions.jsonl').open('ab') as stream:
            stream.write(b'{"seed":')  # Unfinished final write, not a flight record.
        self.benchmark_fixture(out,args,recorded)
        self.assertEqual((out/'executions.jsonl').read_bytes(),before)
        with (out/'executions.jsonl').open('a') as stream:
            stream.write(json.dumps(originals['no_recovery'])+'\n')
        with self.assertRaisesRegex(ValueError,'Duplicate flight'):
            self.benchmark_fixture(out,args,recorded)
        (out/'executions.jsonl').write_bytes(before.replace(b'"seed": 0',b'"seed": 9',1))
        with self.assertRaisesRegex(ValueError,'committed execution records'):
            self.benchmark_fixture(out,args,recorded)


class TTLAccuracyTests(unittest.TestCase):
    """Reuse recorded forecasts/flights; these checks launch no physical flights."""
    @classmethod
    def setUpClass(cls):
        from geofence_turn.ttl_pred_accuracy.run import validate_ground_truth
        cls.cases = load_development(ROOT/'results/development/calibration')
        cls.actual,cls.evidence = validate_ground_truth(cls.cases,Config())
        cls.forecasts = read(ROOT/'results/development/baseline_benchmark/run.json')['predictions']

    def fixture(self, folder):
        from geofence_turn.ttl_pred_accuracy.run import comparison, SEEDS, DELAYS
        out = Path(folder)
        journal = out/'executions.jsonl'
        rows = [comparison(s,self.actual[s.seed,delay],self.forecasts[str(s.seed)])
                for s,_,_ in self.cases for delay in DELAYS]
        journal.write_text(''.join(json.dumps(row)+'\n' for row in rows))
        protocol = dict(seeds=list(SEEDS),delays_s=list(DELAYS),
                        prediction_grid_s=[i/10 for i in range(11)],control_hz=HZ)
        return journal,protocol,rows

    def test_ground_truth_matches_all_frozen_inputs_and_waiting_safety(self):
        self.assertEqual(len(self.cases),20)
        self.assertEqual(len(self.actual),380)
        self.assertEqual(self.evidence['status'],'passed')
        self.assertEqual(self.evidence['new_pybullet_flights'],0)

    def test_raw_and_ttl_are_separate_and_timing_counts_forecasts_once(self):
        from geofence_turn.ttl_pred_accuracy.metrics import summarize
        with tempfile.TemporaryDirectory() as folder:
            journal,protocol,rows = self.fixture(folder)
            result = summarize(journal,protocol)
        self.assertEqual(result['raw_prediction']['total_tests'],220)
        self.assertEqual(result['raw_prediction']['false_safe'],0)
        self.assertEqual(result['ttl_dispatch_decision']['total_tests'],380)
        self.assertEqual(result['ttl_dispatch_decision']['false_safe'],15)
        self.assertEqual(result['warmed_prediction_computation']['samples'],20)
        for matrix in [result['raw_prediction'],result['ttl_dispatch_decision']]:
            self.assertEqual(sum(matrix[key] for key in ('correct_safe','correct_unsafe','false_safe','false_rejection')),matrix['total_tests'])
        for row in rows:
            if row['first_violation_s'] is not None:
                self.assertEqual(row['first_violation_s'],row['first_violation_flight_s']-row['approval_s'])

    def test_expiry_equality_rejects_even_when_recorded_prediction_is_slow(self):
        from geofence_turn.ttl_pred_accuracy.run import comparison
        scenario = self.cases[0][0]
        forecast = dict(self.forecasts['0'],computation_s=999.)
        expiry = forecast['expiry_steps']/HZ
        at_expiry = comparison(scenario,self.actual[0,expiry],forecast)
        self.assertFalse(at_expiry['predicted_safe_ttl'])
        before = comparison(scenario,self.actual[0,expiry-.1],forecast)
        self.assertTrue(before['predicted_safe_ttl'])
        self.assertEqual(at_expiry['prediction_computation_s'],999.)

    def test_off_grid_predictions_duplicates_and_undefined_rates(self):
        from geofence_turn.ttl_pred_accuracy.metrics import summarize, read_records, confusion
        with tempfile.TemporaryDirectory() as folder:
            journal,protocol,rows = self.fixture(folder)
            rows[1]['predicted_safe_raw'] = True
            journal.write_text(''.join(json.dumps(row)+'\n' for row in rows))
            with self.assertRaisesRegex(ValueError,'Raw prediction'):
                summarize(journal,protocol)
            journal.write_text(json.dumps(rows[0])+'\n'+json.dumps(rows[0])+'\n')
            with self.assertRaisesRegex(ValueError,'Duplicate comparison'):
                read_records(journal)
            journal.write_text(json.dumps(rows[0])+'\n'+ '{"seed":')
            self.assertEqual(read_records(journal,repair_tail=True),rows[:1])
        empty = confusion([],'predicted_safe_raw')
        self.assertIsNone(empty['accuracy'])
        self.assertIsNone(empty['false_safe_rate'])
        self.assertIsNone(empty['false_rejection_rate'])

    def test_confusion_rates_use_actual_class_denominators(self):
        from geofence_turn.ttl_pred_accuracy.metrics import confusion
        rows = [dict(safe=actual,predicted_safe_raw=predicted) for actual,predicted in
                [(True,True),(True,False),(False,True),(False,False),(True,None)]]
        result = confusion(rows,'predicted_safe_raw')
        self.assertEqual(result['total_tests'],4)
        for field in ('correct_safe','correct_unsafe','false_safe','false_rejection'):
            self.assertEqual(result[field],1)
        for field in ('accuracy','false_safe_rate','false_rejection_rate'):
            self.assertEqual(result[field],.5)

    def test_interrupted_resume_reuses_forecasts_and_prevents_changed_data(self):
        from geofence_turn.ttl_pred_accuracy.run import run_accuracy
        from unittest.mock import MagicMock
        warm = MagicMock()
        warm.start.return_value = (SimpleNamespace(select_action=lambda *_:None),None,None,None)
        def recorded_prediction(snapshot, platform):
            seed = next(s.seed for s,snap,_ in self.cases if np.array_equal(snap.state,snapshot.state))
            return self.forecasts[str(seed)]
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'accuracy'
            args = SimpleNamespace(development=str(ROOT/'results/development/calibration'),out=str(out),resume=False)
            with patch('model.scg.load_scg_platform_params',return_value=(None,None)), \
                    patch('geofence_turn.ttl_pred_accuracy.run.LightweightPredictiveModel',return_value=warm), \
                    patch('geofence_turn.simulation.check_memory',return_value=(100,3000)), \
                    patch('geofence_turn.ttl_pred_accuracy.run.check_memory',return_value=(100,3000)), \
                    patch('geofence_turn.ttl_pred_accuracy.run.predict_delays',side_effect=recorded_prediction) as predicted, \
                    contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(RuntimeError,'trial/time budget'):
                    run_accuracy(args,start_work(2,120),Config())
                self.assertEqual(predicted.call_count,2)
                self.assertEqual(read(out/'run.json')['status'],'interrupted')
                args.resume = True
                run_accuracy(args,start_work(40,120),Config())
                self.assertEqual(predicted.call_count,20)
                before = (out/'executions.jsonl').read_bytes()
                with (out/'executions.jsonl').open('ab') as stream:
                    stream.write(b'{"seed":')
                run_accuracy(args,start_work(40,120),Config())
                self.assertEqual(predicted.call_count,20)
                self.assertEqual((out/'executions.jsonl').read_bytes(),before)
                self.assertEqual(set(p.name for p in out.iterdir()),{'run.json','metrics.json','executions.jsonl'})
                self.assertEqual(read(out/'run.json')['status'],'complete')
                self.assertEqual(len(before.splitlines()),380)
                self.assertEqual(read(out/'metrics.json')['raw_prediction']['total_tests'],220)
                changed = before.replace(b'"seed": 0',b'"seed": 5',1)
                (out/'executions.jsonl').write_bytes(changed)
                with self.assertRaisesRegex(ValueError,'committed comparisons'):
                    run_accuracy(args,start_work(40,120),Config())


if __name__ == '__main__':
    unittest.main()
