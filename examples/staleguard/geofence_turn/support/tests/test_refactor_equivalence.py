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
from geofence_turn.calibration import (calibrate, calibrate_development, load_development, prepare_output)
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
            def evaluate(candidate):
                return by_step[round(candidate.approval_s*HZ)]['outcome'], None, None
            selected, _, _ = calibrate(scenario_from_dict(attempt['generated']), evaluate,
                                        latest_approval_s=attempt['search_upper_s'])
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

    def test_calibration_script_preserves_selection_using_recorded_candidates(self):
        # Exercise the moved script and JSON writer without recalibrating flights.
        cases = {s.seed:(s,snapshot,path) for s,snapshot,path in load_development(DEV)}
        records = {seed:[read_line for read_line in map(json.loads,
                   (DEV/f'seed_{seed}/candidates.jsonl').read_text().splitlines())]
                   for seed in cases}
        def recorded_flight(candidate, delay=0., **kwargs):
            seed = candidate.seed
            if delay is None:
                return read(DEV/f'seed_{seed}/nominal_bound.json'), None, None
            selected, snapshot, _ = cases[seed]
            result = next(row['outcome'] for row in records[seed]
                          if row['approval_s']==candidate.approval_s)
            trace = np.load(DEV/f'seed_{seed}/d0_trace.npz')['controls']
            return result, snapshot, trace
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'calibration'
            args = SimpleNamespace(resume=False, generated=str(DEV/'generated.json'), attempts=10,target=5)
            work = start_work(100,120)
            with patch('geofence_turn.calibration.execute',side_effect=recorded_flight) as flown, patch('geofence_turn.calibration.check_memory',return_value=(100,3000)), contextlib.redirect_stdout(io.StringIO()):
                calibrate_development(out,args,work,Config())
            self.assertEqual(flown.call_count,43)
            exported = load_development(out/'calibrated_scenarios.json')
            self.assertEqual([s.approval_s for s,_,_ in exported],[2.,1.8,1.8,1.4,1.4])
            self.assertEqual([s.command for s,_,_ in exported],[cases[i][0].command for i in sorted(cases)])
            data = read(out/'calibrated_scenarios.json')
            self.assertEqual(data['statistics']['attempted'],5)
            self.assertEqual(data['statistics']['generated'],10)
            self.assertEqual(data['statistics']['accepted'],5)
            self.assertEqual({p.name for p in out.iterdir()},{'calibrated_scenarios.json','snapshots'})
            self.assertEqual(len(list((out/'snapshots').glob('*.pkl'))),5)
            attempts = data['calibration']['attempts']
            args.resume = True
            with patch('geofence_turn.calibration.execute') as flown, contextlib.redirect_stdout(io.StringIO()):
                calibrate_development(out,args,work,Config())
            flown.assert_not_called()
            self.assertEqual(read(out/'calibrated_scenarios.json')['calibration']['attempts'],attempts)

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
            prepare_output(DEV, Config(), resume=True)
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
        moves += [('scenario.py.txt','calibration.py',name) for name in
                  ('approval_candidates','rejection_reasons','calibrate')]
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

    def test_export_all_twenty_without_recalibration_or_snapshot_conversion(self):
        from geofence_turn.calibration import export_calibrated
        source = ROOT/'support/results/delay_tolerance_20/calibrated_scenarios.json'
        original = load_development(source)
        target = self.folder/'calibration'
        target.mkdir()
        with patch('geofence_turn.calibration.execute') as flown:
            export_calibrated(target,source,Config())
        flown.assert_not_called()
        loaded = load_development(target)
        self.assertEqual(len(loaded),20)
        self.assertEqual({p.name for p in target.iterdir()},{'snapshots','calibrated_scenarios.json'})
        data = read(target/'calibrated_scenarios.json')
        self.assertEqual(data['statistics']['attempted'],22)
        self.assertEqual(data['statistics']['rejected'],2)
        self.assertEqual(data['statistics']['acceptance_rate'],20/22)
        for (s,before,path),(other,after,copy) in zip(original,loaded):
            self.assertEqual(s,other)
            self.assertEqual(path.read_bytes(),copy.read_bytes())
            np.testing.assert_array_equal(before.state,after.state)
            np.testing.assert_array_equal(before.remaining_reference,after.remaining_reference)
        data['scenarios'][0]['snapshot_sha256'] = '0'*64
        (target/'calibrated_scenarios.json').write_text(json.dumps(data))
        with self.assertRaisesRegex(ValueError,'snapshot checksum'):
            load_development(target)

    def calibration_fixture(self):
        scenario,snapshot,_ = load_development(DEV)[0]
        source = next(row['generated'] for row in map(json.loads,(DEV/'attempts.jsonl').read_text().splitlines()) if row['seed']==0)
        generated = self.folder/'input.json'
        generated.write_text(json.dumps([source]))
        args = SimpleNamespace(resume=False,generated=str(generated),attempts=1,target=1,diagnostics=False)
        candidates = {row['approval_s']:row['outcome'] for row in map(json.loads,(DEV/'seed_0/candidates.jsonl').read_text().splitlines())}
        def recorded(candidate,delay=0.,**kwargs):
            if delay is None:
                return read(DEV/'seed_0/nominal_bound.json'),None,None
            return candidates[candidate.approval_s],snapshot,np.zeros((1,25))
        return args,recorded

    def test_calibration_resumes_current_seed_without_losing_denominator(self):
        args,recorded = self.calibration_fixture()
        calls = []
        def interrupted(candidate,delay=0.,**kwargs):
            calls.append((candidate.approval_s,delay))
            if len(calls)==3:
                raise RuntimeError('fixture interruption')
            return recorded(candidate,delay,**kwargs)
        out = self.folder/'calibration'
        with patch('geofence_turn.calibration.execute',side_effect=interrupted), patch('geofence_turn.calibration.check_memory',return_value=(100,3000)):
            with self.assertRaisesRegex(RuntimeError,'fixture interruption'):
                calibrate_development(out,args,start_work(100,120),Config())
        data = read(out/'calibrated_scenarios.json')
        self.assertEqual(len(data['progress']['current']['candidates']),1)
        cached_approval = data['progress']['current']['candidates'][0]['approval_s']
        args.resume = True
        with patch('geofence_turn.calibration.execute',side_effect=recorded) as flown, patch('geofence_turn.calibration.check_memory',return_value=(100,3000)),contextlib.redirect_stdout(io.StringIO()):
            calibrate_development(out,args,start_work(100,120),Config())
        self.assertFalse(any(call.args[0].approval_s==cached_approval for call in flown.call_args_list))
        self.assertFalse(any(len(call.args)>1 and call.args[1] is None for call in flown.call_args_list))
        data = read(out/'calibrated_scenarios.json')
        self.assertEqual(data['statistics']['attempted'],1)
        self.assertEqual(data['statistics']['accepted'],1)
        self.assertNotIn('progress',data)
        self.assertEqual(load_development(out)[0][0].approval_s,2.)
        args.target = 2
        with self.assertRaisesRegex(ValueError,'inputs/settings'):
            calibrate_development(out,args,start_work(100,120),Config())

    def test_calibration_diagnostics_are_explicit(self):
        args,recorded = self.calibration_fixture()
        args.diagnostics = True
        out = self.folder/'calibration'
        with patch('geofence_turn.calibration.execute',side_effect=recorded),patch('geofence_turn.calibration.check_memory',return_value=(100,3000)),contextlib.redirect_stdout(io.StringIO()):
            calibrate_development(out,args,start_work(100,120),Config())
        self.assertEqual({p.name for p in (out/'diagnostics').iterdir()},{'calibration.json','seed_0_d0.npz'})

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


if __name__ == '__main__':
    unittest.main()
