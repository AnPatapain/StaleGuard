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
            out = prepare_output(Path(folder)/'calibration', Config())
            args = SimpleNamespace(resume=False, generated=str(DEV/'generated.json'), attempts=10,target=5)
            work = start_work(100,120)
            with patch('geofence_turn.calibration.execute',side_effect=recorded_flight) as flown, contextlib.redirect_stdout(io.StringIO()):
                calibrate_development(out,args,work,Config())
            self.assertEqual(flown.call_count,43)
            exported = load_development(out/'calibrated_scenarios.json')
            self.assertEqual([s.approval_s for s,_,_ in exported],[2.,1.8,1.8,1.4,1.4])
            self.assertEqual([s.command for s,_,_ in exported],[cases[i][0].command for i in sorted(cases)])
            attempts = (out/'attempts.jsonl').read_bytes()
            prepare_output(out,Config(),resume=True)
            args.resume = True
            with patch('geofence_turn.calibration.execute') as flown, contextlib.redirect_stdout(io.StringIO()):
                calibrate_development(out,args,work,Config())
            flown.assert_not_called()
            self.assertEqual((out/'attempts.jsonl').read_bytes(),attempts)

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
        for name in ('diagnostics.py','validation.py','mission.py','tests','logs','results'):
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


if __name__ == '__main__':
    unittest.main()
