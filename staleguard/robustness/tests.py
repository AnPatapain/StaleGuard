"""Executable deterministic E01 unit/integration checks; evidence stays under E01."""

from dataclasses import asdict
from pathlib import Path
from unittest.mock import patch
import contextlib
import inspect
import io
import json
import shutil
import sys
import types
import unittest

import numpy as np

from staleguard.io import file_hash
from staleguard.model.scg import load_scg_platform_params
from staleguard.reproduce.common import Data
from staleguard.reproduce.run_experiments import load_cases
from staleguard.underpass_inspection.simulation import execute
from .protocol import (
    ROOT,
    REPO,
    CONDITIONS,
    POLICIES,
    freeze,
    perturbation,
    save_record,
    load_record,
    atomic_json,
)
from . import physics, runner


class E01Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = freeze(cls.data_root)
        runner.prepare_archive(cls.data_root, cls.spec)
        cases = load_cases(Data(cls.data_root), 'underpass_inspection')
        cls.seed = cls.spec['selected_seeds'][0]
        _, cls.scenario, cls.archived_snapshot = next(r for r in cases if r[0] == cls.seed)
        cls.platform = load_scg_platform_params()
        cls.uncertainties = {
            c: dict(perturbation(cls.spec, c, cls.seed), audit_scope='verification')
            for c in CONDITIONS
        }
        cls.flights = {}
        approval = round(cls.scenario.approval_s * 50)
        steps = [approval, approval + 10]
        for c in CONDITIONS:
            cls.flights[c] = physics.physical(
                cls.scenario, cls.uncertainties[c], ('dive', approval), steps
            )
        cls.snapshots = {c: value[1][approval] for c, value in cls.flights.items()}
        physics.warm(cls.scenario, cls.snapshots['N0'], cls.platform)

    def test_01_locked_selection_and_sizes(self):
        self.assertEqual(len(set(self.spec['selected_seeds'])), 64)
        population = Data(self.data_root).entry('underpass_inspection/benchmark')['scenarios']
        wanted = np.random.default_rng(2027).choice(sorted(population), 64, False).tolist()
        self.assertEqual(wanted, self.spec['selected_seeds'])
        self.assertEqual(64 * 13 * 6 * 5, self.spec['expected_h3_outcomes'])
        self.assertEqual(len(self.spec['policy_delays']), 13)
        self.assertEqual(len(self.spec['prediction_delays']), 19)

    def test_02_balanced_paired_mass_signs(self):
        signs = list(self.spec['mass_signs'].values())
        self.assertEqual(signs.count(1), 32)
        self.assertEqual(signs.count(-1), 32)
        for seed in self.spec['selected_seeds']:
            a = perturbation(self.spec, 'A1', seed)
            b = perturbation(self.spec, 'A2', seed)
            self.assertEqual(a['mass_sign'], b['mass_sign'])
            self.assertAlmostEqual(b['mass_factor'] - 1, 2 * (a['mass_factor'] - 1))

    def test_03_nominal_injection_is_exact(self):
        original, arrays = execute(self.scenario, 'dive', 0)
        hooked = self.flights['N0'][0]
        self.assertEqual(original['physics_sha256'], hooked['physics_sha256'])
        self.assertEqual(original['safe'], hooked['safe'])
        self.assertEqual(original['safe_completion'], hooked['safe_completion'])
        snap = self.snapshots['N0']
        np.testing.assert_array_equal(
            physics.Hooks(self.uncertainties['N0']).observation(snap).state, snap.state
        )

    def test_04_true_mass_only_with_nominal_pid_and_model(self):
        nominal = self.flights['N0'][0]['physical_plant']
        for c in ('A1', 'A2'):
            actual = self.flights[c][0]['physical_plant']
            self.assertAlmostEqual(
                actual['actual_base_mass_kg'],
                nominal['actual_base_mass_kg'] * self.uncertainties[c]['mass_factor'],
            )
            self.assertEqual(actual['nominal_controller_mass_kg'], self.platform[0].mass)
            self.assertEqual(actual['pid_gains'], nominal['pid_gains'])
            self.assertEqual(actual['inertia_before'], actual['inertia_after'])
            self.assertEqual(actual['inertia_after'], nominal['inertia_after'])

    def test_05_mass_positive_control_changes_physics(self):
        self.assertNotEqual(
            self.flights['A1'][0]['physics_sha256'], self.flights['N0'][0]['physics_sha256']
        )
        self.assertNotEqual(
            self.flights['A2'][0]['physics_sha256'], self.flights['N0'][0]['physics_sha256']
        )
        self.assertFalse(np.array_equal(self.snapshots['A2'].state, self.snapshots['N0'].state))

    def test_06_bias_does_not_touch_physics_or_pid(self):
        for c in ('B1', 'B2'):
            self.assertEqual(
                self.flights[c][0]['physics_sha256'], self.flights['N0'][0]['physics_sha256']
            )
            self.assertEqual(
                self.flights[c][0]['control_sha256'], self.flights['N0'][0]['control_sha256']
            )
            snap = self.snapshots['N0']
            observed = physics.Hooks(self.uncertainties[c]).observation(snap)
            self.assertFalse(np.array_equal(snap.state[:6], observed.state[:6]))
            np.testing.assert_array_equal(snap.state[6:], observed.state[6:])
            self.assertIs(observed.pid_state, snap.pid_state)
            self.assertIs(observed.remaining_reference, snap.remaining_reference)
            self.assertEqual(observed.prefix_min_clearance_m, snap.prefix_min_clearance_m)

    def test_07_bias_scaling_and_order_independence(self):
        for seed in reversed(self.spec['selected_seeds']):
            a, b = (perturbation(self.spec, c, seed) for c in ('B1', 'B2'))
            np.testing.assert_allclose(np.asarray(a['position_bias_m']) * 3, b['position_bias_m'])
            np.testing.assert_allclose(
                np.asarray(a['velocity_bias_mps']) * 2.5, b['velocity_bias_mps']
            )
            for policy in POLICIES:
                self.assertEqual(perturbation(self.spec, 'B2', seed), b)

    def test_08_forecast_inputs_no_delay_truth_or_perturbed_mass(self):
        self.assertEqual(
            list(inspect.signature(physics.forecast).parameters),
            ['snapshot', 'uncertainty', 'platform'],
        )
        snap = self.snapshots['A2']
        before = snap.state.copy()
        result = physics.forecast(snap, self.uncertainties['A2'], self.platform)
        self.assertEqual(result['nominal_predictor_mass_kg'], self.platform[0].mass)
        self.assertEqual(len(result['results']), 19)
        np.testing.assert_array_equal(snap.state, before)

    def test_09_dispatch_recheck_uses_biased_snapshot(self):
        snap = self.snapshots['N0']
        requested = []

        def recheck(snapshot, platform):
            requested.append(snapshot.state.copy())
            return dict(safe=True)

        with patch('staleguard.underpass_inspection.baselines.policies.predict_immediate', recheck):
            decision = physics.decide(
                self.scenario,
                0,
                'use_time_recheck',
                None,
                snap,
                self.uncertainties['B2'],
                self.platform,
            )
        np.testing.assert_array_equal(
            requested[0], physics.Hooks(self.uncertainties['B2']).observation(snap).state
        )
        self.assertEqual(len(requested), 1)

    def test_10_cache_equivalence_and_plant_separation(self):
        scratch = ROOT / 'verification/cache_fixture'
        with patch.object(runner, 'ROOT', scratch):
            cache = runner.Cache(self.spec, self.scenario, 'A1', self.data_root, 'test-v1')
            cache.uncertainty['audit_scope'] = 'verification'
            action = ('sideways', round(self.scenario.approval_s * 50))
            first = cache.get(action, allow_archive=False)
            second = cache.get(action, allow_archive=False)
            independently, _ = physics.physical(self.scenario, cache.uncertainty, action)
            self.assertEqual(first['record'], second['record'])
            self.assertEqual(first['record']['physics_sha256'], independently['physics_sha256'])
            other = runner.Cache(self.spec, self.scenario, 'A2', self.data_root, 'test-v1')
            self.assertNotEqual(cache.path(action), other.path(action))
            nominal = runner.Cache(self.spec, self.scenario, 'N0', self.data_root, 'test-v1')
            biased = runner.Cache(self.spec, self.scenario, 'B2', self.data_root, 'test-v1')
            self.assertEqual(nominal.path(action), biased.path(action))

    def test_11_atomic_resume_and_corruption_detection(self):
        path = ROOT / 'verification/resume/record.json'
        payload = dict(seed=self.seed, completed=True)
        save_record(path, payload)
        before = path.read_bytes()
        save_record(path, payload)
        self.assertEqual(path.read_bytes(), before)
        self.assertEqual(load_record(path), payload)
        path.with_suffix('.json.interrupted.tmp').write_text('{unfinished')
        self.assertEqual(load_record(path), payload)
        with self.assertRaises(ValueError):
            save_record(path, dict(seed=-1))
        corrupt = path.with_name('corrupt.json')
        corrupt.write_text('{"payload": {}, "sha256": "invalid"}')
        with self.assertRaises(ValueError):
            load_record(corrupt)

    def test_12_canonical_data_integrity(self):
        manifest = json.loads((self.data_root / 'manifest.json').read_text())
        for name, entry in manifest['files'].items():
            path = self.data_root / name
            self.assertEqual(path.stat().st_size, entry['bytes'], name)
            self.assertEqual(file_hash(path), entry['sha256'], name)

    def test_13_condition_ground_truth_and_unfavorable_cases_retained(self):
        for c in CONDITIONS:
            snap = self.snapshots[c]
            self.assertEqual(snap.scenario, self.scenario)
            self.assertEqual(snap.approval_time, self.scenario.approval_s)
            row = self.flights[c][0]
            self.assertEqual(row['seed'], self.seed)
            self.assertEqual(
                row['physics_samples'],
                round((self.scenario.approval_s + self.scenario.observation_s) * 1000) + 1,
            )
        archive = load_record(ROOT / 'archive' / f'{self.seed}.json')
        nominal = next(r for r in archive['h2'] if r['mode'] == 'dive' and r['start_s'] == 0)
        self.assertEqual(nominal['physics_sha256'], self.flights['N0'][0]['physics_sha256'])

    def test_14_late_forecast_fails_open_beyond_prediction_grid(self):
        received = physics.forecast(self.snapshots['N0'], self.uncertainties['N0'], self.platform)
        received['computation_s'] = 20.0
        for policy in (
            'staleguard_early_recovery',
            'staleguard_recovery_aware',
            'staleguard_dual_horizon',
        ):
            decision = physics.decide(
                self.scenario, 2.0, policy, received, None, self.uncertainties['N0'], self.platform
            )
            self.assertEqual(
                decision['action'], ['dive', round(self.scenario.approval_s * 50) + 100]
            )
            self.assertFalse(decision['event']['prediction_ready_at_dispatch'])

    def test_15_snapshot_checkpoint_round_trip(self):
        path = ROOT / 'verification/checkpoints.npz'
        step = round(self.scenario.approval_s * 50)
        physics.save_snapshots(path, {step: self.snapshots['A2']})
        actual = physics.load_snapshot(path, self.scenario, step)
        expected = self.snapshots['A2']
        for name in ('state', 'remaining_reference'):
            np.testing.assert_array_equal(getattr(actual, name), getattr(expected, name))
        for name in ('integral_pos_e', 'last_rpy', 'integral_rpy_e'):
            np.testing.assert_array_equal(
                getattr(actual.pid_state, name), getattr(expected.pid_state, name)
            )
        self.assertEqual(actual.prefix_min_clearance_m, expected.prefix_min_clearance_m)
        self.assertEqual(actual.horizon_steps, expected.horizon_steps)

    @unittest.skipUnless(
        (ROOT / 'PILOT.json').exists(), 'Run the pilot before interruption integration'
    )
    def test_16_interrupted_pipeline_reuses_forecasts_decisions_and_flights(self):
        fixture = ROOT / 'verification/interrupted_pipeline'
        if fixture.exists():
            shutil.rmtree(fixture)
        for folder in ('snapshots/N0', 'physical/N0', 'forecasts/N0'):
            source = ROOT / folder
            if folder == 'physical/N0':
                shutil.copytree(source / str(self.seed), fixture / folder / str(self.seed))
            else:
                for extension in (
                    ('.json', '.npz') if folder.startswith('snapshots') else ('.json',)
                ):
                    target = fixture / folder / f'{self.seed}{extension}'
                    target.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source / f'{self.seed}{extension}', target)
        policies = ['no_recovery', 'fixed_early_recovery']
        for delay in (0, 10):
            for policy in policies:
                name = f'{delay:03d}_{policy}.json'
                target = fixture / 'decisions/N0' / str(self.seed) / name
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(ROOT / 'decisions/N0' / str(self.seed) / name, target)
        implementation = json.loads((ROOT / 'implementation.json').read_text())['implementation']
        job = (
            self.spec,
            self.seed,
            ['N0'],
            'h3',
            [0.0, 0.2],
            policies,
            str(self.data_root),
            implementation,
        )
        save = runner.save_record
        committed = []

        def interrupted(path, payload):
            save(path, payload)
            if 'outcomes' in path.parts:
                committed.append((path, file_hash(path)))
                raise RuntimeError('Deliberate interruption after the first atomic outcome commit')

        with (
            patch.object(runner, 'ROOT', fixture),
            patch.object(runner, 'save_record', interrupted),
        ):
            failed = runner.seed_job(job)
            self.assertIn('traceback', failed[0])
        self.assertEqual(len(committed), 1)
        with (
            patch.object(runner, 'ROOT', fixture),
            patch.object(physics, 'physical', side_effect=AssertionError('Physical rerun')),
            patch.object(physics, 'forecast', side_effect=AssertionError('Forecast recomputed')),
            patch.object(physics, 'decide', side_effect=AssertionError('Decision recomputed')),
        ):
            resumed = runner.seed_job(job)
        self.assertNotIn('traceback', resumed[0])
        rows = [load_record(p) for p in (fixture / 'outcomes/N0' / str(self.seed)).glob('*.json')]
        self.assertEqual(len(rows), 4)
        self.assertEqual(len({(r['seed'], r['delay_s'], r['policy']) for r in rows}), 4)
        self.assertEqual(file_hash(committed[0][0]), committed[0][1])
        for row in rows:
            name = f'{round(row["delay_s"] * 50):03d}_{row["policy"]}.json'
            original = load_record(ROOT / 'outcomes/N0' / str(self.seed) / name)
            for field in ('safe', 'safe_completion', 'physics_sha256'):
                self.assertEqual(row[field], original[field])
        atomic_json(
            ROOT / 'verification/INTERRUPTION_RESULTS.json',
            dict(
                passed=True,
                unique_outcomes=4,
                duplicate_keys=0,
                physical_reruns=0,
                forecast_recomputations=0,
                decision_recomputations=0,
                previously_committed_outcome_unchanged=True,
            ),
        )

    def test_17_completed_physical_attempt_recovers_without_simulation(self):
        fixture = ROOT / 'verification/orphan_attempt'
        if fixture.exists():
            shutil.rmtree(fixture)
        action = ('dive', round(self.scenario.approval_s * 50))
        save_record(
            fixture / 'attempts' / str(self.seed) / 'completed.json',
            dict(
                scenario=asdict(self.scenario),
                uncertainty=self.uncertainties['A1'],
                action=list(action),
                record=self.flights['A1'][0],
                capture_steps=[],
            ),
        )
        with (
            patch.object(runner, 'ROOT', fixture),
            patch.object(physics, 'physical', side_effect=AssertionError('Unexpected simulation')),
        ):
            cache = runner.Cache(self.spec, self.scenario, 'A1', self.data_root, 'orphan-test')
            recovered = cache.get(action, allow_archive=False)
        self.assertTrue(recovered['cache_hit'])
        self.assertEqual(recovered['source'], 'recovered completed physical execution')
        self.assertEqual(
            recovered['record']['physics_sha256'], self.flights['A1'][0]['physics_sha256']
        )


def run_tests(data_root):
    E01Tests.data_root = Path(data_root)
    log = ROOT / 'verification/tests.log'
    log.parent.mkdir(parents=True, exist_ok=True)
    with (
        log.open('w') as stream,
        contextlib.redirect_stdout(stream),
        contextlib.redirect_stderr(stream),
    ):
        result = unittest.TextTestRunner(stream=stream, verbosity=2).run(
            unittest.defaultTestLoader.loadTestsFromTestCase(E01Tests)
        )
    evidence = dict(
        tests_run=result.testsRun,
        failures=len(result.failures),
        errors=len(result.errors),
        passed=result.wasSuccessful(),
        skipped=len(result.skipped),
        log=str(log.relative_to(ROOT)),
    )
    atomic_json(ROOT / 'verification/TEST_RESULTS.json', evidence)
    print(json.dumps(evidence, indent=2))
    if not result.wasSuccessful():
        raise SystemExit(1)
