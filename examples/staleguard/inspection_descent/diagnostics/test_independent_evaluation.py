"""Streaming, durable resume and scenario-level statistical tests; no simulator."""
from dataclasses import replace
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import independent_evaluation as e
import independent_report as report


class IndependentEvaluationTests(unittest.TestCase):
    def test_streamed_dwell_matches_official_elapsed_tick_measure(self):
        scenario = e.b.generate_scenario(20)
        scan = e.b.inspection_x(scenario)
        positions = [(scan, 0, scenario.low_height)]*4+[(scan+.5, 0, scenario.low_height)]+[(scan, 0, scenario.low_height)]*7
        rows = list(enumerate(positions))
        summary = e.PhysicsSummary(scenario)
        for row in rows:
            summary.append(row)
        self.assertEqual(summary.longest, e.calibration.inspection_dwell_ticks(rows, scenario))
        self.assertEqual(summary.longest, 6)
        self.assertFalse(hasattr(summary, 'positions'))
        self.assertEqual(summary.first_unsafe_time_s, .004)

    def test_journal_recovers_only_committed_prefix(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'records.jsonl'
            e.append_record(path, {'rank': 1})
            with path.open('ab') as stream:
                stream.write(b'{"rank":2')
            self.assertEqual(e.journal(path), [{'rank': 1}])
            e.append_record(path, {'rank': 2})
            self.assertEqual(e.journal(path), [{'rank': 1}, {'rank': 2}])

    def calibration_fixture(self, folder, dwells):
        scenario = e.b.generate_scenario(20)
        candidates = [replace(scenario, approval_time=0, low_seconds=1+i/10) for i in range(3)]
        simulations = []
        def actual(candidate, snapshot, physics_trace, **kwargs):
            index = candidates.index(candidate)
            simulations.append(index)
            for tick in range(dwells[index]+1):
                physics_trace.append((tick, (e.b.inspection_x(candidate), 0, candidate.low_height)))
            complete = dwells[index] >= 180
            return complete, False, False, complete
        return candidates, simulations, actual

    def test_calibration_resume_reuses_candidates_and_preserves_first_tie(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            candidates, simulations, actual = self.calibration_fixture(folder, [180, 250, 250])
            with patch.object(e.calibration, 'command_candidates', return_value=candidates), \
                 patch.object(e.calibration, 'capture_approval', return_value='snapshot'), \
                 patch.object(e.calibration, 'run_actual', side_effect=actual), \
                 patch.object(e, 'check_memory'), patch.object(e, 'release'):
                with self.assertRaises(e.CheckpointPause):
                    e.calibrate(output, 20, pause_after=1)
                selected = e.calibrate(output, 20)
                self.assertEqual(simulations, [0, 1, 2])
                self.assertEqual(selected['selected_rank'], 2)
                self.assertEqual(selected['command']['low_seconds'], 1.1)
                self.assertEqual(e.calibrate(output, 20), selected)
                self.assertEqual(simulations, [0, 1, 2])

    def test_infeasible_calibration_does_not_issue_or_replace_command(self):
        with tempfile.TemporaryDirectory() as folder:
            candidates, simulations, actual = self.calibration_fixture(folder, [100, 141, 141])
            with patch.object(e.calibration, 'command_candidates', return_value=candidates), \
                 patch.object(e.calibration, 'capture_approval', return_value='snapshot'), \
                 patch.object(e.calibration, 'run_actual', side_effect=actual), \
                 patch.object(e, 'check_memory'), patch.object(e, 'release'):
                selected = e.calibrate(Path(folder), 20)
                self.assertIsNone(selected['command'])
                self.assertEqual(selected['seed'], 20)
                self.assertEqual(selected['best_safe_dwell_ticks'], 141)

    def test_bootstrap_keeps_policies_and_delays_in_same_cluster(self):
        values = np.zeros((4, 11, 2, 1))
        for scenario in range(4):
            values[scenario, :, :, 0] = scenario
        with patch.object(report, 'check_memory'):
            draws = report.cluster_bootstrap(values, replicates=100, random_seed=7)
            again = report.cluster_bootstrap(values, replicates=100, random_seed=7)
        np.testing.assert_array_equal(draws, again)
        for delay in range(11):
            np.testing.assert_array_equal(draws[:, delay, 0], draws[:, 0, 1])
        self.assertEqual(report.interval(draws[:, 0, 0, 0]-draws[:, 10, 1, 0]), [0, 0])

    def test_sign_test_holm_and_feasibility_uncertainty(self):
        self.assertEqual(report.paired_sign_test([1, 1, 1, 1, 0])['p_value'], .125)
        self.assertEqual(report.paired_sign_test([0, 0])['p_value'], 1)
        rows = [{'p_value': .01}, {'p_value': .04}, {'p_value': .03}]
        report.holm_adjust(rows)
        self.assertEqual([r['p_value_holm'] for r in rows], [.03, .06, .06])
        self.assertGreater(report.wilson(0, 300)[1], 0)
        self.assertLess(report.wilson(300, 300)[0], 1)

    def test_complete_report_schema_uses_new_feasible_count_and_six_columns(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)
            e.atomic_json(output/'protocol.json', {'bootstrap': {'replicates': 20, 'random_seed': 1729}})
            for seed in e.SEEDS:
                directory = e.seed_directory(output, seed)
                e.atomic_json(directory/'calibration.json', dict(seed=seed, command={} if seed == 20 else None, candidates=1))
            directory = e.seed_directory(output, 20)
            e.atomic_json(directory/'reception.json', {'computation_s': .01})
            for steps in e.DELAY_STEPS:
                for policy in e.POLICIES:
                    fallback = steps > 15 and policy in ('Always-Recover', 'StaleGuard', 'Fixed TTL (0.3s)')
                    event = {}
                    if policy == 'FBS':
                        event = dict(prediction={'ttl_steps': 210}, observation_update_s=[.0001], cold_start=True)
                    row = dict(seed=20, delay_steps=steps, delay_s=steps/50, policy=policy,
                               outcome=[not fallback, False, fallback, not fallback],
                               execution_kind='native', prediction_ready=steps > 0,
                               prediction_computation_s=.001 if policy in ('StaleGuard','FBS','Use-Time Recheck') else None,
                               event=event)
                    e.append_record(directory/'flights.jsonl', row)
            with patch.object(report, 'check_memory'), patch.object(report, 'plot_tradeoff'):
                report.generate_report(output)
            result = json.loads((output/'summary.json').read_text())
            self.assertEqual(result['calibration']['attempted'], 300)
            self.assertEqual(result['calibration']['feasible'], 1)
            self.assertEqual(len(result['paired']), 180)
            text = (output/'six_policy_report.log').read_text()
            self.assertIn('D = 0.3s | T = 4.2s | N = 1', text)
            self.assertIn('Fixed TTL (0.3s)', text)
            self.assertIn('STALEGUARD vs USE-TIME RECHECK', text)
            self.assertLessEqual(len((output/'interpretation.txt').read_text().splitlines()), 15)


if __name__ == '__main__':
    unittest.main()
