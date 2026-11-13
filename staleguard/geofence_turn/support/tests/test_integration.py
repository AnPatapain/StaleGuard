"""Real SCG/PID tests; run serially after the CLI characterization/calibration.

Requires the development artifacts, or set GEOFENCE_DEVELOPMENT to another run.
No stored result is substituted for the three fresh physics flights below.
"""
import fcntl
import json
import os
from pathlib import Path
import tempfile
import unittest
import numpy as np
from staleguard.geofence_turn.scenario import HZ, BODY_RADIUS_M, Config, POS, VEL, apply_turn_command, in_checkpoint, scenario_from_dict
from staleguard.geofence_turn.simulation import execute
from staleguard.geofence_turn.storage import load_snapshot

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT/'support/logs/northbound_review'
DEV = Path(os.environ.get('GEOFENCE_DEVELOPMENT', RUN/'development'))


class IntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not (DEV/'summary.json').exists():
            raise unittest.SkipTest('Run bounded characterization and calibration first')
        cls.lock = (ROOT/'support/logs/.simulation.lock').open('a')
        fcntl.flock(cls.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        cls.paths = sorted(DEV.glob('seed_*/scenario.json'))
        cls.output = tempfile.TemporaryDirectory(prefix='geofence_integration_')

    @classmethod
    def tearDownClass(cls):
        cls.lock.close()
        cls.output.cleanup()

    def save(self, name, result, trace):
        folder = Path(self.output.name)
        (folder/f'{name}.json').write_text(json.dumps(result, indent=2)+'\n')
        np.savez_compressed(folder/f'{name}.npz', controls=trace)

    def test_measured_turns_at_three_speeds_are_curved_horizontal_flight(self):
        # Inspect the actual saved SCG characterization trajectories, not references.
        envelope = json.loads((RUN/'characterization/envelope.json').read_text())
        for speed in (.4, .6, .8):
            row = next(r for r in envelope['results']
                       if r['speed']==speed and r['radius']==.4)
            self.assertTrue(row['feasible'])
            trace = np.load(RUN/f'characterization/trace_v{speed}_r0.4.npz')['controls']
            start = round(1.2*HZ)
            end = round((1.2+np.pi*.4/(2*speed)+.3)*HZ)
            self.assertGreater(trace[end,3]-trace[start,3], .25)  # actual +Y travel
            self.assertGreater(trace[end,4], .8*speed)  # actual +Y velocity
            self.assertLess(abs(trace[end,2]), .4*speed)  # actual VX nearly stopped
            self.assertLess(np.max(np.abs(trace[:,9])), .1)  # yaw did not effect the course change
            self.assertEqual(row['outcome']['physics_samples'], row['outcome']['expected_physics_samples'])

    def test_fresh_d0_replay_restores_approval_and_completes_safely(self):
        path = self.paths[0]
        scenario = scenario_from_dict(json.loads(path.read_text()))
        stored = load_snapshot(path.with_name('approval.pkl'))
        result, snapshot, trace = execute(scenario, save_trace=True)
        self.save('fresh_d0', result, trace)
        original = ROOT/'support/results/refactor_equivalence/reference/d0_seed0.npz'
        if scenario.seed == 0 and DEV.resolve() == (RUN/'development').resolve():
            np.testing.assert_array_equal(trace, np.load(original)['controls'])
        self.assertTrue(result['safe_completion'])
        self.assertLessEqual(result['completion_s'], scenario.deadline_s)
        np.testing.assert_array_equal(snapshot.state, stored.state)
        np.testing.assert_array_equal(snapshot.remaining_reference, stored.remaining_reference)
        self.assertEqual(snapshot.command, stored.command)
        np.testing.assert_allclose(trace[-1, [1+i for i in VEL]], [0,scenario.speed,0],atol=.12)
        arc_end = round((scenario.approval_s+scenario.command.arc_s)*HZ)
        np.testing.assert_allclose(trace[arc_end+1:,14],0,atol=1e-12)
        self.assertGreater(trace[-1,3], scenario.command.checkpoint_upper[1])
        # 1 kHz extrema include and may be stricter than the 50 Hz state samples.
        sampled_min = np.min(scenario.fence_x-trace[:,1]-BODY_RADIUS_M)
        self.assertLessEqual(result['min_clearance_m'], sampled_min+1e-10)
        self.assertGreaterEqual(result['min_clearance_m'], Config().d_min_m)

    def test_fresh_delay_keeps_region_without_endpoint_replanning(self):
        path = self.paths[0]
        scenario = scenario_from_dict(json.loads(path.read_text()))
        result, snapshot, trace = execute(scenario, .6, save_trace=True)
        self.save('fresh_delayed', result, trace)
        original = ROOT/'support/results/refactor_equivalence/reference/baseline_seed0_D0.6_no_recovery.npz'
        if scenario.seed == 0 and DEV.resolve() == (RUN/'development').resolve():
            np.testing.assert_array_equal(trace, np.load(original)['controls'])
        self.assertEqual(snapshot.command, scenario.command)
        # Reference columns: step, actual[12], reference[12].
        np.testing.assert_allclose(trace[-1,14],0,atol=1e-12)
        np.testing.assert_allclose(trace[-1,16],scenario.speed,atol=1e-12)
        expected_x = (scenario.initial_x + scenario.speed*(scenario.approval_s+.6-.2)
                      + scenario.command.radius)
        np.testing.assert_allclose(trace[-1,13],expected_x)
        self.assertGreater(trace[-1,15],scenario.command.checkpoint_upper[1])
        # Delayed lane shifts east; neither checkpoint edge follows it.
        reference = snapshot.remaining_reference
        hypothetical = apply_turn_command(reference,30,snapshot.command)
        np.testing.assert_allclose(hypothetical[-1,0],expected_x)
        self.assertEqual(snapshot.command.checkpoint_lower,scenario.command.checkpoint_lower)
        self.assertEqual(snapshot.command.checkpoint_upper,scenario.command.checkpoint_upper)
        self.assertEqual(result['event']['application_step'], round((scenario.approval_s+.6)*HZ))
        self.assertEqual(result['physics_samples'], result['expected_physics_samples'])

    def test_fresh_predispatch_violation_cannot_be_erased_by_later_turn(self):
        scenario = scenario_from_dict(json.loads(self.paths[0].read_text()))
        # Extend D until actual nominal prefix crosses, while still before T.
        result, snapshot, trace = execute(scenario, scenario.deadline_s-1, save_trace=True)
        self.save('fresh_predispatch_violation', result, trace)
        self.assertIsNotNone(result['pre_dispatch_violation_s'])
        self.assertFalse(result['safe'])
        self.assertFalse(result['safe_completion'])
        self.assertLess(result['pre_dispatch_violation_s'],
                        result['event']['scheduled_dispatch_step']/HZ)


if __name__ == '__main__':
    unittest.main()
