"""Small scientific regression checks: python -m staleguard.reproduce.tests."""

from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import json
import unittest

from staleguard.policies import POLICIES, LABELS
from staleguard.underpass_inspection.baselines.policies import make_policy, POLICIES as IMPLEMENTED
from staleguard.underpass_inspection.baselines.recovery_horizons import (
    _boundary,
    completion_expiry_steps,
    policy_plans,
)
from staleguard.underpass_inspection.baselines.staleguard import first_ready_step
from staleguard.robustness.protocol import save_record, load_record
from staleguard.robustness.archive import verify_archive
from .common import Data


def forecast(command=(True, True, True, True), completion=None, recovery=None, runtime=0.0):
    completion = command if completion is None else completion
    recovery = (True,) * 4 if recovery is None else recovery
    rows = [
        dict(delay_s=i * 0.2, command_safe=c, safe=c, command_complete=q, recovery_safe=r)
        for i, (c, q, r) in enumerate(zip(command, completion, recovery))
    ]
    boundaries = {
        name: _boundary(rows, field)
        for name, field in [
            ('command', 'command_safe'),
            ('completion', 'command_complete'),
            ('recovery', 'recovery_safe'),
        ]
    }
    value = dict(
        results=rows,
        boundaries=boundaries,
        expiry_steps=boundaries['command']['first_invalid_steps'],
        computation_s=runtime,
    )
    value['policy_plans'] = policy_plans(value)
    return value


class PolicyRegression(unittest.TestCase):
    def agent(self, profile, name='staleguard_dual_horizon'):
        agent = make_policy(name, SimpleNamespace(approval_s=0), profile)
        agent.approve()
        return agent

    def test_command_and_recovery_use_first_loss(self):
        value = forecast((True, False, True, True), recovery=(True, True, False, True))
        self.assertEqual(value['expiry_steps'], 10)
        self.assertEqual(value['boundaries']['recovery']['first_invalid_steps'], 20)
        self.assertEqual(value['policy_plans']['staleguard_recovery_aware']['trigger_steps'], 10)

    def test_completion_waits_for_later_success(self):
        value = forecast(completion=(True, False, True, False))
        self.assertEqual(completion_expiry_steps(value['results']), 30)
        agent = self.agent(value)
        agent.announce_readiness(10)
        agent.tick(10)
        self.assertIsNone(agent.take_action())
        agent.tick(30)
        self.assertEqual(agent.take_action(), 'sideways')

    def test_recovery_uses_safe_prefix_before_first_loss(self):
        value = forecast(recovery=(True, True, False, True))
        self.assertEqual(value['boundaries']['recovery']['first_invalid_steps'], 20)
        agent = self.agent(value)
        agent.announce_readiness(9)
        agent.tick(9)
        self.assertIsNone(agent.take_action())
        agent.tick(10)
        self.assertEqual(agent.take_action(), 'sideways')

    def test_dispatch_inside_completion_gap_rejects_when_recoverable(self):
        agent = self.agent(forecast(completion=(True, False, True, False)))
        agent.announce_readiness(10)
        agent.dispatch(10, None, None)
        self.assertEqual(agent.take_action(), 'sideways')
        self.assertEqual(agent.event['recovery_reason'], 'completion_at_dispatch')

    def test_undefined_boundaries_do_not_invent_loss(self):
        value = forecast()
        self.assertIsNone(value['expiry_steps'])
        self.assertIsNone(completion_expiry_steps(value['results']))
        self.assertIsNone(value['policy_plans']['staleguard_dual_horizon']['trigger_steps'])
        agent = self.agent(value)
        agent.announce_readiness(40)
        agent.tick(40)
        self.assertIsNone(agent.take_action())

    def test_late_prediction_fails_open(self):
        agent = self.agent(forecast((True, False, False, False), runtime=0.21))
        agent.announce_readiness(10)
        agent.dispatch(10, None, None)
        self.assertEqual(agent.take_action(), 'dive')
        agent.announce_readiness(11)
        agent.tick(11)
        self.assertIsNone(agent.take_action())

    def test_ready_prediction_recovers_before_dispatch_and_cancels(self):
        agent = self.agent(forecast((True, False, False, False), runtime=0.04))
        agent.announce_readiness(2)
        agent.tick(10)
        self.assertEqual(agent.take_action(), 'sideways')
        agent.dispatch(20, None, None)
        self.assertIsNone(agent.take_action())
        self.assertTrue(agent.event['command_canceled'])

    def test_ready_dispatch_rejects_expired_command(self):
        agent = self.agent(forecast((True, False, False, False)), 'staleguard')
        agent.announce_readiness(10)
        agent.dispatch(10, None, None)
        self.assertEqual(agent.take_action(), 'sideways')

    def test_readiness_clock_boundary(self):
        self.assertEqual(first_ready_step(0.12), 6)
        self.assertEqual(first_ready_step(0.120001), 7)

    def test_all_eight_policy_names_and_factories(self):
        self.assertEqual(set(POLICIES), set(IMPLEMENTED))
        self.assertEqual(LABELS['staleguard_dual_horizon'], 'StaleGuard (Three-opportunity-aware)')
        self.assertEqual(LABELS['staleguard_early_recovery'], 'StaleGuard (Command-safety-aware)')
        for name in POLICIES:
            value = make_policy(name, SimpleNamespace(approval_s=0), forecast())
            self.assertEqual(value.event['policy'], name)

    def test_record_integrity_and_reuse_cannot_change_inputs(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / 'record.json'
            save_record(path, dict(action=['sideways', 10], safe=True))
            original = path.read_bytes()
            save_record(path, load_record(path))
            self.assertEqual(path.read_bytes(), original)
            with self.assertRaises(ValueError):
                save_record(path, dict(action=['sideways', 11], safe=True))

    def test_published_grid_and_physical_reuse(self):
        data = Data('data')
        for mission, count in [
            ('inspection_descent', 24288),
            ('geofence_turn', 55200),
            ('underpass_inspection', 41600),
        ]:
            rows, _, _ = data.policy_records(mission)
            self.assertEqual(len(rows), count)
            if mission != 'underpass_inspection':
                self.assertGreater(data.verify_reuse(mission), 0)

    def test_e01_archive(self):
        value = verify_archive(Path('data'))
        self.assertEqual(value['outcomes'], 24960)
        self.assertEqual(value['forecasts'], 320)

    def test_corrupted_dataset_is_rejected(self):
        with TemporaryDirectory() as folder:
            root = Path(folder)
            path = root / 'record.json'
            path.write_text('good')
            from staleguard.io import file_hash

            manifest = dict(
                format='staleguard-primary-data-v1',
                datasets={},
                files={'record.json': dict(bytes=4, sha256=file_hash(path))},
            )
            (root / 'manifest.json').write_text(json.dumps(manifest))
            path.write_text('evil')
            with self.assertRaises(ValueError):
                Data(root).file('record.json')


if __name__ == '__main__':
    unittest.main(verbosity=2)
