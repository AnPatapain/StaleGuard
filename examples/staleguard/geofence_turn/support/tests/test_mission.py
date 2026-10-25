from dataclasses import asdict, FrozenInstanceError, replace
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from geofence_turn.scenario import (BODY_RADIUS_M, HZ, POS, VEL, Config, Command, Scenario,
    apply_turn_command, clearance, in_checkpoint, make_forward_trajectory, turn_sample,
    safety_only_reference, scenario_dict, scenario_from_dict)
from geofence_turn.scenario import generate_scenario
from geofence_turn.calibration import approval_candidates
from geofence_turn.baselines.staleguard import staleguard_status
from geofence_turn.baselines.use_time_recheck import first_step_after_check
from geofence_turn.baselines.fixed_ttl import fixed_ttl_expired


class MissionTests(unittest.TestCase):
    def setUp(self):
        self.config = Config()
        self.cmd = Command(.4, .6, (1.2, 1.5, 1.15), (1.8, 1.7, 1.35))
        self.s = Scenario(0, .6, 1.25, .05, -.05, 2., 7., self.cmd, 1.2)
        self.ref = make_forward_trajectory(650, self.s)
        self.anchor = self.ref[60]

    def envelope(self):
        return dict(config=asdict(self.config), results=[dict(speed=v, radius=r, feasible=True)
                    for v in (.4, .6, .8) for r in (.4, .6)])

    def test_seed_generation_deterministic_and_independent(self):
        for seed in range(20):
            try:
                s = generate_scenario(seed, self.envelope())
            except ValueError as error:
                self.assertIn('checkpoint_geometry:', str(error))
                with self.assertRaisesRegex(ValueError, 'checkpoint_geometry:'):
                    generate_scenario(seed, self.envelope())
                continue
            self.assertEqual(s, generate_scenario(seed, self.envelope()))
            self.assertEqual(s, scenario_from_dict(scenario_dict(s)))
            self.assertGreaterEqual(clearance(s.command.checkpoint_upper[0], s.fence_x), self.config.d_min_m-1e-12)
        self.assertNotEqual(generate_scenario(0, self.envelope()), generate_scenario(1, self.envelope()))
        with self.assertRaises(ValueError):
            generate_scenario(0, dict(config=asdict(self.config), results=[]))

    def test_command_is_frozen_and_target_never_varies_with_candidate(self):
        with self.assertRaises(FrozenInstanceError):
            self.cmd.radius = .7
        for s in approval_candidates(self.s,3.2):
            self.assertIs(s.command, self.cmd)
            self.assertEqual(s.command.checkpoint_lower, self.cmd.checkpoint_lower)
            self.assertEqual(s.command.checkpoint_upper, self.cmd.checkpoint_upper)

    def test_left_turn_is_horizontal_course_change(self):
        start, initial_v = turn_sample(0, self.anchor, self.cmd)
        end, final_v = turn_sample(self.cmd.arc_s, self.anchor, self.cmd)
        np.testing.assert_allclose(initial_v, [.6, 0, 0], atol=1e-12)
        np.testing.assert_allclose(final_v, [0, .6, 0], atol=1e-12)
        np.testing.assert_allclose(end-start, [.4, .4, 0], atol=1e-12)
        changed = apply_turn_command(self.ref, 60, self.cmd)
        np.testing.assert_array_equal(changed[:, 8], self.ref[:, 8])
        self.assertGreater(changed[-1, 2], start[1])

    def test_positions_velocities_continuous_at_every_phase_boundary(self):
        eps = 1e-7
        for t in (0., self.cmd.arc_s):
            p, v = turn_sample(t, self.anchor, self.cmd)
            if t == 0:
                np.testing.assert_allclose(p, self.anchor[POS], atol=1e-12)
                np.testing.assert_allclose(v, self.anchor[VEL], atol=1e-12)
            else:
                before = turn_sample(t-eps, self.anchor, self.cmd)
                after = turn_sample(t+eps, self.anchor, self.cmd)
                np.testing.assert_allclose(before[0], after[0], atol=2e-7)
                np.testing.assert_allclose(before[1], after[1], atol=2e-7)

    def test_delays_change_lane_but_never_move_region_or_replan(self):
        bounds = (self.cmd.checkpoint_lower, self.cmd.checkpoint_upper)
        for step in (30, 60, 110, 160):
            changed = apply_turn_command(self.ref, step, self.cmd)
            np.testing.assert_array_equal(changed[:step], self.ref[:step])
            np.testing.assert_allclose(changed[step, POS], self.ref[step, POS])
            np.testing.assert_allclose(changed[step, VEL], self.ref[step, VEL])
            self.assertTrue(np.all(np.diff(changed[:,0]) >= -1e-12))
            self.assertTrue(np.all(changed[:,1] >= 0))
            north = step + math.ceil(self.cmd.arc_s*HZ)
            np.testing.assert_allclose(changed[north:,0], self.ref[step,0]+self.cmd.radius)
            np.testing.assert_allclose(changed[north:,1], 0)
            np.testing.assert_allclose(changed[north:,3], self.cmd.speed)
            self.assertGreater(changed[-1,2], self.cmd.checkpoint_upper[1])
            self.assertEqual(bounds, (self.cmd.checkpoint_lower,self.cmd.checkpoint_upper))

    def test_delay_can_miss_a_fixed_checkpoint_instead_of_replanning(self):
        command = replace(self.cmd,checkpoint_lower=(.9,1.5,1.15),checkpoint_upper=(1.2,1.7,1.35))
        offset = math.ceil((command.arc_s+2)*HZ)
        immediate = apply_turn_command(self.ref,60,command)[60+offset]
        delayed = apply_turn_command(self.ref,110,command)[110+offset]
        self.assertTrue(in_checkpoint(immediate[POS],immediate[VEL],command,self.config))
        self.assertFalse(in_checkpoint(delayed[POS],delayed[VEL],command,self.config))
        self.assertGreater(delayed[0],command.checkpoint_upper[0])

    def test_checkpoint_geometry_does_not_steer_trajectory(self):
        shifted = replace(self.cmd, checkpoint_lower=(10.,20.,1.15), checkpoint_upper=(11.,21.,1.35))
        np.testing.assert_array_equal(apply_turn_command(self.ref,60,self.cmd),
                                      apply_turn_command(self.ref,60,shifted))
        self.assertFalse(in_checkpoint([10.5,20.5,1.25], [0,0,0], shifted, self.config))

    def test_samples_match_analytic_turn_and_unlimited_north_leg(self):
        for step in (60, 110):
            changed = apply_turn_command(self.ref, step, self.cmd)
            for offset in (0,1,50,100,300):
                p,v = turn_sample(offset/HZ, self.ref[step], self.cmd)
                np.testing.assert_allclose(changed[step+offset,POS],p,atol=1e-12)
                np.testing.assert_allclose(changed[step+offset,VEL],v,atol=1e-12)
        for step in (5,-1,len(self.ref)):
            with self.assertRaises(ValueError):
                apply_turn_command(self.ref, step, self.cmd)

    def test_footprint_matches_scg_collision_and_encloses_every_orientation(self):
        path = Path(__file__).resolve().parents[5]/'safe_control_gym/envs/gym_pybullet_drones/assets/cf2x.urdf'
        cylinder = ET.parse(path).find('.//collision/geometry/cylinder')
        r, h = float(cylinder.attrib['radius']), float(cylinder.attrib['length'])/2
        self.assertEqual(BODY_RADIUS_M, math.hypot(r, h))
        # Tilted cylinder support in +X never exceeds the enclosing sphere.
        for angle in np.linspace(0, math.pi, 100):
            support = r*abs(math.cos(angle))+h*abs(math.sin(angle))
            self.assertLessEqual(support, BODY_RADIUS_M+1e-14)
        self.assertAlmostEqual(clearance(1-BODY_RADIUS_M, 1), 0)
        self.assertAlmostEqual(clearance(1-BODY_RADIUS_M-.05, 1), .05)
        self.assertLess(clearance(1., 1.), 0)

    def test_completion_requires_fixed_region_and_northbound_velocity(self):
        center = [1.5,1.6,1.25]
        self.assertTrue(in_checkpoint(center, [0,.6,0], self.cmd, self.config))
        self.assertFalse(in_checkpoint(center, [0,0,0], self.cmd, self.config))
        self.assertFalse(in_checkpoint(center, [0,-.6,0], self.cmd, self.config))
        self.assertFalse(in_checkpoint([1.81,1.6,1.25], [0,.6,0], self.cmd,self.config))
        self.assertFalse(in_checkpoint([1.5,1.71,1.25], [0,.6,0], self.cmd,self.config))
        self.assertFalse(in_checkpoint(center, [.13,.6,0], self.cmd,self.config))

    def test_safety_only_hover_never_counts_as_northbound_completion(self):
        changed = safety_only_reference(self.ref,60,self.anchor)
        np.testing.assert_allclose(changed[-1,POS], self.anchor[POS])
        np.testing.assert_allclose(changed[-1,VEL],0)
        self.assertFalse(in_checkpoint([1.5,1.6,1.25],changed[-1,VEL],self.cmd,self.config))

    def test_old_connector_scenarios_are_rejected_explicitly(self):
        legacy = scenario_dict(self.s)
        legacy.pop('schema')
        with self.assertRaisesRegex(ValueError, 'regenerate'):
            scenario_from_dict(legacy)

    def test_incompatible_fixed_region_geometry_is_rejected(self):
        cfg = replace(self.config, checkpoint_width_m=10.)
        envelope = self.envelope()
        envelope['config'] = asdict(cfg)
        with self.assertRaisesRegex(ValueError, 'checkpoint_geometry:'):
            generate_scenario(0,envelope,cfg)




    def test_approval_grid_uses_measured_bound_and_ends_at_minimum(self):
        rows = list(approval_candidates(self.s,4.2))
        self.assertEqual([s.approval_s for s in rows[:5]],[4.2,4.,3.8,3.6,3.4])
        self.assertEqual(rows[-1].approval_s,.6)
        self.assertEqual(len(rows),19)
        self.assertTrue(all(s.command == self.cmd for s in rows))

    def test_staleguard_readiness_equality_fail_open_and_no_late_cancellation(self):
        received = dict(computation_s=.1, expiry_steps=5)
        self.assertEqual(staleguard_status(received, 0), (False, False))
        self.assertEqual(staleguard_status(received, 4), (False, False))
        self.assertEqual(staleguard_status(received, 5), (True, True))
        self.assertEqual(staleguard_status(dict(computation_s=0, expiry_steps=None), 50), (True, False))

    def test_recheck_tick_and_fixed_ttl_equality_match_existing_conventions(self):
        for elapsed, expected in ((0,11), (.001,11), (.02,12), (.021,12), (.04,13)):
            self.assertEqual(first_step_after_check(10, elapsed), expected)
        self.assertFalse(fixed_ttl_expired(20))
        self.assertTrue(fixed_ttl_expired(21))


if __name__ == '__main__':
    unittest.main()
