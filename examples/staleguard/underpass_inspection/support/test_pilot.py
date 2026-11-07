"""Geometry, fixed-shape reference and reporting checks; no extra flight sweep."""
from dataclasses import replace
import math
import unittest
import numpy as np

from underpass_inspection.scenario import (Box, Scenario, POS, VEL, HZ,
    BODY_RADIUS_M, box_signed_distance, body_clearances, displacement,
    smooth_step, apply_maneuver, make_nominal, WORLD_TO_NED, world_to_ned)
from underpass_inspection.pilot import deadline_summary


class PilotTests(unittest.TestCase):
    def test_both_lateral_maneuvers_are_positive_ned_east(self):
        s = Scenario()
        for mode, start, duration in (('dive', s.swerve_start_s, s.swerve_s), ('sideways', 0., s.sideways_s)):
            times = np.linspace(start, start+duration, 101)
            positions, velocities = displacement(times, s, mode)
            self.assertLess(positions[-1, 1], positions[0, 1])
            self.assertTrue(np.all(velocities[:, 1] <= 0))
            self.assertGreater(world_to_ned(positions)[-1, 1], world_to_ned(positions)[0, 1])
            self.assertGreater(world_to_ned(velocities)[:, 1].max(), 0)
        with self.assertRaises(ValueError):
            replace(s, swerve_y=-s.swerve_y)

    def test_ned_frame_is_right_handed_and_z_up_physics_stays_unmodified(self):
        np.testing.assert_array_equal(np.cross(WORLD_TO_NED[0], WORLD_TO_NED[1]), WORLD_TO_NED[2])
        self.assertEqual(np.linalg.det(WORLD_TO_NED), 1.)
        np.testing.assert_array_equal(world_to_ned([1., -2., 3.]), [1., 2., -3.])
        np.testing.assert_array_equal(WORLD_TO_NED @ WORLD_TO_NED, np.eye(3))

    def test_signed_distance_faces_corners_and_interior(self):
        box = Box('cube', (0, 0, 0), (1, 1, 1))
        for position, expected in (((-.5, .5, .5), .5), ((1.3, 1.4, .5), .5),
                                   ((1.3, 1.4, 2.2), 1.3), ((.5, .5, .5), -.5),
                                   ((0, .5, .5), 0.)):
            self.assertAlmostEqual(box_signed_distance(position, box), expected)
        safe = BODY_RADIUS_M+.05
        self.assertAlmostEqual(body_clearances((-safe, .5, .5), [box])['cube'], .05)
        self.assertLess(body_clearances((-safe+1e-6, .5, .5), [box])['cube'], .05)

    def test_smooth_reference_derivative_and_internal_joins(self):
        s = Scenario()
        eps = 1e-6
        for mode in ('dive', 'climb', 'sideways'):
            times = np.array([.1, .5, 1., 1.3, 2.6, 3., 3.8, 5.])
            a, velocity = displacement(times, s, mode)
            before, _ = displacement(times-eps, s, mode)
            after, _ = displacement(times+eps, s, mode)
            np.testing.assert_allclose((after-before)/(2*eps), velocity, atol=2e-8)
        f, df = smooth_step(np.array([0., 1.3]), 0., 1.3)
        np.testing.assert_array_equal(f, [0., 1.])
        np.testing.assert_array_equal(df, [0., 0.])
        for t in (s.dive_s, s.swerve_start_s, s.swerve_start_s+s.swerve_s):
            _, before = displacement(np.array([t-eps]), s, 'dive')
            _, after = displacement(np.array([t+eps]), s, 'dive')
            np.testing.assert_allclose(before, after, atol=1e-9)

    def test_measured_anchor_fixed_shape_and_no_input_mutation(self):
        s = Scenario(); reference = make_nominal(441, s)
        original = reference.copy()
        state = np.arange(12, dtype=float)/10; state[POS] = [.4, -.01, 1.35]
        saved = state.copy()
        a = apply_maneuver(reference, 40, state, s, 'dive')
        late_state = state.copy(); late_state[POS] += [.24, .02, .001]
        b = apply_maneuver(reference, 60, late_state, s, 'dive')
        n = len(reference)-60
        np.testing.assert_allclose(b[60:, POS]-a[40:40+n, POS],
                                   np.tile(late_state[POS]-state[POS], (n, 1)), atol=1e-12)
        np.testing.assert_array_equal(a[40, POS], state[POS])
        np.testing.assert_array_equal(a[:40], original[:40])
        np.testing.assert_array_equal(reference, original)
        np.testing.assert_array_equal(state, saved)
        # Changing only the pier cannot change the command or climb definition.
        moved = replace(s, pier_x=2.9, pier_y=.2)
        np.testing.assert_array_equal(apply_maneuver(reference, 40, state, moved, 'dive'), a)
        np.testing.assert_array_equal(displacement(np.arange(20)/HZ, s, 'climb')[0],
                                      displacement(np.arange(20)/HZ, moved, 'climb')[0])

    def test_nonmonotonic_and_censored_deadlines_are_not_hidden(self):
        rows = []
        profiles = {'dive': (True, False, True, False),
                    'climb': (True, True, True, True),
                    'sideways': (True, True, False, False)}
        for mode, flags in profiles.items():
            rows += [dict(mode=mode, start_s=i/HZ, safe=safe, inspection_completed=False)
                     for i, safe in enumerate(flags)]
        result = deadline_summary(rows)
        self.assertEqual(result['t_cmd_s'], .02)
        self.assertTrue(result['by_mode']['dive']['non_monotonic'])
        self.assertEqual(len(result['by_mode']['dive']['safety_intervals']), 4)
        self.assertIsNone(result['t_rec_climb_s'])
        self.assertEqual(result['by_mode']['climb']['latest_tested_safe_s'], .06)
        self.assertIsNone(result['gap_sideways_s'])
        self.assertEqual(result['t_rec_sideways_s'], .02)


if __name__ == '__main__':
    unittest.main()
