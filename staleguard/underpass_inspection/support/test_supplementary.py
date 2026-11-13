"""Focused sampled-boundary/accuracy/readiness tests; no model or physical flights."""
import unittest
from staleguard.underpass_inspection.support.supplementary_analysis import profile,gap,accuracy,readiness,GRID
from staleguard.underpass_inspection.support.feasibility_sweep import key


class SupplementaryTests(unittest.TestCase):
    def test_censored_and_nonmonotonic_prefix(self):
        p=profile([True,False,True],[0.,.2,.4])
        self.assertEqual(p['first_loss_s'],.2);self.assertEqual(p['last_safe_prefix_s'],0.)
        self.assertEqual(p['last_tested_safe_s'],.4);self.assertTrue(p['non_monotonic'])
        p=profile([True]*19);self.assertIsNone(p['first_loss_s']);self.assertTrue(p['right_censored'])
        p=profile([False]+[True]*18);self.assertTrue(p['left_censored']);self.assertTrue(p['non_monotonic'])

    def test_gaps_use_first_loss_on_both_sides_not_last_safe(self):
        c=profile([True,True,False],[0.,.2,.4]);r=profile([True,False,False],[0.,.2,.4])
        self.assertEqual(gap(c,r),(-.2,'recovery_first'))
        self.assertEqual(gap(c,c),(0.,'equal_sampled_loss'))
        self.assertEqual(gap(r,c),(.2,'command_first'))
        self.assertEqual(gap(profile([True]*3,[0.,.2,.4]),r),(None,'recovery_first_command_censored'))

    def forecasts(self,command,recovery,runtime):
        return {'1':dict(computation_s=runtime,results=[dict(delay_s=d,command_safe=c,command_complete=c,recovery_safe=r) for d,c,r in zip(GRID,command,recovery)])}

    def test_confusion_denominators_and_censored_errors(self):
        actual=[True]*10+[False]*9;pred=actual.copy();pred[1]=False;pred[11]=True
        physical={1:{p:profile(actual) for p in ('command','completion','recovery')}}
        q=accuracy(self.forecasts(pred,pred,.1),physical)['command']
        self.assertEqual((q['tp'],q['tn'],q['fp'],q['fn']),(9,8,1,1))
        self.assertAlmostEqual(q['accuracy'],17/19);self.assertAlmostEqual(q['false_feasible_rate'],1/9)
        self.assertAlmostEqual(q['false_reject_rate'],1/10)
        q=accuracy(self.forecasts([True]*19,[True]*19,.1),physical)['command']
        self.assertEqual(q['boundary_comparisons'],0);self.assertIsNone(q['mae_s'])
        self.assertEqual(q['predicted_right_censored'],1)

    def test_rounded_readiness_and_unresolved_bracket(self):
        actual=[True]*10+[False]*9
        physical={1:{p:profile(actual) for p in ('command','completion','recovery')}}
        q=readiness(self.forecasts(actual,actual,.181),physical)
        self.assertEqual(q['counts']['at_or_after_first_unsafe_sample'],1)
        q=readiness(self.forecasts(actual,actual,.18),physical)
        self.assertEqual(q['counts']['by_last_safe_sample'],1)
        actual=[True]*11+[False]*8
        physical={1:{p:profile(actual) for p in ('command','completion','recovery')}}
        q=readiness(self.forecasts(actual,actual,.211),physical)
        self.assertEqual(q['counts']['within_unresolved_loss_bracket'],1)

    def test_candidate_alignment_uses_control_steps(self):
        self.assertEqual(key(dict(seed=1,mode='sideways',start_s=.3)),(1,'sideways',15))
        self.assertEqual(len(GRID),19)
        physical=set(GRID)&{0.,.04,.08,.12,.16,.2,.4,.6,.8,1.,1.2,1.6,2.}
        self.assertEqual(len(physical),10)
        self.assertEqual(400*(19-len(physical))+400*19,11200)


if __name__=='__main__':unittest.main()
