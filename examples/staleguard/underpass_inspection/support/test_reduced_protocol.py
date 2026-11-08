"""Reduced protocol tests; no held-out policy outcomes are read or generated."""
from dataclasses import dataclass
import json
from pathlib import Path
import unittest
import numpy as np
from underpass_inspection.baseline_benchmark.run import reduced_cases,REDUCED_DELAYS,REDUCED_SAMPLE_SEED
from underpass_inspection.baselines import staleguard
from underpass_inspection.storage import ROOT


@dataclass
class SeedOnly:
    seed:int


class ReducedProtocolTests(unittest.TestCase):
    def test_exact_grid_and_pair_counts(self):
        self.assertEqual(REDUCED_DELAYS,(0.,.04,.08,.12,.16,.20,.40,.60,.80,1.,1.20,1.60,2.))
        self.assertEqual(100*len(REDUCED_DELAYS)*7,9100)
        self.assertEqual(sum(d<=1. for d in REDUCED_DELAYS),10)
        self.assertEqual(sum(d>1. for d in REDUCED_DELAYS),3)
        self.assertEqual(len(staleguard.TTL_GRID),19)
        self.assertTrue(all(d in staleguard.TTL_GRID for d in REDUCED_DELAYS if d<=1.))

    def test_sample_is_uniform_without_replacement_deterministic_and_order_independent(self):
        cases=[(SeedOnly(i),None,None) for i in range(1001,1866)]
        actual=[s.seed for s,_,_ in reduced_cases(cases)]
        expected=sorted(np.random.default_rng(20261108).choice(list(range(1001,1866)),100,replace=False).tolist())
        self.assertEqual(REDUCED_SAMPLE_SEED,20261108)
        self.assertEqual(actual,expected);self.assertEqual(len(set(actual)),100)
        self.assertEqual(actual,[s.seed for s,_,_ in reduced_cases(cases[::-1])])
        self.assertNotEqual(actual,list(range(1001,1101)))
        with self.assertRaises(ValueError):reduced_cases(cases[:100])

    def test_actual_sample_uses_only_frozen_accepted_ids(self):
        # This reads calibration definitions only, never delayed/policy outcomes.
        data=json.loads((ROOT/'results/evaluation/calibration/calibrated_scenarios.json').read_text())
        ids=sorted(e['seed'] for e in data['scenarios'])
        sample=[s.seed for s,_,_ in reduced_cases([(SeedOnly(i),None,None) for i in ids])]
        expected=sorted(np.random.default_rng(20261108).choice(ids,100,replace=False).tolist())
        self.assertEqual(sample,expected);self.assertTrue(set(sample)<=set(ids))
        self.assertFalse(set(sample)&set(range(21)))


if __name__=='__main__':unittest.main()
