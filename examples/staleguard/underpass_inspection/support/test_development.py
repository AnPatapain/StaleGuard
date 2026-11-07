"""Small generation, calibration-causality and journal/analysis tests; toy data labelled."""
from dataclasses import asdict
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import unittest
import numpy as np

from model.common import PIDState
from underpass_inspection.scenario import Scenario,generate_scenario,DEVELOPMENT_RANGES,displacement
from underpass_inspection.calibration import calibrate
from underpass_inspection.support.deadline_sweep import read_journal,summarize


class DevelopmentTests(unittest.TestCase):
    def test_generation_deterministic_fixed_seed0_and_geometry_relations(self):
        baseline=json.loads(Path('underpass_inspection/results/development/pilot_ned_east_v2/summary.json').read_text())['scenario']
        self.assertEqual(json.loads(json.dumps(asdict(generate_scenario(0)))),baseline)
        for seed in range(1,21):
            s=generate_scenario(seed)
            self.assertEqual(s,generate_scenario(seed))
            self.assertAlmostEqual(s.deck_back-s.deck_front,2.5)
            self.assertAlmostEqual(s.inspection_lower[0]-s.deck_front,.05)
            self.assertAlmostEqual(s.inspection_upper[0]-s.deck_front,.35)
            self.assertAlmostEqual(s.approval_s*50,round(s.approval_s*50))
            for field,bounds in DEVELOPMENT_RANGES.items():
                self.assertTrue(bounds[0]<=getattr(s,field)<=bounds[1])
            a,_=displacement(np.arange(20)/50,s,'dive')
            b,_=displacement(np.arange(20)/50,Scenario(speed=s.speed),'dive')
            np.testing.assert_array_equal(a,b)

    def test_calibration_uses_only_immediate_flights_and_climb_is_not_acceptance(self):
        calls=[]
        def toy_execute(s,mode,start,**kwargs):
            calls.append((s.seed,mode,start))
            safe=mode in ('dive','sideways')
            row=dict(seed=s.seed,mode=mode,start_s=start,safe=safe,inspection_completed=mode=='dive',
                     safe_completion=mode=='dive',physics_sha256='toy physical digest',
                     first_violation=None if safe else dict(part='deck_front'))
            data={}
            if kwargs.get('capture_snapshot'):
                data['approval_snapshot']=dict(state=np.zeros(12),remaining_reference=np.zeros((3,12)),
                    pid_state=PIDState(np.zeros(3),np.zeros(3),np.zeros(3)),
                    approval_time=s.approval_s,horizon_steps=2,scenario=s)
            return row,data
        with TemporaryDirectory() as folder,patch('underpass_inspection.calibration.execute',toy_execute):
            data=calibrate([1],folder)
            before=len(calls)
            resumed=calibrate([1],folder,resume=True)
            self.assertEqual(len(calls),before)
            self.assertEqual(resumed['statistics'],data['statistics'])
        self.assertEqual(data['statistics'],dict(attempted=1,accepted=1,rejected=0))
        self.assertFalse(data['attempts'][0]['initial_climb_safe'])
        self.assertTrue(all(start==0. for _,_,start in calls))
        self.assertEqual([mode for _,mode,_ in calls],['nominal','dive','sideways','climb','dive'])

    def test_journal_repairs_only_unfinished_tail_and_rejects_duplicates(self):
        row=dict(seed=1,mode='dive',start_s=0.,safe=True,inspection_completed=True,safe_completion=True)
        line=(json.dumps(row)+'\n').encode()
        with TemporaryDirectory() as folder:
            p=Path(folder)/'executions.jsonl';p.write_bytes(line+b'{"seed":')
            self.assertEqual(read_journal(p,True),[row]);self.assertEqual(p.read_bytes(),line)
            p.write_bytes(line+line)
            with self.assertRaises(ValueError):
                read_journal(p,True)

    def test_metrics_keep_negative_undefined_and_nonmonotonic_cases(self):
        rows=[]
        for seed in (1,2):
            profiles=({'dive':[True,False,True],'sideways':[True,False,False],'climb':[True,True,False]}
                      if seed==1 else {m:[True]*3 for m in ('dive','sideways','climb')})
            for mode,flags in profiles.items():
                for i,safe in enumerate(flags):
                    rows.append(dict(seed=seed,mode=mode,start_s=i/50,safe=safe,
                        inspection_completed=True,safe_completion=safe,initiation_step=40+i,
                        first_violation=None if safe else dict(obstacle='deck',part='deck_front',time_flight_s=2.)))
        p=dict(scenarios=[dict(seed=1),dict(seed=2)],expected_flights=18,
               calibration_statistics=dict(attempted=2,accepted=2,rejected=0))
        result=summarize(rows,p)
        self.assertEqual(result['counts']['negative_side_grid_gaps'],1)
        self.assertEqual(result['counts']['undefined_side_grid_gaps'],1)
        self.assertEqual(result['counts']['non_monotonic_scenarios'],1)
        self.assertEqual(len(result['by_scenario'][0]['profiles']['dive']['safety_intervals']),3)


if __name__=='__main__':
    unittest.main()
