"""Small generation, calibration-causality and journal/analysis tests; toy data labelled."""
from dataclasses import asdict
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch
import unittest
import numpy as np

from staleguard.model.common import PIDState
from staleguard.underpass_inspection.scenario import Scenario,generate_scenario,DEVELOPMENT_RANGES,displacement,make_nominal
from staleguard.underpass_inspection.calibration import calibrate
from staleguard.underpass_inspection.storage import ROOT,load_calibrated_scenarios
from staleguard.underpass_inspection.support.deadline_sweep import read_journal,summarize


class DevelopmentTests(unittest.TestCase):
    def test_development_draws_match_all_frozen_attempts(self):
        data=json.loads((ROOT/'results/development/calibration/calibrated_scenarios.json').read_text())
        for attempt in data['attempts']:
            self.assertEqual(json.loads(json.dumps(asdict(generate_scenario(attempt['seed'])))),
                             attempt['scenario'])

    def test_evaluation_rejects_overlap_and_wrong_output_before_flying(self):
        with TemporaryDirectory() as folder,patch('staleguard.underpass_inspection.calibration.execute') as execute:
            out=Path(folder)/'calibration'
            for seeds,split in (([0],'evaluation'),([1],'evaluation'),([20],'evaluation'),
                                ([21],'development'),([21,21],'evaluation'),([True],'evaluation')):
                with self.assertRaises(ValueError):calibrate(seeds,out,split=split)
            with self.assertRaises(ValueError):
                calibrate([21],ROOT/'results/development/calibration',split='evaluation')
            with self.assertRaises(ValueError):
                calibrate([1],ROOT/'results/evaluation/calibration',split='development')
            execute.assert_not_called()
            self.assertFalse(out.exists())

    def test_evaluation_d0_screening_rejections_resume_and_snapshot_loading(self):
        # Synthetic outcomes only: never fly or score held-out scenarios in tests.
        calls=[]
        def toy_execute(s,mode,start,**kwargs):
            calls.append((s.seed,mode,start))
            safe=mode=='sideways' or (mode=='dive' and s.seed==21)
            reference=make_nominal(round((s.approval_s+s.observation_s)*50)+1,s)[round(s.approval_s*50):]
            state=reference[0].copy()
            row=dict(seed=s.seed,mode=mode,start_s=start,safe=safe,
                inspection_completed=safe and mode=='dive',safe_completion=safe and mode=='dive',
                physics_sha256='synthetic fixture, not PyBullet evidence',approval_state=state.tolist(),
                first_violation=None if safe else dict(part='pier' if mode=='dive' else 'deck_front'))
            arrays={}
            if kwargs.get('capture_snapshot'):
                arrays['approval_snapshot']=dict(state=state,remaining_reference=reference,
                    pid_state=PIDState(np.zeros(3),np.zeros(3),np.zeros(3)),
                    approval_time=s.approval_s,horizon_steps=len(reference)-1,scenario=s)
            return row,arrays
        with TemporaryDirectory() as folder,patch('staleguard.underpass_inspection.calibration.execute',toy_execute):
            out=Path(folder)
            partial=calibrate([21,22],out,max_scenarios=1,split='evaluation')
            self.assertEqual(partial['status'],'partial')
            with self.assertRaises(ValueError):load_calibrated_scenarios(out)
            with self.assertRaises(ValueError):
                calibrate([21,23],out,resume=True,split='evaluation')
            complete=calibrate([21,22],out,resume=True,split='evaluation')
            self.assertEqual(complete['status'],'complete')
            self.assertEqual(complete['dataset_split'],'evaluation')
            self.assertEqual(complete['statistics'],dict(attempted=2,accepted=1,rejected=1))
            self.assertFalse(complete['protocol']['replacement'])
            self.assertEqual(complete['protocol']['generation_ranges'],DEVELOPMENT_RANGES)
            self.assertEqual(complete['attempts'][1]['rejection_reasons'],
                             ['d0_unsafe:pier','d0_inspection_incomplete'])
            self.assertEqual([s.seed for s,_,_ in load_calibrated_scenarios(out)],[21])
            before=len(calls)
            calibrate([21,22],out,resume=True,split='evaluation')
            self.assertEqual(len(calls),before)
        self.assertEqual(len(calls),9)  # Four D=0 modes per attempt + accepted dive replay.
        self.assertTrue(all(start==0. for _,_,start in calls))
        self.assertEqual(generate_scenario(21),generate_scenario(21))
        for seed in (-1,True,1.5):
            with self.assertRaises(ValueError):generate_scenario(seed)

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
        with TemporaryDirectory() as folder,patch('staleguard.underpass_inspection.calibration.execute',toy_execute):
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
