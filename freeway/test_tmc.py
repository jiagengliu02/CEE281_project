"""Independent causal, conservation and actuator tests for the five-stage loop."""
import copy
import json
from pathlib import Path
import unittest

from .control import RampController
from .observation import BoundaryDetectors, RampObservation, CHANNELS
from .model import simulate

CONFIG = json.loads((Path(__file__).parent / 'configs/default.json').read_text())


class RampLoopTests(unittest.TestCase):
    def report(self, end=30, counts=None):
        counts = counts or dict(main_arrivals=20, main_entries=18, ramp_arrivals=8, ramp_entries=3, exits=10)
        return [{'sensor_id': k, 'interval_end_s': end, 'available_time_s': end,
                 'quality_flag': 'ok', 'cumulative_count': v} for k,v in counts.items()]

    def test_counter_conservation_estimate(self):
        pipeline = RampObservation(CONFIG['tmc'])
        self.assertIsNone(pipeline.estimate(0)['road_vehicles'])
        for r in self.report(): pipeline.receive(r,30)
        estimate = pipeline.estimate(30)
        self.assertEqual((estimate['road_vehicles'],estimate['ramp_queue'],estimate['main_queue']), (11,5,2))
        self.assertEqual(pipeline.forecast(30)['counts']['ramp_arrivals'],8)

    def test_missing_recovery_and_old_report(self):
        pipeline = RampObservation(CONFIG['tmc'])
        for r in self.report(): pipeline.receive(r,30)
        for key in CHANNELS:
            pipeline.receive({'sensor_id':key,'interval_end_s':60,'available_time_s':60,
                              'quality_flag':'missing','cumulative_count':None},60)
        self.assertEqual(pipeline.estimate(60)['quality_flag'],'missing')
        self.assertIsNone(pipeline.forecast(60))
        for r in self.report(90,dict(main_arrivals=60,main_entries=54,ramp_arrivals=24,ramp_entries=9,exits=30)):
            pipeline.receive(r,90)
        for r in self.report(): pipeline.receive(r,90)
        self.assertEqual(pipeline.estimate(90)['road_vehicles'],33)
        self.assertEqual(pipeline.estimate(90)['quality_flag'],'ok')
        self.assertEqual(pipeline.estimate(136)['quality_flag'],'stale')

    def test_delayed_data_cannot_be_used_early(self):
        detectors = BoundaryDetectors(30,[{'start_s':30,'end_s':60,'kind':'delay','delay_s':45}])
        detectors.cross('ramp_arrivals'); detectors.report(30)
        self.assertEqual(list(detectors.deliver(74.9)),[])
        rows=list(detectors.deliver(75))
        self.assertEqual(len(rows),5)
        pipeline=RampObservation(CONFIG['tmc'])
        with self.assertRaises(ValueError): pipeline.receive(rows[0],74)
        for row in rows: pipeline.receive(row,75)
        self.assertEqual(pipeline.estimate(75)['ramp_queue'],1)

    def test_forecast_changes_action_without_truth(self):
        ctrl=RampController(CONFIG['tmc'],'adaptive',12)
        estimate={'quality_flag':'ok','road_vehicles':40,'ramp_queue':0}
        low={'counts':dict(main_entries=10,exits=20,ramp_arrivals=0)}
        high={'counts':dict(main_entries=30,exits=10,ramp_arrivals=0)}
        first=ctrl.decide(30,estimate,low)
        second=ctrl.decide(30,estimate,high)
        self.assertLess(first['value'],second['value'])
        self.assertEqual(second['value'],20)
        ctrl.executed(30)
        self.assertEqual(ctrl.decide(31,estimate,low)['action'],'hold')
        self.assertEqual(ctrl.decide(34,estimate,low)['action'],'permit')
        estimate['quality_flag']='missing'
        command=ctrl.decide(50,estimate,None)
        self.assertTrue(command['fallback']); self.assertEqual(command['value'],12)

    def test_queue_relief_and_bounds(self):
        ctrl=RampController(CONFIG['tmc'],'adaptive',12)
        estimate={'quality_flag':'ok','road_vehicles':100,'ramp_queue':40}
        forecast={'counts':dict(main_entries=50,exits=10,ramp_arrivals=10)}
        command=ctrl.decide(60,estimate,forecast)
        self.assertEqual(command['reason'],'ramp_queue_relief')
        self.assertEqual(command['value'],3)

    def test_complete_loop_and_forecast_scoring(self):
        c=copy.deepcopy(CONFIG); c['duration_s']=120; c['tmc']['evaluation_start_s']=30
        c['demand']=[{'start_s':0,'main_vph':1200,'ramp_vph':300}]
        c['tmc']['faults']=[{'start_s':60,'end_s':90,'kind':'missing'}]
        run=simulate(c,'adaptive_fault')
        m=run['metrics']
        self.assertEqual(m['generated_vehicles'],m['exited_vehicles']+m['remaining_on_road']+m['remaining_main_queue']+m['remaining_ramp_queue'])
        self.assertEqual(m['safety_interventions'],0)
        self.assertGreater(m['fallback_time_s'],30)
        self.assertTrue(any(r['reason']=='forecast_road_inventory' for r in run['control_log']))
        self.assertTrue(any(r['executed_action']=='release' for r in run['control_log']))
        releases=[r for r in run['control_log'] if r['executed_action']=='release']
        for first,second in zip(releases,releases[1:]):
            self.assertGreaterEqual(second['apply_time_s']-first['apply_time_s']+1e-8,second['value'])
        self.assertTrue(all(3<=r['value']<=20 for r in run['control_log']))
        rows=[r for r in run['forecasts'] if r['evaluation_period']]
        self.assertTrue(rows)
        for row in rows:
            self.assertEqual(row['target_time_s']-row['issue_time_s'],30)
            self.assertAlmostEqual(row['absolute_error'],abs(row['actual_value']-row['predicted_value']))
        for key in CHANNELS:
            samples={method:{(r['issue_time_s'],r['target_time_s']) for r in rows
                             if r['segment_id']==key and r['method']==method}
                     for method in ('ema','moving_average')}
            self.assertEqual(samples['ema'],samples['moving_average'])
        # Audit each target against observed, cumulative counters, not predictor logic.
        obs={(r['sensor_id'],r['interval_end_s']):r['cumulative_count']
             for r in run['boundary_observations'] if r['quality_flag']=='ok'}
        for row in rows:
            key=row['segment_id']; start=obs.get((key,row['issue_time_s'])); end=obs.get((key,row['target_time_s']))
            if start is not None and end is not None:self.assertEqual(row['actual_value'],end-start)

    def test_future_demand_cannot_change_past_commands(self):
        c=copy.deepcopy(CONFIG); c['duration_s']=90
        c['demand']=[{'start_s':0,'main_vph':1200,'ramp_vph':300},
                     {'start_s':60,'main_vph':0,'ramp_vph':0}]
        changed=copy.deepcopy(c); changed['demand'][1].update(main_vph=4000,ramp_vph=900)
        first=simulate(c,'adaptive'); second=simulate(changed,'adaptive')
        self.assertEqual([r for r in first['control_log'] if r['issue_time_s']<60],
                         [r for r in second['control_log'] if r['issue_time_s']<60])

    def test_zero_demand(self):
        c=copy.deepcopy(CONFIG); c['duration_s']=60
        c['demand']=[{'start_s':0,'main_vph':0,'ramp_vph':0}]
        run=simulate(c,'adaptive')
        self.assertEqual(run['metrics']['total_delay_vehicle_s'],0)
        self.assertFalse(any(r['executed_action']=='release' for r in run['control_log']))
        self.assertTrue(all(r['predicted_value']==0 for r in run['forecasts']))


def verify_tmc():
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(RampLoopTests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    if not result.wasSuccessful(): raise AssertionError('Ramp loop checks failed')
    return {'tests':result.testsRun,'passed':True}
