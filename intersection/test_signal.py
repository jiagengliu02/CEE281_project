"""Small independent examples for the observation pipeline and the signal controllers."""
import copy
import json
from pathlib import Path
import unittest

from .control import SignalController, webster_plan
from .model import make_arrivals, simulate
from .observation import APPROACHES, ApproachDetectors, SignalObservation

CONFIG = json.loads((Path(__file__).parent / 'configs/default.json').read_text())


def reports(end, adv, stop, approach='N', available=None, quality='ok'):
    rows = []
    for sensor, cum in (('advance', adv), ('stopline', stop)):
        rows.append({'sensor_id': f'{approach}_{sensor}', 'approach': approach, 'interval_end_s': end,
                     'available_time_s': end if available is None else available,
                     'count': None if quality == 'missing' else cum, 'cumulative_count': None if quality == 'missing' else cum,
                     'quality_flag': quality})
    return rows


def healthy(pipeline, end, adv=10, stop=6):
    for a in APPROACHES:
        for row in reports(end, adv, stop, a):
            pipeline.receive(row, end)


def estimates(queues, recent=1, quality='ok'):
    return {a: {'estimated_queue_veh': q, 'recent_arrivals_veh': recent, 'quality_flag': quality}
            for a, q in zip(APPROACHES, queues)}


def forecasts(values):
    return {a: {'predicted_value': v} for a, v in zip(APPROACHES, values)}


class ObservationTests(unittest.TestCase):
    def test_queue_is_advance_minus_stopline(self):
        pipeline = SignalObservation(CONFIG['control'])
        self.assertEqual(pipeline.estimate(0, 'N')['quality_flag'], 'unavailable')
        healthy(pipeline, 5, adv=10, stop=6)
        e = pipeline.estimate(5, 'N')
        self.assertEqual((e['estimated_queue_veh'], e['quality_flag']), (4, 'ok'))
        self.assertAlmostEqual(pipeline.forecast(5, 'N')['predicted_value'], 10 / 5 * 30)

    def test_missing_then_recovery_and_staleness(self):
        pipeline = SignalObservation(CONFIG['control'])
        healthy(pipeline, 5)
        for row in reports(10, None, None, 'N', quality='missing'):
            pipeline.receive(row, 10)
        self.assertEqual(pipeline.estimate(10, 'N')['quality_flag'], 'missing')
        self.assertEqual(pipeline.estimate(10, 'S')['quality_flag'], 'ok')
        self.assertIsNone(pipeline.forecast(10, 'N'))
        for row in reports(15, 14, 9, 'N'):
            pipeline.receive(row, 15)
        e = pipeline.estimate(15, 'N')
        self.assertEqual((e['estimated_queue_veh'], e['quality_flag']), (5, 'ok'))
        self.assertEqual(pipeline.estimate(15 + CONFIG['control']['stale_after_s'] + 1, 'N')['quality_flag'], 'stale')

    def test_delayed_report_cannot_be_used_early_and_old_never_overwrites(self):
        detectors = ApproachDetectors(5, {'advance': 200, 'stopline': 300},
                                      [{'approach': 'E', 'start_s': 5, 'end_s': 10, 'kind': 'delay', 'delay_s': 20}])
        detectors.cross('E', 'advance', 12.0)
        detectors.report(5)
        delivered = list(detectors.deliver(5))
        self.assertEqual({r['approach'] for r in delivered}, {'N', 'S', 'W'})
        late = list(detectors.deliver(25))
        self.assertEqual([r['sensor_id'] for r in late], ['E_advance', 'E_stopline'])
        pipeline = SignalObservation(CONFIG['control'])
        with self.assertRaises(ValueError):
            pipeline.receive(late[0], 24)
        for row in reports(10, 5, 2, 'E'):
            pipeline.receive(row, 10)
        for row in late:
            self.assertFalse(pipeline.receive(row, 25))
        self.assertEqual(pipeline.estimate(25, 'E')['estimated_queue_veh'], 3)

    def test_arrival_table_is_controller_independent(self):
        a = make_arrivals(CONFIG, CONFIG['scenarios']['balanced'])
        b = make_arrivals(CONFIG, CONFIG['scenarios']['balanced'])
        self.assertEqual(a['demand_id'], b['demand_id'])
        self.assertEqual([r['vehicle_id'] for r in a['rows']], list(range(len(a['rows']))))


class ControllerTests(unittest.TestCase):
    def advance(self, ctrl, t0, t1, est, fc, dt=0.1):
        last = None
        t = t0
        while t < t1 - 1e-9:
            last = ctrl.update(round(t, 6), est, fc)
            t += dt
        return last

    def test_webster_plan_by_hand(self):
        plan = webster_plan(CONFIG)
        c = CONFIG
        y = 450 / c['saturation_flow_vph']
        lost = c['startup_lost_s'] + c['yellow_s'] + c['all_red_s'] - c['yellow_used_s']
        cycle = (1.5 * 2 * lost + 5) / (1 - 2 * y)
        green = (cycle - 2 * lost) / 2 + c['startup_lost_s'] - c['yellow_used_s']
        self.assertAlmostEqual(plan['optimal_cycle_s'], cycle, places=1)
        self.assertAlmostEqual(plan['green_s']['NS'], round(green, 1), places=1)
        unbalanced = copy.deepcopy(CONFIG)
        unbalanced['fixed_plan']['design_demand_vph'].update(N=700, S=700, E=300, W=300)
        p = webster_plan(unbalanced)
        self.assertGreater(p['green_s']['NS'], p['green_s']['EW'])

    def test_minimum_green_clearance_and_fixed_durations(self):
        ctrl = SignalController(CONFIG, 'fixed')
        est, fc = estimates([5, 5, 5, 5]), forecasts([1, 1, 1, 1])
        self.assertEqual(ctrl.update(0, est, fc)['reason'], 'minimum_green')
        green = CONFIG['fixed_plan'] and ctrl.plan['green_s']['NS']
        cmd = self.advance(ctrl, 0, green, est, fc)
        self.assertEqual(cmd['value'], 'NS_green')
        cmd = ctrl.update(green, est, fc)
        self.assertEqual((cmd['action'], cmd['value']), ('transition', 'NS_yellow'))
        cmd = self.advance(ctrl, green, green + CONFIG['yellow_s'], est, fc)
        self.assertEqual(cmd['value'], 'NS_yellow')
        cmd = ctrl.update(green + CONFIG['yellow_s'], est, fc)
        self.assertEqual(cmd['value'], 'NS_all_red')
        cmd = ctrl.update(green + CONFIG['yellow_s'] + CONFIG['all_red_s'], est, fc)
        self.assertEqual((cmd['value'], cmd['reason']), ('EW_green', 'clearance_complete'))

    def test_actuated_gap_out_max_out_and_rest(self):
        ctrl = SignalController(CONFIG, 'actuated')
        busy = estimates([3, 3, 3, 3], recent=1)
        fc = forecasts([2, 2, 2, 2])
        self.advance(ctrl, 0, CONFIG['min_green_s'], busy, fc)
        self.assertEqual(ctrl.update(CONFIG['min_green_s'], busy, fc)['reason'], 'extend_green')
        quiet_current = {**busy, 'N': {**busy['N'], 'recent_arrivals_veh': 0}, 'S': {**busy['S'], 'recent_arrivals_veh': 0}}
        cmd = ctrl.update(CONFIG['min_green_s'] + 0.1, quiet_current, fc)
        self.assertEqual((cmd['reason'], cmd['value']), ('gap_out', 'NS_yellow'))
        ctrl = SignalController(CONFIG, 'actuated')
        cmd = self.advance(ctrl, 0, CONFIG['max_green_s'], busy, fc)
        self.assertEqual(cmd['reason'], 'extend_green')
        self.assertEqual(ctrl.update(CONFIG['max_green_s'], busy, fc)['reason'], 'max_out')
        ctrl = SignalController(CONFIG, 'actuated')
        no_call = estimates([3, 3, 0, 0], recent=0)
        no_call['N']['recent_arrivals_veh'] = no_call['S']['recent_arrivals_veh'] = 1
        cmd = self.advance(ctrl, 0, CONFIG['max_green_s'] + 10, no_call, fc)
        self.assertEqual((cmd['reason'], cmd['value']), ('rest_no_opposing_call', 'NS_green'))

    def test_adaptive_forecast_changes_the_decision(self):
        t = CONFIG['min_green_s']
        est = estimates([4, 4, 4, 4])
        same = forecasts([2, 2, 2, 2])
        opposing_surge = forecasts([0, 0, 4, 4])
        with_forecast = SignalController(CONFIG, 'adaptive')
        self.advance(with_forecast, 0, t, est, same)
        self.assertEqual(with_forecast.update(t, est, same)['reason'], 'hold_green')
        self.assertEqual(with_forecast.update(t + 0.1, est, opposing_surge)['reason'], 'opposing_pressure_larger')
        without = SignalController(CONFIG, 'adaptive_no_forecast')
        self.advance(without, 0, t, est, same)
        self.assertEqual(without.update(t, est, opposing_surge)['reason'], 'hold_green')
        self.assertEqual(without.update(t + 0.1, estimates([0, 0, 3, 3], recent=0), opposing_surge)['reason'],
                         'served_approaches_empty')

    def test_fallback_uses_fixed_plan_without_skipping_clearance(self):
        for mode in ('actuated', 'adaptive', 'adaptive_no_forecast'):
            ctrl = SignalController(CONFIG, mode)
            bad = estimates([None, None, None, None], recent=None, quality='unavailable')
            none = {a: None for a in APPROACHES}
            cmd = self.advance(ctrl, 0, CONFIG['min_green_s'] + 0.1, bad, none)
            self.assertTrue(cmd['fallback'])
            self.assertEqual(cmd['reason'], 'fallback_fixed')
            stale = estimates([1, 1, 1, 1], quality='stale')
            cmd = self.advance(ctrl, CONFIG['min_green_s'] + 0.1, ctrl.plan['green_s']['NS'] + 0.1, stale, none)
            self.assertEqual(cmd['value'], 'NS_yellow')
            cmd = self.advance(ctrl, ctrl.plan['green_s']['NS'] + 0.1, ctrl.plan['green_s']['NS'] + CONFIG['yellow_s'], stale, none)
            self.assertEqual(cmd['value'], 'NS_yellow')


class LoopTests(unittest.TestCase):
    def short(self, scenario_demand, duration=120, faults=None):
        c = copy.deepcopy(CONFIG)
        c['duration_s'] = duration
        c['control']['evaluation_start_s'] = 30
        c['scenarios'] = {'t': {'label': 't', 'demand': scenario_demand}}
        if faults:
            c['scenarios']['t']['faults'] = faults
        return c

    def test_complete_loop_forecast_scoring_and_fallback(self):
        c = self.short([{'start_s': 0, 'rates_vph': {'N': 500, 'S': 500, 'E': 500, 'W': 500}}], 180,
                       faults=[{'approach': 'N', 'start_s': 60, 'end_s': 90, 'kind': 'missing'}])
        run = simulate(c, 't', 'adaptive')
        m = run['metrics']
        self.assertEqual(m['generated_vehicles'], m['exited_vehicles'] + m['remaining_on_road'] + m['remaining_boundary_queue'])
        self.assertEqual((m['red_light_runs'], m['conflict_box_steps_with_both_groups'], m['safety_interventions']), (0, 0, 0))
        self.assertGreaterEqual(m['fallback_time_s'], 30)
        self.assertTrue(any(r['reason'] in ('opposing_pressure_larger', 'served_approaches_empty', 'max_out')
                            for r in run['control_log']))
        rows = [r for r in run['forecasts'] if r['evaluation_period']]
        self.assertTrue(rows)
        obs = {(r['sensor_id'], r['interval_end_s']): r['cumulative_count']
               for r in run['observations'] if r['quality_flag'] != 'missing'}
        for row in rows:
            self.assertEqual(row['target_time_s'] - row['issue_time_s'], c['control']['forecast_horizon_s'])
            key = f"{row['segment_id']}_advance"
            start, end = obs.get((key, row['issue_time_s'])), obs.get((key, row['target_time_s']))
            if start is not None and end is not None:
                self.assertEqual(row['actual_value'], end - start)
        for a in APPROACHES:
            samples = {method: {(r['issue_time_s'], r['target_time_s']) for r in rows
                                if r['segment_id'] == a and r['method'] == method}
                       for method in ('ema', 'moving_average')}
            self.assertEqual(samples['ema'], samples['moving_average'])

    def test_future_demand_cannot_change_past_commands(self):
        base = [{'start_s': 0, 'rates_vph': {'N': 500, 'S': 500, 'E': 500, 'W': 500}},
                {'start_s': 90, 'rates_vph': {'N': 0, 'S': 0, 'E': 0, 'W': 0}}]
        changed = copy.deepcopy(base)
        changed[1]['rates_vph'] = {'N': 1500, 'S': 1500, 'E': 1500, 'W': 1500}
        first = simulate(self.short(base), 't', 'adaptive')
        second = simulate(self.short(changed), 't', 'adaptive')
        self.assertEqual([r for r in first['control_log'] if r['issue_time_s'] < 90],
                         [r for r in second['control_log'] if r['issue_time_s'] < 90])
        # Identical arrival streams before the change: same first vehicles on each approach.
        self.assertEqual([r for r in first['arrivals'] if r['time_s'] < 90],
                         [r for r in second['arrivals'] if r['time_s'] < 90])

    def test_zero_demand(self):
        c = self.short([{'start_s': 0, 'rates_vph': {'N': 0, 'S': 0, 'E': 0, 'W': 0}}], 120)
        fixed = simulate(c, 't', 'fixed')['metrics']
        self.assertEqual((fixed['generated_vehicles'], fixed['total_delay_vehicle_s']), (0, 0))
        self.assertGreater(fixed['green_phases'], 1)
        actuated = simulate(c, 't', 'actuated')
        self.assertEqual(actuated['metrics']['green_phases'], 1)     # rests in green without calls
        self.assertTrue(all(o['mean_speed_mps'] is None for o in actuated['observations']))


def verify_loop():
    suite = unittest.defaultTestLoader.loadTestsFromModule(__import__(__name__, fromlist=['*']))
    result = unittest.TextTestRunner(verbosity=1).run(suite)
    if not result.wasSuccessful():
        raise AssertionError('Signal loop checks failed')
    return {'tests': result.testsRun, 'passed': True}
