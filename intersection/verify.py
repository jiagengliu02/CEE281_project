"""Independent signal audits, analytical queue case, causality and timestep checks."""
from bisect import bisect_right
import copy
import json
from pathlib import Path
import unittest

from .control import SignalController
from .detector import APPROACHES
from .model import make_arrivals, simulate, validate
from .observation import ObservationPipeline

ROOT = Path(__file__).resolve().parent
CONFIG = json.loads((ROOT / 'configs/default.json').read_text())
REPORT = {}


def small_config():
    c = copy.deepcopy(CONFIG)
    c.update(duration_s=120, evaluation_start_s=30)
    c['scenarios'] = {'test': {'label': 'test', 'demand': [
        {'start_s': 0, 'rates_vph': {a: 0 for a in APPROACHES}}]}}
    return c


def audit(test, c, run):
    """Independent checks of one run, re-derived from its event and control logs."""
    m = run['metrics']
    test.assertEqual(m['generated_vehicles'],
                     m['exited_vehicles'] + m['residual_queue_veh'] + m['residual_external_veh'])
    transitions = [r for i, r in enumerate(run['control_log']) if i == 0 or r['action'] == 'transition']
    audit_phase_sequence(test, c, transitions)
    audit_departures(test, c, run, transitions)
    audit_fifo(test, run['events'])
    audit_detector_counts(test, run)
    audit_delay(test, c, run)


def audit_phase_sequence(test, c, transitions):
    """green -> yellow -> all_red -> other green, with legal durations."""
    for old, new in zip(transitions, transitions[1:]):
        group, stage = old['value'].split('_', 1)
        elapsed = new['apply_time_s'] - old['apply_time_s']
        if stage == 'green':
            test.assertGreaterEqual(elapsed, c['min_green_s'])
            test.assertLessEqual(elapsed, c['max_green_s'])
            test.assertEqual(new['value'], group + '_yellow')
        elif stage == 'yellow':
            test.assertEqual(elapsed, c['yellow_s'])
            test.assertEqual(new['value'], group + '_all_red')
        else:
            test.assertEqual(elapsed, c['all_red_s'])
            test.assertEqual(new['value'], ('EW' if group == 'NS' else 'NS') + '_green')


def audit_departures(test, c, run, transitions):
    """Departures only on green, after start-up loss, at saturation headway, once each."""
    starts = [r['apply_time_s'] for r in transitions]
    last_exit, departed_ids = {}, set()
    for e in run['events']:
        if e['event'] != 'departure':
            continue
        phase = transitions[bisect_right(starts, e['time_s']) - 1]
        group, stage = phase['value'].split('_', 1)
        test.assertEqual(stage, 'green')
        test.assertIn(e['approach'], group)
        test.assertGreaterEqual(e['time_s'] - phase['apply_time_s'], c['startup_lost_s'])
        if e['approach'] in last_exit:
            test.assertGreaterEqual(e['time_s'] - last_exit[e['approach']], c['saturation_headway_s'])
        last_exit[e['approach']] = e['time_s']
        test.assertNotIn(e['vehicle_id'], departed_ids)
        departed_ids.add(e['vehicle_id'])
    test.assertEqual(len(departed_ids), run['metrics']['exited_vehicles'])


def audit_fifo(test, events):
    for a in APPROACHES:
        arrived = [e['vehicle_id'] for e in events if e['approach'] == a and e['event'] == 'arrival']
        departed = [e['vehicle_id'] for e in events if e['approach'] == a and e['event'] == 'departure']
        test.assertEqual(departed, arrived[:len(departed)])


def audit_detector_counts(test, run):
    """Every non-missing report matches a recount of the event log."""
    events = run['events']
    for o in run['observations']:
        if o['quality_flag'] == 'missing':
            test.assertIsNone(o['count'])
            test.assertIsNone(o['cumulative_entered'])
            continue
        for event, field, cumulative in [('entry', 'count', 'cumulative_entered'),
                                         ('departure', 'departure_count', 'cumulative_departed')]:
            relevant = [e for e in events if e['event'] == event and e['approach'] == o['approach']]
            test.assertEqual(o[field], sum(o['interval_start_s'] <= e['time_s'] < o['interval_end_s']
                                           for e in relevant))
            test.assertEqual(o[cumulative], sum(e['time_s'] < o['interval_end_s'] for e in relevant))


def audit_delay(test, c, run):
    m = run['metrics']
    for v in run['vehicles']:
        if v['departure_time_s'] is not None:
            test.assertGreaterEqual(v['departure_time_s'], v['entered_time_s'])
        if v['entered_time_s'] is not None:
            test.assertGreaterEqual(v['entered_time_s'], v['arrival_time_s'])
    independent_delay = sum(
        (v['departure_time_s'] if v['departure_time_s'] is not None else c['duration_s']) - v['arrival_time_s']
        for v in run['vehicles'])
    test.assertAlmostEqual(independent_delay, m['total_delay_vehicle_s'], places=6)
    # Raw Poisson times are rounded up to the simulation grid; this is the only
    # difference from the queue integral. End-of-run unfinished arrivals are included.
    test.assertGreaterEqual(m['arrival_grid_correction_vehicle_s'], -1e-6)
    test.assertLessEqual(m['arrival_grid_correction_vehicle_s'], m['generated_vehicles'] * c['dt_s'] + 1e-6)


class SignalTests(unittest.TestCase):
    def test_all_scenarios_and_controllers(self):
        report = {}
        for scenario in CONFIG['scenarios']:
            arrivals = make_arrivals(CONFIG, CONFIG['scenarios'][scenario], 11)
            hashes = []
            for mode in ('fixed', 'adaptive', 'adaptive_no_hysteresis'):
                run = simulate(CONFIG, scenario, mode, 11, arrivals)
                audit(self, CONFIG, run)
                hashes.append(run['metrics']['demand_id'])
                report[scenario + '/' + mode] = run['metrics']
            self.assertEqual(len(set(hashes)), 1)
        REPORT['scenario_audits'] = report

    def test_analytical_saturated_queue(self):
        c = small_config()
        c.update(duration_s=40, evaluation_start_s=0, forecast_horizon_s=5)
        arrivals = [{'arrival_time_s': 0, 'approach': 'N', 'vehicle_id': i} for i in range(50)]
        run = simulate(c, 'test', 'fixed', 0, arrivals)
        audit(self, c, run)
        self.assertEqual(run['metrics']['exited_vehicles'], 14)  # t=2,4,...,28
        self.assertAlmostEqual(run['metrics']['total_delay_vehicle_s'], 1650)
        self.assertGreater(run['metrics']['boundary_delay_vehicle_s'], 0)
        REPORT['analytical_case'] = {'expected_exits': 14, 'expected_delay_vehicle_s': 1650, 'passed': True}

    def test_zero_demand(self):
        run = simulate(small_config(), 'test', 'adaptive', 1)
        self.assertEqual(run['metrics']['total_delay_vehicle_s'], 0)
        self.assertEqual(run['metrics']['exited_vehicles'], 0)
        self.assertTrue(all(o['mean_speed_mps'] is None for o in run['observations']))

    def test_finite_pulse_clears(self):
        c = small_config()
        arrivals = [{'arrival_time_s': 0, 'approach': a, 'vehicle_id': j*5+i}
                    for j, a in enumerate(sorted(APPROACHES)) for i in range(5)]
        run = simulate(c, 'test', 'adaptive', 0, arrivals)
        self.assertEqual(run['metrics']['exited_vehicles'], 20)
        self.assertEqual(run['metrics']['residual_queue_veh'], 0)

    def test_delayed_data_is_not_visible_early(self):
        c = small_config()
        c['scenarios']['test']['demand'][0]['rates_vph']['N'] = 600
        c['scenarios']['test']['faults'] = [
            {'approach': 'N', 'start_s': 0, 'end_s': 50, 'kind': 'delay', 'delay_s': 20}]
        run = simulate(c, 'test', 'adaptive', 11)
        for f in run['frames']:
            if f['time_s'] < 25:
                self.assertEqual(f['quality']['N'], 'unavailable')
                self.assertEqual(f['estimates']['N'], 0)
        self.assertTrue(any(f['fallback'] for f in run['frames'] if f['time_s'] > 25))
        self.assertFalse(run['frames'][-1]['fallback'])

    def test_cumulative_recovery_and_out_of_order(self):
        p = ObservationPipeline(CONFIG)
        def row(end, entered, departed, flag='ok'):
            return {'approach': 'N', 'interval_end_s': end, 'cumulative_entered': entered,
                    'cumulative_departed': departed, 'quality_flag': flag}
        p.receive(row(5, 4, 1), 5)
        self.assertEqual(p.estimates(5)['N']['estimated_queue_veh'], 3)
        p.receive(row(10, None, None, 'missing'), 10)
        self.assertEqual(p.estimates(10)['N']['quality_flag'], 'missing')
        p.receive(row(15, 8, 3), 15)
        self.assertEqual(p.estimates(15)['N']['estimated_queue_veh'], 5)
        p.receive(row(5, 4, 1), 25)
        self.assertEqual(p.estimates(25)['N']['estimated_queue_veh'], 5)
        self.assertEqual(p.estimates(30)['N']['quality_flag'], 'stale')

    def test_forecast_horizon_and_comparable_samples(self):
        run = simulate(CONFIG, 'detector_fault', 'adaptive', 11)
        keys = {}
        for method in ('ema', 'moving_average'):
            rows = [r for r in run['forecasts'] if r['method'] == method and r['evaluation_period']]
            keys[method] = [(r['issue_time_s'], r['target_time_s'], r['segment_id']) for r in rows]
            for r in rows:
                self.assertEqual(r['target_time_s'] - r['issue_time_s'], CONFIG['forecast_horizon_s'])
                actual = sum(e['event'] == 'entry' and e['approach'] == r['segment_id']
                             and r['issue_time_s'] <= e['time_s'] < r['target_time_s'] for e in run['events'])
                self.assertEqual(actual, r['actual_value'])
        self.assertEqual(keys['ema'], keys['moving_average'])

    def test_controller_requires_only_estimates(self):
        estimates = {a: {'quality_flag': 'ok', 'estimated_queue_veh': 0} for a in APPROACHES}
        controller = SignalController(CONFIG, 'adaptive')
        estimates['E']['estimated_queue_veh'] = 20
        self.assertEqual(controller.update(5, estimates)['value'], 'NS_green')
        self.assertEqual(controller.update(10, estimates)['value'], 'NS_yellow')
        self.assertEqual(controller.update(13, estimates)['value'], 'NS_all_red')
        self.assertEqual(controller.update(14, estimates)['value'], 'EW_green')

    def test_future_demand_cannot_change_past_decisions(self):
        c1, c2 = small_config(), small_config()
        c1['scenarios']['test']['demand'][0]['rates_vph'] = {a: 400 for a in APPROACHES}
        c2['scenarios'] = copy.deepcopy(c1['scenarios'])
        c2['scenarios']['test']['demand'].append({'start_s': 60, 'rates_vph': {a: 1800 for a in APPROACHES}})
        r1, r2 = simulate(c1, 'test', 'adaptive', 11), simulate(c2, 'test', 'adaptive', 11)
        self.assertEqual([r for r in r1['control_log'] if r['issue_time_s'] < 60],
                         [r for r in r2['control_log'] if r['issue_time_s'] < 60])

    def test_time_step_sensitivity(self):
        fine = copy.deepcopy(CONFIG)
        fine['dt_s'] /= 2
        result = {}
        for mode in ('fixed', 'adaptive'):
            coarse = simulate(CONFIG, 'directional_shift', mode, 11)
            refined = simulate(fine, 'directional_shift', mode, 11)
            self.assertEqual(coarse['metrics']['demand_id'], refined['metrics']['demand_id'])
            audit(self, fine, refined)
            result[mode] = {k: {'dt_0.5': coarse['metrics'][k], 'dt_0.25': refined['metrics'][k]}
                            for k in ('total_delay_vehicle_s', 'exited_vehicles', 'residual_queue_veh')}
        REPORT['time_step_sensitivity'] = result

    def test_invalid_config_rejected(self):
        for key, value in [('dt_s', 0), ('storage_veh_per_approach', 0), ('yellow_s', -1), ('min_green_s', 60)]:
            c = copy.deepcopy(CONFIG)
            c[key] = value
            with self.assertRaises(ValueError):
                validate(c)


def verify():
    """Run signal checks and write verification.json without import side effects."""
    REPORT.clear()
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(SignalTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    REPORT['tests_run'] = result.testsRun
    REPORT['passed'] = result.wasSuccessful()
    out = ROOT.parent / 'results/intersection'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'verification.json').write_text(json.dumps(REPORT, indent=2))
    return result.wasSuccessful()
