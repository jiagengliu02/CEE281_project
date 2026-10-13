"""Finite-storage FIFO point queues with explicit signal-dependent stop-line service.

    No RGB, spatial trajectories, turning movements or pedestrian model. Demand is
    generated before control and shared exactly between paired experiments.
"""
from bisect import bisect_left
from collections import deque
import hashlib
import heapq
import json
import math
import random

from .control import SignalController
from .observation import APPROACHES, ObservationPipeline

MODEL_VERSION = 'signal-point-queue-1.0'
CONTROLLER_VERSION = 'bounded-queue-response-1.0'


def validate(c):
    for key in ('dt_s', 'duration_s', 'frame_interval_s', 'detector_interval_s',
                'saturation_headway_s', 'yellow_s', 'all_red_s', 'min_green_s',
                'max_green_s', 'forecast_horizon_s', 'stale_after_s'):
        if not isinstance(c[key], (int, float)) or not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    for key in ('duration_s', 'frame_interval_s', 'detector_interval_s',
                'saturation_headway_s', 'startup_lost_s', 'yellow_s', 'all_red_s',
                'min_green_s', 'max_green_s'):
        if c[key] < 0 or abs(c[key] / c['dt_s'] - round(c[key] / c['dt_s'])) > 1e-8:
            raise ValueError(f'{key} must be a nonnegative multiple of dt_s')
    if c['duration_s'] % c['detector_interval_s'] or c['duration_s'] % c['frame_interval_s']:
        raise ValueError('duration must cover complete detector and frame intervals')
    if not isinstance(c['storage_veh_per_approach'], int) or c['storage_veh_per_approach'] < 1:
        raise ValueError('storage must be a positive integer')
    if not 0 < c['ema_alpha'] <= 1 or c['moving_average_intervals'] < 1:
        raise ValueError('invalid forecast parameters')
    if not 0 <= c['evaluation_start_s'] < c['duration_s'] - c['forecast_horizon_s']:
        raise ValueError('need a nonempty forecast evaluation period')
    if c['min_green_s'] > c['max_green_s'] or c['switch_threshold_veh'] < 0:
        raise ValueError('invalid controller bounds')
    for green in c['fixed_green_s'].values():
        if not c['min_green_s'] <= green <= c['max_green_s'] or abs(green/c['dt_s'] - round(green/c['dt_s'])) > 1e-8:
            raise ValueError('fixed greens must respect bounds and time grid')
    if set(c['fixed_green_s']) != {'NS', 'EW'}:
        raise ValueError('fixed plan must specify NS and EW')
    for scenario in c['scenarios'].values():
        starts = [d['start_s'] for d in scenario['demand']]
        if starts[0] != 0 or starts != sorted(set(starts)) or starts[-1] >= c['duration_s']:
            raise ValueError('demand starts must be sorted, unique, starting at zero and within duration')
        for d in scenario['demand']:
            if set(d['rates_vph']) != set(APPROACHES) or any(
                    not math.isfinite(v) or v < 0 for v in d['rates_vph'].values()):
                raise ValueError('nonnegative finite demand for all four approaches required')
        for f in scenario.get('faults', []):
            if f['approach'] not in APPROACHES or f['kind'] not in ('missing', 'delay'):
                raise ValueError('unsupported detector fault')
            if not 0 <= f['start_s'] < f['end_s'] <= c['duration_s'] or f.get('delay_s', 0) < 0:
                raise ValueError('invalid fault timing')


def make_arrivals(c, scenario, seed):
    """Piecewise Poisson arrivals; independent RNG per approach, independent of dt/control."""
    events = []
    for i, a in enumerate(APPROACHES):
        rng = random.Random(seed * 1009 + i)
        for j, d in enumerate(scenario['demand']):
            end = scenario['demand'][j + 1]['start_s'] if j + 1 < len(scenario['demand']) else c['duration_s']
            rate, t = d['rates_vph'][a] / 3600, d['start_s']
            if rate == 0:
                continue
            while True:
                t += rng.expovariate(rate)
                if t >= end:
                    break
                events.append((round(t, 8), a))
    events.sort()
    return [{'arrival_time_s': t, 'approach': a, 'vehicle_id': i} for i, (t, a) in enumerate(events)]


def simulate(c, scenario_id, mode, seed, arrivals=None):
    validate(c)
    scenario = c['scenarios'][scenario_id]
    arrivals = make_arrivals(c, scenario, seed) if arrivals is None else arrivals
    if any(r['approach'] not in APPROACHES or not 0 <= r['arrival_time_s'] < c['duration_s'] for r in arrivals):
        raise ValueError('invalid arrival event')
    if arrivals != sorted(arrivals, key=lambda r: (r['arrival_time_s'], r['approach'])):
        raise ValueError('arrivals must be time sorted')
    if len({r['vehicle_id'] for r in arrivals}) != len(arrivals):
        raise ValueError('vehicle ids must be unique')
    demand_hash = hashlib.sha256(json.dumps(arrivals, sort_keys=True).encode()).hexdigest()
    controller, pipeline = SignalController(c, mode), ObservationPipeline(c)
    queues, external = {a: deque() for a in APPROACHES}, {a: deque() for a in APPROACHES}
    entered, departed, generated = ({a: 0 for a in APPROACHES} for _ in range(3))
    previous_entered, previous_departed = entered.copy(), departed.copy()
    next_service = {a: c['startup_lost_s'] for a in APPROACHES}
    admission_times = {a: [] for a in APPROACHES}
    vehicles = {}
    observations, estimates_log, controls, frames, events = [], [], [], [], []
    deliveries, delivery_sequence, arrival_index = [], 0, 0
    max_queue, max_external = entered.copy(), entered.copy()
    queue_integral = 0.0
    dt, duration = c['dt_s'], c['duration_s']
    obs_steps, frame_steps = round(c['detector_interval_s'] / dt), round(c['frame_interval_s'] / dt)

    for step in range(round(duration / dt) + 1):
        t = round(step * dt, 8)
        # Snapshot counters for [t-interval, t), before events at t.
        if step and step % obs_steps == 0:
            for a in APPROACHES:
                row = {'sensor_id': f'entry_exit_{a}', 'approach': a,
                       'interval_start_s': t - c['detector_interval_s'], 'interval_end_s': t,
                       'count': entered[a] - previous_entered[a],
                       'departure_count': departed[a] - previous_departed[a],
                       'cumulative_entered': entered[a], 'cumulative_departed': departed[a],
                       'mean_speed_mps': None, 'quality_flag': 'ok', 'available_time_s': t}
                previous_entered[a], previous_departed[a] = entered[a], departed[a]
                for fault in scenario.get('faults', []):
                    if fault['approach'] == a and fault['start_s'] <= t < fault['end_s']:
                        if fault['kind'] == 'missing':
                            row['quality_flag'] = 'missing'
                            for key in ('count', 'departure_count', 'cumulative_entered', 'cumulative_departed'):
                                row[key] = None
                        else:
                            row['available_time_s'] = t + fault['delay_s']
                            row['quality_flag'] = 'delayed'
                observations.append(row)
                heapq.heappush(deliveries, (row['available_time_s'], delivery_sequence, row))
                delivery_sequence += 1
        while deliveries and deliveries[0][0] <= t + 1e-9:
            _, _, row = heapq.heappop(deliveries)
            pipeline.receive(row, t)
        estimates = pipeline.estimates(t)
        old_phase = (controller.state.group, controller.state.stage)
        record = controller.update(t, estimates)
        new_phase = (controller.state.group, controller.state.stage)
        if new_phase != old_phase and controller.state.stage == 'green':
            for a in APPROACHES:
                if controller.state.permits(a):
                    next_service[a] = t + c['startup_lost_s']
        if step % frame_steps == 0 or record['action'] == 'transition':
            controls.append(record)
        if step % obs_steps == 0:
            pipeline.predict(t)
            estimates_log.extend(estimates.values())

        while arrival_index < len(arrivals) and arrivals[arrival_index]['arrival_time_s'] <= t + 1e-9:
            source = arrivals[arrival_index]
            v = dict(source, entered_time_s=None, departure_time_s=None)
            vehicles[v['vehicle_id']] = v
            external[v['approach']].append(v)
            generated[v['approach']] += 1
            events.append({'time_s': v['arrival_time_s'], 'vehicle_id': v['vehicle_id'],
                           'approach': v['approach'], 'event': 'arrival'})
            arrival_index += 1
        for a in APPROACHES:
            while external[a] and len(queues[a]) < c['storage_veh_per_approach']:
                v = external[a].popleft()
                v['entered_time_s'] = t
                queues[a].append(v)
                entered[a] += 1
                admission_times[a].append(t)
                events.append({'time_s': t, 'vehicle_id': v['vehicle_id'], 'approach': a, 'event': 'entry'})
            max_queue[a] = max(max_queue[a], len(queues[a]))
            max_external[a] = max(max_external[a], len(external[a]))
            if t < duration and controller.state.permits(a) and queues[a] and t >= next_service[a] - 1e-9:
                v = queues[a].popleft()
                v['departure_time_s'] = t
                departed[a] += 1
                next_service[a] = t + c['saturation_headway_s']
                events.append({'time_s': t, 'vehicle_id': v['vehicle_id'], 'approach': a,
                               'event': 'departure'})
            assert generated[a] == departed[a] + len(queues[a]) + len(external[a])
            assert len(queues[a]) <= c['storage_veh_per_approach']
        if t < duration:
            queue_integral += sum(len(queues[a]) + len(external[a]) for a in APPROACHES) * dt
        if step % frame_steps == 0:
            frames.append({'time_s': t, 'phase': record['value'], 'fallback': record['fallback'],
                           'reason': record['reason'], 'queues': {a: len(queues[a]) for a in APPROACHES},
                           'external': {a: len(external[a]) for a in APPROACHES},
                           'estimates': {a: estimates[a]['estimated_queue_veh'] for a in APPROACHES},
                           'quality': {a: estimates[a]['quality_flag'] for a in APPROACHES},
                           'exited_vehicles': sum(departed.values())})

    delay, boundary_delay = {a: 0.0 for a in APPROACHES}, {a: 0.0 for a in APPROACHES}
    for v in vehicles.values():
        finish = v['departure_time_s'] if v['departure_time_s'] is not None else duration
        admission = v['entered_time_s'] if v['entered_time_s'] is not None else duration
        v['waiting_time_s'] = finish - v['arrival_time_s']
        v['boundary_waiting_time_s'] = admission - v['arrival_time_s']
        delay[v['approach']] += v['waiting_time_s']
        boundary_delay[v['approach']] += v['boundary_waiting_time_s']
    for row in pipeline.forecasts:
        end, start, a = row['target_time_s'], row['issue_time_s'], row['segment_id']
        row['evaluation_period'] = start >= c['evaluation_start_s'] and end <= duration
        row['actual_value'] = (bisect_left(admission_times[a], end) - bisect_left(admission_times[a], start)) if end <= duration else None
        row['absolute_error_veh'] = abs(row['predicted_value'] - row['actual_value']) if row['actual_value'] is not None else None
    prediction_metrics = {}
    for method in ('moving_average', 'ema'):
        rows = [r for r in pipeline.forecasts if r['method'] == method and r['evaluation_period']]
        prediction_metrics[method] = {'mae_veh': sum(r['absolute_error_veh'] for r in rows) / len(rows) if rows else None,
                                       'sample_count': len(rows)}
    by_approach = {a: {'generated': generated[a], 'exited': departed[a],
                       'total_delay_vehicle_s': delay[a], 'boundary_delay_vehicle_s': boundary_delay[a],
                       'mean_accrued_delay_s_per_arrival': delay[a] / generated[a] if generated[a] else 0,
                       'max_queue_veh': max_queue[a], 'max_external_queue_veh': max_external[a],
                       'residual_queue_veh': len(queues[a]), 'residual_external_veh': len(external[a])}
                   for a in APPROACHES}
    evaluated = [f for f in frames if f['time_s'] >= c['evaluation_start_s']]
    metrics = {'scenario_id': scenario_id, 'mode': mode, 'seed': seed,
               'demand_id': demand_hash, 'model_version': MODEL_VERSION,
               'controller_version': CONTROLLER_VERSION, 'generated_vehicles': sum(generated.values()),
               'exited_vehicles': sum(departed.values()), 'throughput_vph': sum(departed.values()) * 3600 / duration,
               'total_delay_vehicle_s': sum(delay.values()), 'boundary_delay_vehicle_s': sum(boundary_delay.values()),
               'residual_queue_veh': sum(len(q) for q in queues.values()),
               'residual_external_veh': sum(len(q) for q in external.values()),
               'queue_integral_vehicle_s': queue_integral,
               'arrival_grid_correction_vehicle_s': sum(delay.values()) - queue_integral,
               'approaches': by_approach, 'forecast': prediction_metrics,
               'queue_estimate_mae_veh': sum(abs(f['queues'][a] - f['estimates'][a]) for f in evaluated for a in APPROACHES) / (len(evaluated) * 4),
               'fallback_time_s': sum(f['fallback'] for f in frames[:-1]) * c['frame_interval_s'],
               'service_imbalance_s': max(r['mean_accrued_delay_s_per_arrival'] for r in by_approach.values()) - min(r['mean_accrued_delay_s_per_arrival'] for r in by_approach.values()),
               'conservation_passed': True}
    return {'metrics': metrics, 'frames': frames, 'observations': observations,
            'estimates': estimates_log, 'forecasts': pipeline.forecasts,
            'control_log': controls, 'events': sorted(events, key=lambda r: r['time_s']),
            'vehicles': list(vehicles.values())}
