"""Isolated two-phase intersection as finite-storage FIFO point queues.

Each time step runs, in order:
    detector reports -> estimates -> signal decision -> arrivals
    -> admission into storage -> stop-line discharge -> metrics.
No spatial trajectories, turning movements or pedestrians. Arrivals are generated
before control, so paired experiments share exactly the same demand.
"""
from bisect import bisect_left
from collections import deque
import hashlib
import json
import math
import random

from .control import SignalController
from .detector import APPROACHES, StopLineDetectors
from .observation import ObservationPipeline

MODEL_VERSION = 'signal-point-queue-1.0'
CONTROLLER_VERSION = 'bounded-queue-response-1.0'


# ---------------------------------------------------------------- configuration

def validate(c):
    positive = ('dt_s', 'duration_s', 'frame_interval_s', 'detector_interval_s',
                'saturation_headway_s', 'yellow_s', 'all_red_s', 'min_green_s',
                'max_green_s', 'forecast_horizon_s', 'stale_after_s')
    for key in positive:
        if not isinstance(c[key], (int, float)) or not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'{key} must be positive and finite')
    on_time_grid = ('duration_s', 'frame_interval_s', 'detector_interval_s',
                    'saturation_headway_s', 'startup_lost_s', 'yellow_s', 'all_red_s',
                    'min_green_s', 'max_green_s')
    for key in on_time_grid:
        if c[key] < 0 or not _on_grid(c[key], c['dt_s']):
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
        if not c['min_green_s'] <= green <= c['max_green_s'] or not _on_grid(green, c['dt_s']):
            raise ValueError('fixed greens must respect bounds and time grid')
    if set(c['fixed_green_s']) != {'NS', 'EW'}:
        raise ValueError('fixed plan must specify NS and EW')
    for scenario in c['scenarios'].values():
        _validate_scenario(scenario, c['duration_s'])


def _on_grid(value, dt):
    return abs(value / dt - round(value / dt)) <= 1e-8


def _validate_scenario(scenario, duration):
    starts = [d['start_s'] for d in scenario['demand']]
    if starts[0] != 0 or starts != sorted(set(starts)) or starts[-1] >= duration:
        raise ValueError('demand starts must be sorted, unique, starting at zero and within duration')
    for d in scenario['demand']:
        rates = d['rates_vph']
        if set(rates) != set(APPROACHES) or any(not math.isfinite(v) or v < 0 for v in rates.values()):
            raise ValueError('nonnegative finite demand for all four approaches required')
    for f in scenario.get('faults', []):
        if f['approach'] not in APPROACHES or f['kind'] not in ('missing', 'delay'):
            raise ValueError('unsupported detector fault')
        if not 0 <= f['start_s'] < f['end_s'] <= duration or f.get('delay_s', 0) < 0:
            raise ValueError('invalid fault timing')


# ---------------------------------------------------------------- demand

def make_arrivals(c, scenario, seed):
    """Piecewise Poisson arrivals; one RNG per approach, independent of dt and control."""
    events = []
    for i, a in enumerate(APPROACHES):
        rng = random.Random(seed * 1009 + i)
        for j, period in enumerate(scenario['demand']):
            is_last = j + 1 == len(scenario['demand'])
            end = c['duration_s'] if is_last else scenario['demand'][j + 1]['start_s']
            rate, t = period['rates_vph'][a] / 3600, period['start_s']
            if rate == 0:
                continue
            while True:
                t += rng.expovariate(rate)
                if t >= end:
                    break
                events.append((round(t, 8), a))
    events.sort()
    return [{'arrival_time_s': t, 'approach': a, 'vehicle_id': i} for i, (t, a) in enumerate(events)]


def _check_arrivals(arrivals, duration):
    if any(r['approach'] not in APPROACHES or not 0 <= r['arrival_time_s'] < duration for r in arrivals):
        raise ValueError('invalid arrival event')
    if arrivals != sorted(arrivals, key=lambda r: (r['arrival_time_s'], r['approach'])):
        raise ValueError('arrivals must be time sorted')
    if len({r['vehicle_id'] for r in arrivals}) != len(arrivals):
        raise ValueError('vehicle ids must be unique')


# ---------------------------------------------------------------- simulation

class IntersectionSimulation:
    def __init__(self, c, scenario_id, mode, seed, arrivals=None):
        validate(c)
        self.c = c
        self.scenario_id, self.mode, self.seed = scenario_id, mode, seed
        scenario = c['scenarios'][scenario_id]
        self.arrivals = make_arrivals(c, scenario, seed) if arrivals is None else arrivals
        _check_arrivals(self.arrivals, c['duration_s'])
        self.demand_hash = hashlib.sha256(json.dumps(self.arrivals, sort_keys=True).encode()).hexdigest()

        # Information flow: plant -> detectors -> pipeline (estimates) -> controller.
        self.detectors = StopLineDetectors(c, scenario.get('faults', []))
        self.pipeline = ObservationPipeline(c)
        self.controller = SignalController(c, mode)

        # Plant state. `queues` is on-road storage; `external` holds vehicles
        # that arrived while storage was full (spillback beyond the boundary).
        self.queues = {a: deque() for a in APPROACHES}
        self.external = {a: deque() for a in APPROACHES}
        self.generated = {a: 0 for a in APPROACHES}
        self.entered = {a: 0 for a in APPROACHES}
        self.departed = {a: 0 for a in APPROACHES}
        self.next_service = {a: c['startup_lost_s'] for a in APPROACHES}   # earliest next discharge
        self.admission_times = {a: [] for a in APPROACHES}
        self.vehicles = {}
        self.next_arrival = 0

        # Outputs and running metrics
        self.estimates_log, self.control_log, self.frames, self.events = [], [], [], []
        self.max_queue = {a: 0 for a in APPROACHES}
        self.max_external = {a: 0 for a in APPROACHES}
        self.queue_integral = 0.0

    # ------------------------------------------------------------ main loop

    def run(self):
        c = self.c
        dt, duration = c['dt_s'], c['duration_s']
        detector_steps = round(c['detector_interval_s'] / dt)
        frame_steps = round(c['frame_interval_s'] / dt)

        for step in range(round(duration / dt) + 1):
            t = round(step * dt, 8)
            # Sensing: close the interval [t - interval, t) before any event at t.
            if step and step % detector_steps == 0:
                self.detectors.report(t, self.entered, self.departed)
            for row in self.detectors.deliver(t):
                self.pipeline.receive(row, t)
            estimates = self.pipeline.estimates(t)

            record = self.update_signal(t, estimates)
            if step % frame_steps == 0 or record['action'] == 'transition':
                self.control_log.append(record)
            if step % detector_steps == 0:
                self.pipeline.predict(t)
                self.estimates_log.extend(estimates.values())

            self.add_arrivals(t)
            for a in APPROACHES:
                self.update_approach(a, t)

            if t < duration:
                waiting = sum(len(self.queues[a]) + len(self.external[a]) for a in APPROACHES)
                self.queue_integral += waiting * dt
            if step % frame_steps == 0:
                self.snapshot(t, record, estimates)
        return self.result()

    def update_signal(self, t, estimates):
        """Advance the controller; a new green starts with start-up lost time."""
        old_phase = (self.controller.state.group, self.controller.state.stage)
        record = self.controller.update(t, estimates)
        state = self.controller.state
        if (state.group, state.stage) != old_phase and state.stage == 'green':
            for a in APPROACHES:
                if state.permits(a):
                    self.next_service[a] = t + self.c['startup_lost_s']
        return record

    def add_arrivals(self, t):
        """Arrivals up to t join the external (boundary) queue first."""
        while (self.next_arrival < len(self.arrivals)
               and self.arrivals[self.next_arrival]['arrival_time_s'] <= t + 1e-9):
            source = self.arrivals[self.next_arrival]
            vehicle = dict(source, entered_time_s=None, departure_time_s=None)
            self.vehicles[vehicle['vehicle_id']] = vehicle
            self.external[vehicle['approach']].append(vehicle)
            self.generated[vehicle['approach']] += 1
            self.log_event(vehicle['arrival_time_s'], vehicle, 'arrival')
            self.next_arrival += 1

    def update_approach(self, a, t):
        """Admit into free storage, then discharge one vehicle if green and headway allows."""
        queue, external = self.queues[a], self.external[a]
        while external and len(queue) < self.c['storage_veh_per_approach']:
            vehicle = external.popleft()
            vehicle['entered_time_s'] = t
            queue.append(vehicle)
            self.entered[a] += 1
            self.admission_times[a].append(t)
            self.log_event(t, vehicle, 'entry')
        self.max_queue[a] = max(self.max_queue[a], len(queue))
        self.max_external[a] = max(self.max_external[a], len(external))

        can_discharge = (t < self.c['duration_s'] and self.controller.state.permits(a)
                         and queue and t >= self.next_service[a] - 1e-9)
        if can_discharge:
            vehicle = queue.popleft()
            vehicle['departure_time_s'] = t
            self.departed[a] += 1
            self.next_service[a] = t + self.c['saturation_headway_s']
            self.log_event(t, vehicle, 'departure')

        assert self.generated[a] == self.departed[a] + len(queue) + len(external), 'Conservation failure'
        assert len(queue) <= self.c['storage_veh_per_approach']

    def log_event(self, t, vehicle, kind):
        self.events.append({'time_s': t, 'vehicle_id': vehicle['vehicle_id'],
                            'approach': vehicle['approach'], 'event': kind})

    def snapshot(self, t, record, estimates):
        """One replay frame: true queues next to the controller's estimates."""
        self.frames.append({'time_s': t, 'phase': record['value'], 'fallback': record['fallback'],
            'reason': record['reason'],
            'queues': {a: len(self.queues[a]) for a in APPROACHES},
            'external': {a: len(self.external[a]) for a in APPROACHES},
            'estimates': {a: estimates[a]['estimated_queue_veh'] for a in APPROACHES},
            'quality': {a: estimates[a]['quality_flag'] for a in APPROACHES},
            'exited_vehicles': sum(self.departed.values())})

    # ------------------------------------------------------------ evaluation (uses truth)

    def accrue_delays(self):
        """Delay = time from arrival to departure (or to the end of the run)."""
        duration = self.c['duration_s']
        delay = {a: 0.0 for a in APPROACHES}
        boundary_delay = {a: 0.0 for a in APPROACHES}
        for v in self.vehicles.values():
            finish = v['departure_time_s'] if v['departure_time_s'] is not None else duration
            admission = v['entered_time_s'] if v['entered_time_s'] is not None else duration
            v['waiting_time_s'] = finish - v['arrival_time_s']
            v['boundary_waiting_time_s'] = admission - v['arrival_time_s']
            delay[v['approach']] += v['waiting_time_s']
            boundary_delay[v['approach']] += v['boundary_waiting_time_s']
        return delay, boundary_delay

    def score_forecasts(self):
        """Compare each forecast with the true admitted count over its horizon."""
        duration = self.c['duration_s']
        for row in self.pipeline.forecasts:
            start, end, a = row['issue_time_s'], row['target_time_s'], row['segment_id']
            times = self.admission_times[a]
            row['evaluation_period'] = start >= self.c['evaluation_start_s'] and end <= duration
            in_window = bisect_left(times, end) - bisect_left(times, start)
            row['actual_value'] = in_window if end <= duration else None
            row['absolute_error_veh'] = (abs(row['predicted_value'] - row['actual_value'])
                                         if row['actual_value'] is not None else None)
        scores = {}
        for method in ('moving_average', 'ema'):
            rows = [r for r in self.pipeline.forecasts if r['method'] == method and r['evaluation_period']]
            scores[method] = {'mae_veh': sum(r['absolute_error_veh'] for r in rows) / len(rows) if rows else None,
                              'sample_count': len(rows)}
        return scores

    def result(self):
        c, duration = self.c, self.c['duration_s']
        delay, boundary_delay = self.accrue_delays()
        forecast_scores = self.score_forecasts()
        by_approach = {a: {
            'generated': self.generated[a],
            'exited': self.departed[a],
            'total_delay_vehicle_s': delay[a],
            'boundary_delay_vehicle_s': boundary_delay[a],
            'mean_accrued_delay_s_per_arrival': delay[a] / self.generated[a] if self.generated[a] else 0,
            'max_queue_veh': self.max_queue[a],
            'max_external_queue_veh': self.max_external[a],
            'residual_queue_veh': len(self.queues[a]),
            'residual_external_veh': len(self.external[a]),
        } for a in APPROACHES}

        evaluated = [f for f in self.frames if f['time_s'] >= c['evaluation_start_s']]
        estimate_error = sum(abs(f['queues'][a] - f['estimates'][a]) for f in evaluated for a in APPROACHES)
        mean_delays = [r['mean_accrued_delay_s_per_arrival'] for r in by_approach.values()]
        exited = sum(self.departed.values())
        total_delay = sum(delay.values())

        metrics = {
            'scenario_id': self.scenario_id, 'mode': self.mode, 'seed': self.seed,
            'demand_id': self.demand_hash, 'model_version': MODEL_VERSION,
            'controller_version': CONTROLLER_VERSION,
            'generated_vehicles': sum(self.generated.values()),
            'exited_vehicles': exited,
            'throughput_vph': exited * 3600 / duration,
            'total_delay_vehicle_s': total_delay,
            'boundary_delay_vehicle_s': sum(boundary_delay.values()),
            'residual_queue_veh': sum(len(q) for q in self.queues.values()),
            'residual_external_veh': sum(len(q) for q in self.external.values()),
            'queue_integral_vehicle_s': self.queue_integral,
            # Arrivals are served on the dt grid; this is the only gap to the queue integral.
            'arrival_grid_correction_vehicle_s': total_delay - self.queue_integral,
            'approaches': by_approach,
            'forecast': forecast_scores,
            'queue_estimate_mae_veh': estimate_error / (len(evaluated) * 4),
            'fallback_time_s': sum(f['fallback'] for f in self.frames[:-1]) * c['frame_interval_s'],
            'service_imbalance_s': max(mean_delays) - min(mean_delays),
            'conservation_passed': True,    # asserted every step in update_approach()
        }
        return {'metrics': metrics, 'frames': self.frames,
                'observations': self.detectors.observations,
                'estimates': self.estimates_log, 'forecasts': self.pipeline.forecasts,
                'control_log': self.control_log,
                'events': sorted(self.events, key=lambda r: r['time_s']),
                'vehicles': list(self.vehicles.values())}


def simulate(c, scenario_id, mode, seed, arrivals=None):
    """Run one scenario with one controller ('fixed', 'adaptive', 'adaptive_no_hysteresis')."""
    return IntersectionSimulation(c, scenario_id, mode, seed, arrivals).run()
