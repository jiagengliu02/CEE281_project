"""Isolated four-approach signalized intersection on the freeway IDM vehicle model.

Geometry per approach (metres along the travel direction, front-bumper positions):
    0 ............ approach_length_m = stop line S ...... S + box_length_m ...... exit
    |-- single through lane, advance loop at S - advance_detector_m --|-- conflict box --|-- exit leg --|

Each time step runs, in order:
    deliver reports -> arrivals -> boundary entry -> signal decision
    -> IDM motion with the stop line as a signal-dependent obstacle -> loops, metrics, frames.

Car following is the freeway IDM. The stop line is handled by a kinematic rule: a driver
who must stop brakes at the constant deceleration that ends min_gap before the line
(bounded by max_braking_mps2), and the applied acceleration is the smaller of the IDM
and the stopping value. During yellow a driver stops if that required deceleration is at
most yellow_stop_decel_mps2 (ITE dilemma-zone rule); otherwise the vehicle is committed and
clears during yellow/all-red. Commitment is re-evaluated every step, so a committed vehicle
slowed by its leader stops if it can (emergency braking up to max_braking_mps2 on red).
Simulator truth (true queues, positions, future arrivals) is only used for evaluation.
"""
from dataclasses import dataclass
import hashlib
import json
import math
import random

from freeway.behavior import VEHICLE_LENGTH_M, Vehicle, idm_acceleration, motion
from freeway.drivers import driver_parameters, validate_drivers

from .control import MODES, SignalController, webster_plan
from .observation import APPROACHES, GROUPS, ApproachDetectors, SignalObservation

ENTRY_MIN_GAP_M = 2            # standstill gap required when a vehicle enters the approach
GUARD_CLEARANCE_M = .01        # numerical guard keeps this distance to a leader or stop line
MIN_DECEL_DISTANCE_M = .01

GROUP_OF = {a: g for g, members in GROUPS.items() for a in members}


@dataclass
class SignalVehicle(Vehicle):
    committed: bool = False    # decided during yellow to clear the intersection


def current_rates(scenario, t):
    rates = scenario['demand'][0]['rates_vph']
    for period in scenario['demand']:
        if t >= period['start_s']:
            rates = period['rates_vph']
    return rates


def make_arrivals(config, scenario):
    """Reproducible arrival table shared by every controller of one scenario.

    Headways are exponential draws at the rate in force at the previous arrival
    (approximate for piecewise demand; exact within a period). The stream depends
    only on arrival_seed and the approach, never on the controller.
    """
    periods = scenario['demand']
    rows = []
    for approach in APPROACHES:
        rng = random.Random(f"{config['arrival_seed']}:{approach}")
        t = 0.0
        while t < config['duration_s']:
            rate = current_rates(scenario, t)[approach]
            if rate <= 0:
                later = [p['start_s'] for p in periods if p['start_s'] > t]
                if not later:
                    break
                t = later[0]
                continue
            t += rng.expovariate(rate / 3600)
            if t < config['duration_s']:
                rows.append({'time_s': round(t, 3), 'approach': approach})
    rows.sort(key=lambda r: (r['time_s'], APPROACHES.index(r['approach'])))
    for i, row in enumerate(rows):
        row['vehicle_id'] = i
    digest = hashlib.sha1(json.dumps(rows).encode()).hexdigest()[:10]
    return {'rows': rows, 'demand_id': digest}


class IntersectionSimulation:
    def __init__(self, config, scenario_id, mode, arrivals=None):
        validate(config)
        if mode not in MODES:
            raise ValueError('Unknown controller mode')
        self.c, self.mode = config, mode
        self.scenario_id = scenario_id
        self.scenario = config['scenarios'][scenario_id]
        self.arrivals = arrivals or make_arrivals(config, self.scenario)
        self.dt = config['dt_s']
        self.S = config['approach_length_m']
        self.box_end = self.S + config['box_length_m']
        self.end = self.box_end + config['exit_length_m']
        self.advance_x = self.S - config['advance_detector_m']

        self.drivers = {}
        self.lanes = {a: [] for a in APPROACHES}          # front vehicle first
        self.queues = {a: [] for a in APPROACHES}         # boundary (unmet) demand
        self.next_arrival = 0
        self.detectors = ApproachDetectors(config['detector_interval_s'],
                                           {'advance': self.advance_x, 'stopline': self.S},
                                           self.scenario.get('faults', ()))
        self.pipeline = SignalObservation(config['control'])
        self.controller = SignalController(config, mode)
        self.counter_history = {}
        self.last_command = None
        self.fallback_time = 0.0
        self.phases = [{'start_s': 0.0, 'group': 'NS', 'stage': 'green'}]

        # Outputs
        self.frames, self.traces, self.crossings, self.control_log = [], [], [], []
        self.estimates_log, self.forecasts_log = [], []

        # Running metrics
        self.exits = {a: 0 for a in APPROACHES}
        self.entered = {a: 0 for a in APPROACHES}
        self.total_time = {a: 0.0 for a in APPROACHES}
        self.free_time = {a: 0.0 for a in APPROACHES}
        self.max_boundary = {a: 0 for a in APPROACHES}
        self.max_stopped = {a: 0 for a in APPROACHES}
        self.max_zone = {a: 0 for a in APPROACHES}
        self.min_gap = float('inf')
        self.min_accel = 0.0
        self.guards = 0
        self.red_light_runs = 0           # crossing while the other group is green/yellow: must stay 0
        self.all_red_crossings = 0        # committed vehicles still crossing during all-red
        self.yellow_crossings = 0
        self.box_conflicts = 0            # steps with both groups inside the conflict box
        self.sample_steps = round(config['frame_interval_s'] / self.dt)
        self.trace_steps = round(config['trace_interval_s'] / self.dt)
        self.detector_steps = round(config['detector_interval_s'] / self.dt)

    # ------------------------------------------------------------ main loop

    def run(self):
        self.snapshot(0.0)
        steps = round(self.c['duration_s'] / self.dt)
        for step in range(steps):
            t = round(step * self.dt, 8)
            self.counter_history[t] = self.detectors.counts.copy()
            self.observe(t)
            self.generate_arrivals(t)
            for a in APPROACHES:
                self.try_enter(a, t)
            self.control(step, t)
            updates = self.compute_updates(t)
            self.apply_updates(updates, step, t, final=step == steps - 1)
            self.check_invariants()
            if (step + 1) % self.detector_steps == 0:
                self.detectors.report(round(t + self.dt, 8))
            if (step + 1) % self.sample_steps == 0:
                self.snapshot(round(t + self.dt, 8))
        self.counter_history[self.c['duration_s']] = self.detectors.counts.copy()
        self.phases[-1]['end_s'] = self.c['duration_s']
        return self.result()

    # ------------------------------------------------------------ demand and entry

    def generate_arrivals(self, t):
        rows = self.arrivals['rows']
        while self.next_arrival < len(rows) and rows[self.next_arrival]['time_s'] <= t + 1e-9:
            row = rows[self.next_arrival]
            self.drivers[row['vehicle_id']] = driver_parameters(self.c, row['vehicle_id'])
            self.queues[row['approach']].append((row['time_s'], row['vehicle_id']))
            self.next_arrival += 1

    def try_enter(self, approach, t):
        """Place the first waiting vehicle at x = 0 if the gap to the last vehicle is safe."""
        queue = self.queues[approach]
        if not queue:
            return False
        born, vid = queue[0]
        params = self.drivers[vid]
        lane = self.lanes[approach]
        rear = lane[-1] if lane else None
        speed = min(params['desired_speed'], rear.v if rear else params['desired_speed'])
        if rear and rear.x - rear.length < ENTRY_MIN_GAP_M + self.c['entry_headway_s'] * speed:
            return False
        queue.pop(0)
        lane.append(SignalVehicle(vid, approach, born, t, 0.0, speed, 0, **params, entry_speed=speed))
        self.entered[approach] += 1
        return True

    # ------------------------------------------------------------ observation and control

    def observe(self, t):
        changed = False
        for row in self.detectors.deliver(t):
            changed = self.pipeline.receive(row, t) or changed
        if changed:
            for method in ('moving_average', 'ema'):
                for a in APPROACHES:
                    f = self.pipeline.forecast(t, a, method)
                    if f:
                        self.forecasts_log.append(f)

    def control(self, step, t):
        estimates = self.pipeline.estimates(t)
        forecasts = self.pipeline.forecasts(t, 'ema')
        before = (self.controller.state.group, self.controller.state.stage)
        command = self.controller.update(t, estimates, forecasts)
        after = (self.controller.state.group, self.controller.state.stage)
        if after != before:
            self.phases[-1]['end_s'] = t
            self.phases.append({'start_s': t, 'group': after[0], 'stage': after[1]})
        command['scenario_id'] = self.scenario_id
        self.last_command = command
        self.fallback_time += self.dt if command['fallback'] else 0
        if step % self.sample_steps == 0 or command['action'] == 'transition':
            self.control_log.append(command.copy())
        if step % self.sample_steps == 0:
            self.estimates_log.extend(estimates.values())

    # ------------------------------------------------------------ longitudinal motion

    def must_stop(self, car, approach):
        """Decide every step whether the vehicle stops for the signal.

        Yellow: stop if the constant deceleration to the line is at most
        yellow_stop_decel_mps2 (ITE dilemma-zone rule), else commit.
        Red: stop if an emergency stop within max_braking_mps2 is still possible;
        a vehicle that cannot stays committed and clears during all-red.
        """
        if car.x >= self.S:
            car.committed = False
            return False
        phase = self.controller.state.phase_for(approach)
        if phase == 'green':
            car.committed = False
            return False
        distance = max(self.S - car.x, MIN_DECEL_DISTANCE_M)
        required = car.v * car.v / (2 * distance)
        limit = self.c['yellow_stop_decel_mps2'] if phase == 'yellow' else self.c['max_braking_mps2']
        car.committed = required > limit
        return not car.committed

    def stopping_acceleration(self, car):
        """Constant deceleration that stops the front bumper min_gap before the line."""
        distance = self.S - car.x - car.min_gap
        if distance <= MIN_DECEL_DISTANCE_M:
            return -min(self.c['max_braking_mps2'], car.v / self.dt)   # finish the stop this step
        return -car.v * car.v / (2 * distance)

    def compute_updates(self, t):
        updates = {}
        for approach, lane in self.lanes.items():
            for i, car in enumerate(lane):
                lead = lane[i - 1] if i else None
                gap = lead.x - lead.length - car.x if lead else 1e9
                lead_v = lead.v if lead else car.v
                accel = idm_acceleration(car.v, gap, car.v - lead_v, car.desired_speed, car.headway,
                                         car.min_gap, car.max_accel, car.comfortable_braking)
                hold = self.must_stop(car, approach)
                if hold:
                    accel = min(accel, self.stopping_acceleration(car))
                accel = max(-self.c['max_braking_mps2'], min(car.max_accel, accel))
                new_x, new_v = motion(car.x, car.v, accel, self.dt)
                ceiling = updates[lead.id][0] - lead.length - GUARD_CLEARANCE_M if lead else float('inf')
                if hold:
                    ceiling = min(ceiling, self.S - GUARD_CLEARANCE_M)
                if new_x > ceiling:
                    if ceiling < car.x - 1e-9:
                        raise AssertionError('Existing overlap cannot be repaired by a guard')
                    self.guards += 1
                    new_x, new_v = ceiling, 0.0
                if lead:
                    self.min_gap = min(self.min_gap, updates[lead.id][0] - lead.length - new_x)
                updates[car.id] = (new_x, new_v, accel)
        return updates

    def apply_updates(self, updates, step, t, final=False):
        write_trace = step % self.trace_steps == 0 or final   # final row lets audits recount every crossing
        state = self.controller.state
        in_box = set()
        for approach, lane in self.lanes.items():
            self.total_time[approach] += (len(lane) + len(self.queues[approach])) * self.dt
            for car in lane:
                new_x, new_v, accel = updates[car.id]
                covered = max(0.0, min(self.end, new_x) - min(self.end, car.x))
                self.free_time[approach] += covered / car.desired_speed
                for sensor, p in (('advance', self.advance_x), ('stopline', self.S)):
                    if car.x < p <= new_x:
                        speed = math.sqrt(max(0.0, car.v ** 2 + 2 * accel * (p - car.x)))
                        self.detectors.cross(approach, sensor, speed)
                        if sensor == 'stopline':
                            self.record_crossing(car, approach, t, speed)
                if write_trace:
                    self.traces.append({'time_s': round(t + self.dt, 3), 'vehicle_id': car.id,
                        'approach': approach, 'lane_id': 0, 'position_m': round(new_x, 4),
                        'speed_mps': round(new_v, 4), 'acceleration_mps2': round(accel, 4),
                        'committed': int(car.committed)})
                car.x, car.v, car.a = new_x, new_v, accel
                self.min_accel = min(self.min_accel, accel)
                if self.S <= car.x - car.length < self.box_end or self.S <= car.x < self.box_end:
                    in_box.add(GROUP_OF[approach])
            exited = [c for c in lane if c.x >= self.end]
            self.exits[approach] += len(exited)
            self.lanes[approach] = [c for c in lane if c.x < self.end]
            self.max_boundary[approach] = max(self.max_boundary[approach], len(self.queues[approach]))
            self.max_stopped[approach] = max(self.max_stopped[approach], self.stopped(approach))
            self.max_zone[approach] = max(self.max_zone[approach], self.zone_count(approach))
        if len(in_box) > 1:
            self.box_conflicts += 1

    def record_crossing(self, car, approach, t, speed):
        phase = self.controller.state.phase_for(approach)
        stage = self.controller.state.stage
        if phase == 'yellow':
            self.yellow_crossings += 1
        elif phase == 'red' and stage == 'all_red':
            self.all_red_crossings += 1
        elif phase == 'red':
            self.red_light_runs += 1
        self.crossings.append({'time_s': round(t + self.dt, 3), 'vehicle_id': car.id, 'approach': approach,
                               'speed_mps': round(speed, 4), 'signal_phase': phase,
                               'signal_stage': stage, 'committed': int(car.committed),
                               'delay_so_far_s': round(t + self.dt - car.arrival - self.S / car.desired_speed, 3)})
        car.committed = False

    def check_invariants(self):
        generated = self.next_arrival
        in_system = sum(len(l) for l in self.lanes.values()) + sum(len(q) for q in self.queues.values())
        assert generated == sum(self.exits.values()) + in_system, 'Conservation failure'
        for lane in self.lanes.values():
            assert all(c.v >= 0 for c in lane), 'Negative speed'
            assert all(b.x <= a.x - a.length + 1e-6 for a, b in zip(lane, lane[1:])), 'Overlap'

    # ------------------------------------------------------------ truth used for evaluation only

    def stopped(self, approach):
        return sum(c.x < self.S and c.v < self.c['queue_speed_threshold_mps'] for c in self.lanes[approach])

    def zone_count(self, approach):
        return sum(self.advance_x < c.x <= self.S for c in self.lanes[approach])

    def snapshot(self, t):
        state = self.controller.state
        estimates = self.pipeline.estimates(t)
        self.frames.append({
            'time': round(t, 3),
            # compact replay rows: [id, approach index, position in decimetres, committed]
            'vehicles': [[c.id, APPROACHES.index(a), round(c.x * 10), int(c.committed)]
                         for a in APPROACHES for c in self.lanes[a]],
            'signal': [state.group, state.stage, round(t - state.started, 1)],
            'boundary': [len(self.queues[a]) for a in APPROACHES],
            'stopped': [self.stopped(a) for a in APPROACHES],
            'zone_truth': [self.zone_count(a) for a in APPROACHES],
            'estimate': [estimates[a]['estimated_queue_veh'] for a in APPROACHES],
            'quality': [estimates[a]['quality_flag'] for a in APPROACHES],
            'exits': sum(self.exits.values()),
            'reason': self.last_command['reason'] if self.last_command else None,
            'fallback': bool(self.last_command and self.last_command['fallback'])})

    def result(self):
        c = self.c
        for row in self.forecasts_log:
            key = f"{row['segment_id']}_advance"
            start = self.counter_history[round(row['issue_time_s'], 8)]
            end = self.counter_history.get(round(row['target_time_s'], 8))
            row['evaluation_period'] = row['issue_time_s'] >= c['control']['evaluation_start_s'] and end is not None
            row['actual_value'] = end[key] - start[key] if end else None
            row['absolute_error'] = abs(row['predicted_value'] - row['actual_value']) if end else None
        forecast_scores = {}
        for method in ('moving_average', 'ema'):
            rows = [r for r in self.forecasts_log if r['method'] == method and r['evaluation_period']]
            forecast_scores[method] = {'samples': len(rows),
                                       'mae_veh': sum(r['absolute_error'] for r in rows) / len(rows) if rows else None}
        evaluated = [f for f in self.frames if f['time'] >= c['control']['evaluation_start_s']]
        estimate_mae = {}
        for i, a in enumerate(APPROACHES):
            pairs = [(f['estimate'][i], f['zone_truth'][i]) for f in evaluated if f['estimate'][i] is not None]
            estimate_mae[a] = sum(abs(e - z) for e, z in pairs) / len(pairs) if pairs else None
        valid = [v for v in estimate_mae.values() if v is not None]
        greens = [p for p in self.phases if p['stage'] == 'green' and 'end_s' in p]
        green_time = {g: sum(p['end_s'] - p['start_s'] for p in greens if p['group'] == g) for g in GROUPS}
        generated = {a: sum(r['approach'] == a for r in self.arrivals['rows']) for a in APPROACHES}
        delay = {a: round(self.total_time[a] - self.free_time[a], 2) for a in APPROACHES}
        per_vehicle = {a: delay[a] / generated[a] for a in APPROACHES if generated[a]}
        metrics = {
            'scenario_id': self.scenario_id, 'demand_id': self.arrivals['demand_id'],
            'seed': c['arrival_seed'], 'mode': self.mode,
            'model_version': 'idm-intersection-1.0',
            'controller_version': f'two-phase-{self.mode}-1.0',
            'fixed_plan': self.controller.plan,
            'generated_vehicles': sum(generated.values()), 'generated_by_approach': generated,
            'entered_vehicles': sum(self.entered.values()),
            'exited_vehicles': sum(self.exits.values()), 'exits_by_approach': self.exits,
            'remaining_on_road': sum(len(l) for l in self.lanes.values()),
            'remaining_boundary_queue': sum(len(q) for q in self.queues.values()),
            'boundary_queue_by_approach': {a: len(self.queues[a]) for a in APPROACHES},
            'max_boundary_queue_by_approach': self.max_boundary,
            'residual_stopped_queue_by_approach': {a: self.stopped(a) for a in APPROACHES},
            'max_stopped_queue_by_approach': self.max_stopped,
            'max_zone_vehicles_by_approach': self.max_zone,
            'throughput_vph': round(sum(self.exits.values()) * 3600 / c['duration_s'], 2),
            'total_time_in_system_vehicle_s': round(sum(self.total_time.values()), 2),
            'total_delay_vehicle_s': round(sum(delay.values()), 2),
            'delay_by_approach_vehicle_s': delay,
            'delay_per_generated_vehicle_s': {a: round(v, 2) for a, v in per_vehicle.items()},
            'service_imbalance_ratio': (round(max(per_vehicle.values()) / min(per_vehicle.values()), 3)
                                        if per_vehicle and min(per_vehicle.values()) > 0 else None),
            'green_time_s': green_time, 'green_phases': len(greens),
            'fallback_time_s': round(self.fallback_time, 3),
            'queue_estimate_mae_veh': {a: (round(v, 3) if v is not None else None) for a, v in estimate_mae.items()},
            'queue_estimate_mae_mean_veh': round(sum(valid) / len(valid), 3) if valid else None,
            'forecast': forecast_scores,
            'minimum_gap_m': round(self.min_gap, 4) if self.min_gap != float('inf') else None,
            'strongest_braking_mps2': round(self.min_accel, 4),
            'safety_interventions': self.guards,
            'yellow_crossings': self.yellow_crossings,
            'all_red_crossings': self.all_red_crossings,
            'red_light_runs': self.red_light_runs,
            'conflict_box_steps_with_both_groups': self.box_conflicts,
            'conservation_passed': True,
        }
        return {'metrics': metrics, 'frames': self.frames, 'traces': self.traces,
                'crossings': self.crossings, 'control_log': self.control_log,
                'observations': self.detectors.records, 'estimates': self.estimates_log,
                'forecasts': self.forecasts_log, 'phases': self.phases,
                'arrivals': self.arrivals['rows'],
                'drivers': [dict(vehicle_id=vid, **p) for vid, p in sorted(self.drivers.items())]}


def simulate(config, scenario_id, mode, arrivals=None):
    return IntersectionSimulation(config, scenario_id, mode, arrivals).run()


def validate(c):
    validate_drivers(c)
    c.setdefault('trace_interval_s', 0.5)
    if c['dt_s'] <= 0 or c['duration_s'] <= 0:
        raise ValueError('Positive time parameters required')
    for name in ('duration_s', 'frame_interval_s', 'detector_interval_s', 'trace_interval_s',
                 'yellow_s', 'all_red_s', 'min_green_s', 'max_green_s'):
        n = c[name] / c['dt_s']
        if abs(n - round(n)) > 1e-6 or n < 1:
            raise ValueError(name + ' must be a positive multiple of dt_s')
    if abs(c['duration_s'] / c['detector_interval_s'] - round(c['duration_s'] / c['detector_interval_s'])) > 1e-6:
        raise ValueError('duration must cover complete detector intervals')
    if not 0 < c['advance_detector_m'] < c['approach_length_m']:
        raise ValueError('advance loop must lie on the approach')
    for key in ('box_length_m', 'exit_length_m', 'max_braking_mps2', 'yellow_stop_decel_mps2',
                'entry_headway_s', 'startup_lost_s', 'saturation_flow_vph', 'queue_speed_threshold_mps'):
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError('Parameter must be finite and positive: ' + key)
    if not 0 < c['min_green_s'] <= c['max_green_s']:
        raise ValueError('Require 0 < min_green_s <= max_green_s')
    if not 0 <= c['yellow_used_s'] <= c['yellow_s']:
        raise ValueError('yellow_used_s must lie within the yellow interval')
    k = c['control']
    for key in ('stale_after_s', 'forecast_horizon_s'):
        if not math.isfinite(k[key]) or k[key] <= 0:
            raise ValueError('Control parameter must be finite and positive: ' + key)
    if k['switch_threshold_veh'] < 0:
        raise ValueError('switch_threshold_veh must be non-negative')
    if not 0 < k['ema_alpha'] <= 1 or type(k['moving_average_intervals']) is not int or k['moving_average_intervals'] < 1:
        raise ValueError('Invalid forecasting parameters')
    ratio = k['forecast_horizon_s'] / c['dt_s']
    if abs(ratio - round(ratio)) > 1e-6:
        raise ValueError('Forecast horizon must align with simulation steps')
    if type(c['arrival_seed']) is not int:
        raise ValueError('arrival_seed must be an integer')
    for name, scenario in c['scenarios'].items():
        if not scenario['demand'] or scenario['demand'][0]['start_s'] != 0:
            raise ValueError(f'{name}: demand must start at 0 s')
        for period in scenario['demand']:
            if set(period['rates_vph']) != set(APPROACHES) or any(v < 0 for v in period['rates_vph'].values()):
                raise ValueError(f'{name}: each period needs non-negative rates for N, S, E, W')
        for fault in scenario.get('faults', ()):
            if fault['approach'] not in APPROACHES or fault['kind'] not in ('missing', 'delay') \
                    or not 0 <= fault['start_s'] < fault['end_s']:
                raise ValueError(f'{name}: invalid detector fault')
            delay = fault.get('delay_s', 0)
            if delay < 0 or abs(delay / c['dt_s'] - round(delay / c['dt_s'])) > 1e-6:
                raise ValueError('Detector delay must align with simulation steps')
    webster_plan(c)   # raises if the design demand cannot be served
