"""Multi-lane freeway with an on-ramp and a left-lane work-zone closure.

Each time step runs, in order:
    arrivals -> road entry -> ramp control -> lane changes
    -> longitudinal update -> detectors and metrics.
Modes: open, closure, fixed, adaptive, adaptive_fault.
Boundary reports -> estimates -> forecasts -> metering -> IDM motion -> evaluation.
"""
from .observation import BoundaryDetectors, RampObservation, CHANNELS
from .control import RampController
from .ramp import RampRoad

from dataclasses import dataclass
import math

from .behavior import (VEHICLE_LENGTH_M, Vehicle, blocked, courtesy_requests, idm_acceleration, in_merge_zone,
                       lane_change_candidate, lane_neighbors, motion, raw_acceleration, lane_end, merge_urgency)

ENTRY_MIN_GAP_M = 2            # standstill gap required when a vehicle enters the road
ENTRY_CLOSING_FACTOR_S = 2     # extra entry gap per m/s the follower is faster than the new car
SPEED_CLASSES = 5              # desired speed = base + variation * {-1, -0.5, 0, 0.5, 1}
GUARD_CLEARANCE_M = .01        # numerical guard keeps this distance to a leader or lane end
OWED_PASSAGE_RANGE_M = 5       # a yielding car keeps its owed turn until this far past closure start


@dataclass
class DetectorBin:
    """Counts crossings and sums crossing speeds over one reporting interval."""
    count: int = 0
    speed_sum: float = 0.0


def preferred_speed(config, vehicle_id):
    """Deterministic desired speed so every mode gives a vehicle the same driver."""
    speed_class = (vehicle_id % SPEED_CLASSES) - 2
    return config['desired_speed_mps'] + config['speed_variation_mps'] * speed_class / 2


def current_demand(config, t):
    """Piecewise-constant demand period active at time t."""
    demand = config['demand'][0]
    for period in config['demand']:
        if t >= period['start_s']:
            demand = period
    return demand


class FreewaySimulation:
    def __init__(self, config, mode):
        validate(config)
        self.c = config
        self.mode = mode
        self.closure = mode != 'open'
        self.dt = config['dt_s']
        self.lanes = config['lanes']

        self.vehicles = []
        self.queues = {f'main_{i}': [] for i in range(self.lanes)}   # boundary queues
        self.queues['ramp'] = []
        self.ramp = RampRoad(config)
        self.credit = {'main': 0.0, 'ramp': 0.0}       # fractional arrivals carried over
        self.arrivals = {'main': 0, 'ramp': 0}
        if mode not in ('open', 'closure', 'fixed', 'adaptive', 'adaptive_fault'):
            raise ValueError('Unknown freeway mode')
        faults = config['tmc']['faults'] if mode == 'adaptive_fault' else ()
        self.boundary_detectors = BoundaryDetectors(config['detector_interval_s'], faults)
        self.pipeline = RampObservation(config['tmc'])
        self.controller = RampController(config['tmc'], mode, config['ramp_release_interval_s'])
        self.estimates_log, self.forecasts_log = [], []
        self.counter_history = {}
        self.last_command = None
        self.fallback_time = 0.0
        self.requests = {}                              # {yielding_id: merging_vehicle}
        self.owed_passage = set()                       # cars that already yielded once
        self.detectors = {(p, lane): DetectorBin()
                          for p in config['detectors_m'] for lane in range(self.lanes)}

        # Outputs
        self.frames, self.traces, self.observations = [], [], []
        self.control_log, self.lane_changes, self.yield_log = [], [], []

        # Running metrics
        self.exits = 0
        self.total_time = 0.0       # vehicle-seconds in the system, including boundary queues
        self.free_time = 0.0        # vehicle-seconds needed at desired speed for distance covered
        self.min_gap = float('inf')
        self.min_accel = 0.0
        self.guards = 0
        self.max_main = self.max_ramp = self.max_road = 0

        self.sample_steps = round(config['frame_interval_s'] / self.dt)
        self.lane_change_steps = round(config['lane_change_interval_s'] / self.dt)
        self.detector_steps = round(config['detector_interval_s'] / self.dt)

    # ------------------------------------------------------------ main loop

    def run(self):
        self.snapshot(0)
        for step in range(round(self.c['duration_s'] / self.dt)):
            t = round(step * self.dt, 8)
            self.counter_history[t] = self.boundary_detectors.counts.copy()
            self.observe(t)
            self.generate_arrivals(t)
            for lane in range(self.lanes):
                self.try_enter('main', lane, t)
            self.ramp.admit(self.queues['ramp'], t)
            self.control_ramp(step, t)
            self.ramp.advance(self.dt)
            if step % self.lane_change_steps == 0:
                self.change_lanes(t)
            self.owed_passage.intersection_update(
                c.id for c in self.vehicles if c.x < self.c['closure_start_m'] + OWED_PASSAGE_RANGE_M)
            self.requests = courtesy_requests(self.vehicles, self.c, self.closure, self.owed_passage)
            updates = self.compute_updates(step, t)
            self.apply_updates(updates, t)
            self.check_invariants()
            if (step + 1) % self.detector_steps == 0:
                self.report_detectors(round(t + self.dt, 8))
                self.boundary_detectors.report(round(t + self.dt, 8))
            if (step + 1) % self.sample_steps == 0:
                self.snapshot(t + self.dt)
        self.counter_history[self.c['duration_s']] = self.boundary_detectors.counts.copy()
        return self.result()

    # ------------------------------------------------------------ demand and entry

    def generate_arrivals(self, t):
        """Deterministic arrivals; mainline vehicles are assigned to lanes round-robin."""
        demand = current_demand(self.c, t)
        for origin in self.credit:
            self.credit[origin] += demand[origin + '_vph'] * self.dt / 3600
            while self.credit[origin] >= 1 - 1e-10:
                vehicle_id = sum(self.arrivals.values())
                if origin == 'main':
                    queue = f'main_{self.arrivals[origin] % self.lanes}'
                else:
                    queue = 'ramp'
                self.queues[queue].append((t, vehicle_id))
                self.arrivals[origin] += 1
                self.boundary_detectors.cross(origin + '_arrivals')
                self.credit[origin] -= 1

    def try_enter(self, origin, lane, t):
        """Move the first queued vehicle onto the road if the gaps are safe."""
        queue = self.queues[f'main_{lane}' if origin == 'main' else 'ramp']
        if not queue:
            return False
        x = 0.0 if origin == 'main' else self.c['merge_position_m']
        lead, follow = lane_neighbors(self.vehicles, lane, x)
        born, vehicle_id = queue[0]
        if origin == 'ramp' and not self.ramp.ready(vehicle_id):
            return False
        desired = preferred_speed(self.c, vehicle_id)
        speed = min(desired, lead.v if lead else desired)
        # Restore the original mainline handoff speed; ramp crawl speed
        # does not cap the vehicle after it joins the freeway.
        headway = self.c['merge_headway_s']

        if lead and lead.x - lead.length - x < ENTRY_MIN_GAP_M + headway * speed:
            return False
        if follow:
            gap_behind = x - VEHICLE_LENGTH_M - follow.x
            needed = (ENTRY_MIN_GAP_M + headway * follow.v
                      + max(0, follow.v - speed) * ENTRY_CLOSING_FACTOR_S)
            if gap_behind < needed:
                return False
            candidate = Vehicle(vehicle_id, origin, born, t, x, speed, lane, desired_speed=desired)
            follower_accel = raw_acceleration(follow, candidate, lane, self.c, self.closure)
            if follower_accel < -self.c['lane_change_safe_braking_mps2']:
                return False

        queue.pop(0)
        if origin == 'ramp':
            self.ramp.remove_front()
        self.boundary_detectors.cross(origin + '_entries')
        self.vehicles.append(Vehicle(vehicle_id, origin, born, t, x, speed, lane,
                                     desired_speed=desired, previous_lane=lane, entry_speed=speed,
                                     lane_change_time=t if origin == 'ramp' else -100.0))
        return True

    def observe(self, t):
        """Consume delivered reports; forecasts use historical measured rates only."""
        changed = False
        for row in self.boundary_detectors.deliver(t):
            changed = self.pipeline.receive(row, t) or changed
        if changed:
            for method in ('moving_average', 'ema'):
                prediction = self.pipeline.forecast(t, method)
                if prediction:
                    for channel, value in prediction['counts'].items():
                        self.forecasts_log.append({'issue_time_s': t,
                            'target_time_s': prediction['target_time_s'],
                            'segment_id': channel, 'variable_name': 'interval_crossings',
                            'method': method, 'predicted_value': value, 'units': 'veh',
                            'quality_flag': 'ok'})

    def control_ramp(self, step, t):
        """Meter eligibility then safe insertion; the command never sees true queues."""
        estimate = self.pipeline.estimate(t)
        prediction = self.pipeline.forecast(t)
        command = self.controller.decide(t, estimate, prediction)
        eligible = command['action'] == 'permit'
        released = eligible and self.try_enter('ramp', self.lanes - 1, t)
        if released:
            self.controller.executed(t)
        command.update(executed_action='release' if released else 'hold',
                       execution_reason=('safe_gap_release' if released else
                                         'no_safe_gap_or_no_demand' if eligible else 'meter_wait'),
                       scenario=self.mode, last_release_s=self.controller.last_release)
        self.last_command = command
        self.fallback_time += self.dt if command['fallback'] else 0
        if step % self.sample_steps == 0 or released:
            self.control_log.append(command.copy())
        if step % self.sample_steps == 0:
            self.estimates_log.append(estimate)

    # ------------------------------------------------------------ lane changes

    def change_lanes(self, t):
        """Evaluate lane changes one car at a time.

        Merging cars go first, then front to back. Each accepted move updates
        lane occupancy immediately, so two cars cannot claim the same gap.
        """
        priority = sorted(self.vehicles, key=lambda c: (
            not in_merge_zone(c, self.c, self.closure), -c.x, c.id))
        for car in priority:
            if car.origin == 'ramp' and car.x < self.c['merge_position_m'] + 45:
                continue
            if t - car.lane_change_time < self.c['lane_change_cooldown_s']:
                continue
            mandatory = in_merge_zone(car, self.c, self.closure)
            targets = [car.lane + 1] if mandatory else [car.lane - 1, car.lane + 1]
            choices = []
            for target in targets:
                choice = lane_change_candidate(car, target, self.vehicles, self.c, self.closure, mandatory)
                if choice is not None:
                    choices.append((choice['score'], target, choice))
            if not choices:
                continue
            _, target, choice = max(choices, key=lambda item: item[0])
            if mandatory:
                # The car that yielded for this merger gets its own turn next.
                self.owed_passage.update(
                    vid for vid, source in self.requests.items() if source.id == car.id)
            old_lane = car.lane
            car.previous_lane, car.lane, car.lane_change_time = old_lane, target, t
            self.lane_changes.append({'time_s': round(t, 3), 'vehicle_id': car.id,
                'position_m': round(car.x, 4), 'from_lane': old_lane, 'to_lane': target,
                'reason': 'closure_merge' if mandatory else 'incentive',
                'self_acceleration_after_mps2': round(choice['self_after'], 4),
                'target_follower_acceleration_after_mps2': round(choice['follower_after'], 4)})

    # ------------------------------------------------------------ longitudinal motion

    def compute_updates(self, step, t):
        """New (x, v, a) for every car, all computed from the same time-t state."""
        updates = {}
        for lane in range(self.lanes):
            ordered = sorted([c for c in self.vehicles if c.lane == lane], key=lambda c: c.x, reverse=True)
            for i, car in enumerate(ordered):
                lead = ordered[i - 1] if i else None
                accel = raw_acceleration(car, lead, lane, self.c, self.closure)
                if car.id in self.requests:
                    accel = self.yield_acceleration(car, accel, step, t)
                accel = max(-self.c['max_braking_mps2'], min(car.max_accel, accel))
                new_x, new_v = motion(car.x, car.v, accel, self.dt)
                new_x, new_v = self.guard(car, lead, lane, new_x, new_v, updates)
                if lead:
                    self.min_gap = min(self.min_gap, updates[lead.id][0] - lead.length - new_x)
                assert not blocked(self.c, lane, new_x, self.closure), 'Vehicle entered closed lane'
                updates[car.id] = (new_x, new_v, accel)
        return updates

    def yield_acceleration(self, car, accel, step, t):
        """Moderate slowing to open a gap for the merger; never weaker than the
        braking already needed for the real leader."""
        merger = self.requests[car.id]
        yield_a = idm_acceleration(car.v, merger.x - merger.length - car.x,
            car.v - merger.v, car.desired_speed, car.headway,
            car.min_gap + self.c['yield_gap_buffer_m'], car.max_accel, car.comfortable_braking)
        strength = 1.0
        if self.c.get('taper_start_m') is not None:
            strength = merge_urgency(merger, self.c, self.closure)
        applied = min(accel, max(-self.c['yield_braking_mps2'] * strength, yield_a))
        if step % self.sample_steps == 0:
            self.yield_log.append({'time_s': round(t, 3), 'yielding_vehicle_id': car.id,
                'merging_vehicle_id': merger.id, 'position_m': round(car.x, 4),
                'requested_acceleration_mps2': round(yield_a, 4),
                'applied_acceleration_mps2': round(applied, 4)})
        return applied

    def guard(self, car, lead, lane, new_x, new_v, updates):
        """Counted numerical guard: never pass the leader's new rear or the lane end."""
        ceiling = updates[lead.id][0] - lead.length - GUARD_CLEARANCE_M if lead else float('inf')
        if self.closure and lane == self.c['closed_lane'] and car.x < lane_end(self.c):
            ceiling = min(ceiling, lane_end(self.c) - GUARD_CLEARANCE_M)
        if new_x > ceiling:
            if ceiling < car.x:
                raise AssertionError('Existing overlap cannot be repaired by a guard')
            self.guards += 1
            return ceiling, 0.0
        return new_x, new_v

    def apply_updates(self, updates, t):
        """Commit the step: accumulate delay, detector crossings, traces; remove exits."""
        road_length = self.c['road_length_m']
        self.total_time += (len(self.vehicles) + self.main_waiting() + len(self.queues['ramp'])) * self.dt
        for car in self.vehicles:
            new_x, new_v, accel = updates[car.id]
            covered = max(0, min(road_length, new_x) - min(road_length, car.x))
            self.free_time += covered / car.desired_speed
            for p in self.c['detectors_m']:
                if car.x < p <= new_x:
                    detector = self.detectors[(p, car.lane)]
                    detector.count += 1
                    # Speed at the detector, from constant acceleration over the step.
                    detector.speed_sum += math.sqrt(max(0, car.v ** 2 + 2 * accel * (p - car.x)))
            self.traces.append({'time_s': round(t + self.dt, 3), 'vehicle_id': car.id,
                'origin': car.origin, 'lane_id': car.lane, 'position_m': round(new_x, 4),
                'speed_mps': round(new_v, 4), 'acceleration_mps2': round(accel, 4)})
            car.x, car.v, car.a = new_x, new_v, accel
            self.min_accel = min(self.min_accel, accel)
        for car in self.vehicles:
            if car.x >= road_length:
                self.boundary_detectors.cross('exits')
        self.exits += sum(c.x >= road_length for c in self.vehicles)
        self.vehicles = [c for c in self.vehicles if c.x < road_length]
        self.max_main = max(self.max_main, self.main_waiting())
        self.max_ramp = max(self.max_ramp, len(self.queues['ramp']))
        self.max_road = max(self.max_road, self.slow_vehicles())

    def check_invariants(self):
        in_system = len(self.vehicles) + self.main_waiting() + len(self.queues['ramp'])
        assert sum(self.arrivals.values()) == self.exits + in_system, 'Conservation failure'
        assert {c.id for c in self.ramp.cars}.issubset({vid for _,vid in self.queues['ramp']})
        assert not ({c.id for c in self.ramp.cars} & {c.id for c in self.vehicles})
        assert all(c.v >= 0 for c in self.vehicles), 'Negative speed'

    # ------------------------------------------------------------ outputs

    def report_detectors(self, end_time):
        interval = self.c['detector_interval_s']
        for (p, lane), detector in self.detectors.items():
            self.observations.append({'sensor_id': f'loop_{p}_lane_{lane}', 'position_m': p, 'lane_id': lane,
                'interval_start_s': round(end_time - interval, 3), 'interval_end_s': round(end_time, 3),
                'count': detector.count,
                'mean_speed_mps': round(detector.speed_sum / detector.count, 4) if detector.count else None,
                'quality_flag': 'ok' if detector.count else 'no_crossings'})
            detector.count, detector.speed_sum = 0, 0.0

    def snapshot(self, t):
        """One replay frame for the HTML viewer."""
        self.frames.append({'time': round(t, 3), 'vehicles': [
            [c.id, round(c.x, 3), round(c.v, 3), c.origin, c.lane,
             c.previous_lane, round(c.lane_change_time, 3), c.id in self.requests, c.entered, c.entry_speed] for c in self.vehicles],
            'main_queue': self.main_waiting(), 'ramp_queue': len(self.queues['ramp']),
            'ramp_vehicles': self.ramp.snapshot(),
            'ramp_upstream': len(self.queues['ramp'])-len(self.ramp.cars),
            'road_queue': self.slow_vehicles(),
            'exits': self.exits, 'lane_changes': len(self.lane_changes),
            'mandatory_merges': self.mandatory_merges(),
            'yielding_vehicles': len(self.requests),
            'estimate': self.pipeline.estimate(t),
            'control': self.last_command.copy() if self.last_command else None})

    def main_waiting(self):
        return sum(len(self.queues[f'main_{i}']) for i in range(self.lanes))

    def slow_vehicles(self):
        return sum(c.v < self.c['queue_speed_threshold_mps'] for c in self.vehicles)

    def mandatory_merges(self):
        return sum(r['reason'] == 'closure_merge' for r in self.lane_changes)

    def result(self):
        # Evaluation-only truth: never supplied to the observation/controller modules.
        for row in self.forecasts_log:
            start = self.counter_history[round(row['issue_time_s'], 8)]
            end = self.counter_history.get(round(row['target_time_s'], 8))
            row['evaluation_period'] = (row['issue_time_s'] >= self.c['tmc']['evaluation_start_s']
                                        and end is not None)
            row['actual_value'] = end[row['segment_id']] - start[row['segment_id']] if end else None
            row['absolute_error'] = (abs(row['predicted_value'] - row['actual_value'])
                                     if end else None)
        scores = {}
        for method in ('moving_average', 'ema'):
            scores[method] = {}
            for channel in CHANNELS:
                rows = [r for r in self.forecasts_log if r['method'] == method
                        and r['segment_id'] == channel and r['evaluation_period']]
                scores[method][channel] = {'samples': len(rows),
                    'mae_veh': sum(r['absolute_error'] for r in rows) / len(rows) if rows else None}
        evaluated = [f for f in self.frames if f['time'] >= self.c['tmc']['evaluation_start_s']
                     and f['estimate']['road_vehicles'] is not None]
        metrics = {
            'mode': self.mode,
            'model_version': 'idm-freeway-ramp-motion-3.1' if self.c.get('taper_start_m') is not None else 'idm-freeway-2.0',
            'controller_version': 'forecast-inventory-meter-1.0',
            'forecast': scores,
            'fallback_time_s': round(self.fallback_time, 3),
            'road_estimate_mae_veh': (sum(abs(f['estimate']['road_vehicles'] - len(f['vehicles']))
                for f in evaluated) / len(evaluated) if evaluated else None),
            'ramp_estimate_mae_veh': (sum(abs(f['estimate']['ramp_queue'] - f['ramp_queue'])
                for f in evaluated) / len(evaluated) if evaluated else None),
            'generated_vehicles': sum(self.arrivals.values()),
            'generated_main': self.arrivals['main'],
            'generated_ramp': self.arrivals['ramp'],
            'exited_vehicles': self.exits,
            'remaining_on_road': len(self.vehicles),
            'remaining_main_queue': self.main_waiting(),
            'remaining_ramp_queue': len(self.queues['ramp']),
            'max_main_queue': self.max_main,
            'max_ramp_queue': self.max_ramp,
            'max_road_queue': self.max_road,
            'throughput_vph': self.exits * 3600 / self.c['duration_s'],
            'total_time_in_system_vehicle_s': round(self.total_time, 2),
            'total_delay_vehicle_s': round(self.total_time - self.free_time - self.ramp.distance/self.ramp.speed, 2),
            'minimum_gap_m': round(self.min_gap, 4) if self.min_gap != float('inf') else None,
            'strongest_braking_mps2': round(self.min_accel, 4),
            'safety_interventions': self.guards,
            'ramp_minimum_gap_m': round(self.ramp.minimum_gap,4) if self.ramp.minimum_gap != float('inf') else None,
            'ramp_distance_m': round(self.ramp.distance,4),
            'yield_records': len(self.yield_log),
            'lane_changes': len(self.lane_changes),
            'mandatory_merges': self.mandatory_merges(),
            'conservation_passed': True,    # check_invariants() asserts this every step
            'closed_lane_violations': 0,    # compute_updates() asserts this every step
        }
        return {'metrics': metrics, 'frames': self.frames, 'observations': self.observations,
                'traces': self.traces, 'control_log': self.control_log,
                'lane_changes': self.lane_changes, 'yield_log': self.yield_log,
                'boundary_observations': self.boundary_detectors.records,
                'estimates': self.estimates_log, 'forecasts': self.forecasts_log}


def simulate(config, mode='closure'):
    """Run one freeway case and return metrics, replay frames, and logs."""
    return FreewaySimulation(config, mode).run()


def validate(c):
    if c['dt_s'] <= 0 or c['duration_s'] <= 0 or c['lanes'] < 2:
        raise ValueError('Positive time parameters and at least two lanes required')
    for name in ['duration_s', 'frame_interval_s', 'detector_interval_s', 'lane_change_interval_s']:
        n = c[name] / c['dt_s']
        if abs(n - round(n)) > 1e-6 or n < 1:
            raise ValueError(name + ' must be a positive multiple of dt_s')
    if abs(c['duration_s'] / c['detector_interval_s'] - round(c['duration_s'] / c['detector_interval_s'])) > 1e-6:
        raise ValueError('duration must cover complete detector intervals')
    if not (0 < c['merge_position_m'] < c['merge_anticipation_m'] < c['closure_start_m']
            < c['closure_end_m'] < c['road_length_m']):
        raise ValueError('Require ramp < merge anticipation < closure start < closure end < road end')
    if c['closed_lane'] != 0:
        raise ValueError('This prototype implements leftmost-lane closure only')

    if c.get('taper_start_m') is not None:
        if not c['merge_anticipation_m'] <= c['taper_start_m'] < c['closure_start_m']:
            raise ValueError('Require merge anticipation <= taper start < closure start')
        for key in ('lane_width_m', 'vehicle_width_m', 'taper_clearance_m', 'merge_urgency_gain_mps2'):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError('Taper parameter must be finite and positive: ' + key)
        if c['vehicle_width_m'] + 2 * c['taper_clearance_m'] >= c['lane_width_m']:
            raise ValueError('Vehicle and side margins must fit in a full lane')
        if not 0 <= c['courtesy_start_urgency'] < c['mandatory_merge_urgency'] <= 1:
            raise ValueError('Invalid taper urgency thresholds')

    for key,default in [('ramp_length_m',200),('ramp_desired_speed_mps',12)]:
        if not math.isfinite(c.get(key,default)) or c.get(key,default) <= 0:
            raise ValueError('Ramp parameter must be finite and positive: '+key)
    if c.get('ramp_length_m',200) < 10:
        raise ValueError('Ramp must be at least 10 m long')
    tmc = c['tmc']
    for key in ('forecast_horizon_s', 'stale_after_s', 'target_road_vehicles',
                'min_release_interval_s', 'max_release_interval_s', 'ramp_queue_threshold_veh'):
        if not math.isfinite(tmc[key]) or tmc[key] <= 0:
            raise ValueError('TMC parameter must be finite and positive: ' + key)
    if not 0 < tmc['ema_alpha'] <= 1 or not isinstance(tmc['moving_average_intervals'], int) or tmc['moving_average_intervals'] < 1:
        raise ValueError('Invalid forecasting parameters')
    if not tmc['min_release_interval_s'] <= c['ramp_release_interval_s'] <= tmc['max_release_interval_s']:
        raise ValueError('Fallback interval must be within meter bounds')
    ratio = tmc['forecast_horizon_s'] / c['dt_s']
    if abs(ratio - round(ratio)) > 1e-6:
        raise ValueError('Forecast horizon must align with simulation steps')
    for fault in tmc['faults']:
        if fault['kind'] not in ('missing', 'delay') or not 0 <= fault['start_s'] < fault['end_s']:
            raise ValueError('Invalid observation fault')
        delay = fault.get('delay_s', 0)
        if delay < 0 or abs(delay / c['dt_s'] - round(delay / c['dt_s'])) > 1e-6:
            raise ValueError('Detector delay must align with simulation steps')
