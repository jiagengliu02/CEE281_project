"""Multi-lane freeway with an on-ramp and a left-lane work-zone closure.

Each time step runs, in order:
    arrivals -> road entry -> ramp control -> lane changes
    -> longitudinal update -> detectors and metrics.
Modes: 'open' (no closure), 'closure' (no metering), 'fixed' (closure + fixed ramp metering).
"""
from dataclasses import dataclass
import math

from .behavior import (VEHICLE_LENGTH_M, Vehicle, blocked, courtesy_requests, idm_acceleration, in_merge_zone,
                       lane_change_candidate, lane_neighbors, motion, raw_acceleration)

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
        self.credit = {'main': 0.0, 'ramp': 0.0}       # fractional arrivals carried over
        self.arrivals = {'main': 0, 'ramp': 0}
        self.next_release = 0.0                         # earliest metered ramp release
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
            t = step * self.dt
            self.generate_arrivals(t)
            for lane in range(self.lanes):
                self.try_enter('main', lane, t)
            self.control_ramp(step, t)
            if step % self.lane_change_steps == 0:
                self.change_lanes(t)
            self.owed_passage.intersection_update(
                c.id for c in self.vehicles if c.x < self.c['closure_start_m'] + OWED_PASSAGE_RANGE_M)
            self.requests = courtesy_requests(self.vehicles, self.c, self.closure, self.owed_passage)
            updates = self.compute_updates(step, t)
            self.apply_updates(updates, t)
            self.check_invariants()
            if (step + 1) % self.detector_steps == 0:
                self.report_detectors(t + self.dt)
            if (step + 1) % self.sample_steps == 0:
                self.snapshot(t + self.dt)
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
                self.credit[origin] -= 1

    def try_enter(self, origin, lane, t):
        """Move the first queued vehicle onto the road if the gaps are safe."""
        queue = self.queues[f'main_{lane}' if origin == 'main' else 'ramp']
        if not queue:
            return False
        x = 0.0 if origin == 'main' else self.c['merge_position_m']
        lead, follow = lane_neighbors(self.vehicles, lane, x)
        born, vehicle_id = queue[0]
        desired = preferred_speed(self.c, vehicle_id)
        speed = min(desired, lead.v if lead else desired)
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
        self.vehicles.append(Vehicle(vehicle_id, origin, born, t, x, speed, lane,
                                     desired_speed=desired, previous_lane=lane))
        return True

    def control_ramp(self, step, t):
        """Ramp entry; in 'fixed' mode releases are at least ramp_release_interval_s apart."""
        eligible = self.mode != 'fixed' or t >= self.next_release - 1e-9
        released = eligible and self.try_enter('ramp', self.lanes - 1, t)
        if released:
            self.next_release = t + self.c['ramp_release_interval_s']
        if step % self.sample_steps == 0 or released:
            if released:
                reason = 'safe_gap_release'
            elif eligible:
                reason = 'no_safe_gap_or_no_demand'
            else:
                reason = 'meter_wait'
            self.control_log.append({'issue_time_s': round(t, 3), 'actuator_id': 'ramp',
                'action': 'release' if released else 'hold',
                'waiting_vehicles': len(self.queues['ramp']), 'reason': reason, 'scenario': self.mode})

    # ------------------------------------------------------------ lane changes

    def change_lanes(self, t):
        """Evaluate lane changes one car at a time.

        Merging cars go first, then front to back. Each accepted move updates
        lane occupancy immediately, so two cars cannot claim the same gap.
        """
        priority = sorted(self.vehicles, key=lambda c: (
            not in_merge_zone(c, self.c, self.closure), -c.x, c.id))
        for car in priority:
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
        applied = min(accel, max(-self.c['yield_braking_mps2'], yield_a))
        if step % self.sample_steps == 0:
            self.yield_log.append({'time_s': round(t, 3), 'yielding_vehicle_id': car.id,
                'merging_vehicle_id': merger.id, 'position_m': round(car.x, 4),
                'requested_acceleration_mps2': round(yield_a, 4),
                'applied_acceleration_mps2': round(applied, 4)})
        return applied

    def guard(self, car, lead, lane, new_x, new_v, updates):
        """Counted numerical guard: never pass the leader's new rear or the lane end."""
        ceiling = updates[lead.id][0] - lead.length - GUARD_CLEARANCE_M if lead else float('inf')
        if self.closure and lane == self.c['closed_lane'] and car.x < self.c['closure_start_m']:
            ceiling = min(ceiling, self.c['closure_start_m'] - GUARD_CLEARANCE_M)
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
        self.exits += sum(c.x >= road_length for c in self.vehicles)
        self.vehicles = [c for c in self.vehicles if c.x < road_length]
        self.max_main = max(self.max_main, self.main_waiting())
        self.max_ramp = max(self.max_ramp, len(self.queues['ramp']))
        self.max_road = max(self.max_road, self.slow_vehicles())

    def check_invariants(self):
        in_system = len(self.vehicles) + self.main_waiting() + len(self.queues['ramp'])
        assert sum(self.arrivals.values()) == self.exits + in_system, 'Conservation failure'
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
             c.previous_lane, round(c.lane_change_time, 3), c.id in self.requests] for c in self.vehicles],
            'main_queue': self.main_waiting(), 'ramp_queue': len(self.queues['ramp']),
            'road_queue': self.slow_vehicles(),
            'exits': self.exits, 'lane_changes': len(self.lane_changes),
            'mandatory_merges': self.mandatory_merges(),
            'yielding_vehicles': len(self.requests)})

    def main_waiting(self):
        return sum(len(self.queues[f'main_{i}']) for i in range(self.lanes))

    def slow_vehicles(self):
        return sum(c.v < self.c['queue_speed_threshold_mps'] for c in self.vehicles)

    def mandatory_merges(self):
        return sum(r['reason'] == 'closure_merge' for r in self.lane_changes)

    def result(self):
        metrics = {
            'mode': self.mode,
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
            'total_delay_vehicle_s': round(self.total_time - self.free_time, 2),
            'minimum_gap_m': round(self.min_gap, 4) if self.min_gap != float('inf') else None,
            'strongest_braking_mps2': round(self.min_accel, 4),
            'safety_interventions': self.guards,
            'yield_records': len(self.yield_log),
            'lane_changes': len(self.lane_changes),
            'mandatory_merges': self.mandatory_merges(),
            'conservation_passed': True,    # check_invariants() asserts this every step
            'closed_lane_violations': 0,    # compute_updates() asserts this every step
        }
        return {'metrics': metrics, 'frames': self.frames, 'observations': self.observations,
                'traces': self.traces, 'control_log': self.control_log,
                'lane_changes': self.lane_changes, 'yield_log': self.yield_log}


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
