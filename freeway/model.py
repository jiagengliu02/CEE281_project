"""Multi-lane IDM freeway with a physical work-zone closure, SI units.
Lane change: MOBIL-inspired incentive/safety plus explicit mandatory merging.
No speed-reduction zone. Lane 0 is the leftmost lane.
"""
from dataclasses import dataclass
import math


@dataclass
class Vehicle:
    id: int
    origin: str
    arrival: float
    entered: float
    x: float
    v: float
    lane: int
    a: float = 0.0
    length: float = 5.0
    desired_speed: float = 28.0
    headway: float = 1.5
    min_gap: float = 2.0
    max_accel: float = 1.2
    comfortable_braking: float = 2.0
    previous_lane: int = -1
    lane_change_time: float = -100.0


def idm_acceleration(v, gap, closing_speed, desired_speed, headway=1.5,
                     min_gap=2.0, max_accel=1.2, comfortable_braking=2.0):
    desired_gap = min_gap + max(0.0, v * headway +
        v * closing_speed / (2 * math.sqrt(max_accel * comfortable_braking)))
    return max_accel * (1 - (v / desired_speed) ** 4 -
                       (desired_gap / max(gap, 0.01)) ** 2)


def motion(x, v, a, dt):
    if a < 0 and v + a * dt < 0:
        return x - v * v / (2 * a), 0.0
    return x + v * dt + .5 * a * dt * dt, max(0.0, v + a * dt)


def lane_neighbors(vehicles, lane, x, exclude=None):
    others = [c for c in vehicles if c.lane == lane and c is not exclude]
    ahead = [c for c in others if c.x >= x]
    behind = [c for c in others if c.x < x]
    return (min(ahead, key=lambda c: c.x) if ahead else None,
            max(behind, key=lambda c: c.x) if behind else None)


def blocked(config, lane, x, enabled=True):
    return enabled and lane == config['closed_lane'] and config['closure_start_m'] <= x < config['closure_end_m'] + 5


def raw_acceleration(car, lead, lane, config, closure):
    gap = lead.x - lead.length - car.x if lead else 1e9
    lead_v = lead.v if lead else car.v
    # Physical lane-ending obstacle, not a changed desired speed.
    if closure and lane == config['closed_lane'] and car.x < config['closure_start_m']:
        obstacle_gap = config['closure_start_m'] - car.x
        if obstacle_gap < gap:
            gap, lead_v = obstacle_gap, 0.0
    return idm_acceleration(car.v, gap, car.v - lead_v, car.desired_speed,
                            car.headway, car.min_gap, car.max_accel, car.comfortable_braking)


def lane_change_candidate(car, target, vehicles, config, closure, mandatory=False):
    if not 0 <= target < config['lanes'] or blocked(config, target, car.x, closure):
        return None
    # Drivers do not voluntarily enter the lane that is about to end.
    if closure and target == config['closed_lane'] and config['merge_anticipation_m'] - 100 <= car.x < config['closure_end_m'] + car.length:
        return None
    new_lead, new_follow = lane_neighbors(vehicles, target, car.x, car)
    if new_lead and new_lead.x - new_lead.length - car.x < config['lane_change_min_gap_m']:
        return None
    if new_follow and car.x - car.length - new_follow.x < config['lane_change_min_gap_m']:
        return None
    new_a = raw_acceleration(car, new_lead, target, config, closure)
    follow_after = raw_acceleration(new_follow, car, target, config, closure) if new_follow else 0
    # Safety uses UNCLIPPED IDM acceleration; braking limits cannot conceal unsafe insertion.
    if new_a < -config['lane_change_safe_braking_mps2'] or follow_after < -config['lane_change_safe_braking_mps2']:
        return None
    old_lead, old_follow = lane_neighbors(vehicles, car.lane, car.x, car)
    old_a = raw_acceleration(car, old_lead, car.lane, config, closure)
    old_follow_before = raw_acceleration(old_follow, car, car.lane, config, closure) if old_follow else 0
    old_follow_after = raw_acceleration(old_follow, old_lead, car.lane, config, closure) if old_follow else 0
    new_follow_before = raw_acceleration(new_follow, new_lead, target, config, closure) if new_follow else 0
    incentive = new_a - old_a + config['politeness'] * (
        old_follow_after - old_follow_before + follow_after - new_follow_before)
    incentive += config['keep_right_bias_mps2'] * (1 if target > car.lane else -1)
    if mandatory or incentive > config['lane_change_threshold_mps2']:
        return {'score': incentive, 'self_after': new_a, 'follower_after': follow_after}
    return None


def courtesy_requests(vehicles, config, closure, owed_passage=None):
    """Near the lane end, one merging driver requests a target follower to open a gap.

    This is a declared behavioral assumption, not a calibrated zipper model.
    The nearest target-lane vehicle behind the foremost merging vehicle yields.
    """
    if not closure or not config.get('cooperative_merge', True):
        return {}
    merging = sorted([c for c in vehicles if c.lane == config['closed_lane'] and
        config['merge_anticipation_m'] <= c.x < config['closure_start_m']],
        key=lambda c: c.x, reverse=True)
    if not merging:
        return {}
    car = merging[0]
    _, follower = lane_neighbors(vehicles, car.lane + 1, car.x)
    if follower and follower.id not in (owed_passage or set()) and car.x - follower.x <= config['yield_lookahead_m']:
        return {follower.id: car}
    return {}


def simulate(config, mode='closure'):
    validate(config)
    closure = mode != 'open'
    dt, duration = config['dt_s'], config['duration_s']
    length, merge = config['road_length_m'], config['merge_position_m']
    vehicles, frames, traces, observations, controls, lane_changes = [], [], [], [], [], []
    queues = {f'main_{i}': [] for i in range(config['lanes'])}
    queues['ramp'] = []
    credit, arrivals = {'main': 0.0, 'ramp': 0.0}, {'main': 0, 'ramp': 0}
    exits, next_release = 0, 0.0
    total_time, free_time = 0.0, 0.0
    min_gap, min_accel, guards = float('inf'), 0.0, 0
    max_main, max_ramp, max_road = 0, 0, 0
    requests, yield_log, owed_passage = {}, [], set()
    interval = config['detector_interval_s']
    detectors = {(p, lane): [0, 0.0] for p in config['detectors_m'] for lane in range(config['lanes'])}
    sample_steps = round(config['frame_interval_s'] / dt)
    lc_steps = round(config['lane_change_interval_s'] / dt)
    obs_steps = round(interval / dt)

    def main_waiting():
        return sum(len(queues[f'main_{i}']) for i in range(config['lanes']))

    def snapshot(t):
        frames.append({'time': round(t, 3), 'vehicles': [
            [c.id, round(c.x, 3), round(c.v, 3), c.origin, c.lane,
             c.previous_lane, round(c.lane_change_time, 3), c.id in requests] for c in vehicles],
            'main_queue': main_waiting(), 'ramp_queue': len(queues['ramp']),
            'road_queue': sum(c.v < config['queue_speed_threshold_mps'] for c in vehicles),
            'exits': exits, 'lane_changes': len(lane_changes),
            'mandatory_merges': sum(r['reason'] == 'closure_merge' for r in lane_changes),
            'yielding_vehicles': len(requests)})

    def insert(origin, lane, t):
        queue_key = f'main_{lane}' if origin == 'main' else 'ramp'
        if not queues[queue_key]:
            return False
        x = 0.0 if origin == 'main' else merge
        lead, follow = lane_neighbors(vehicles, lane, x)
        born, vehicle_id = queues[queue_key][0]
        preferred = config['desired_speed_mps'] + config['speed_variation_mps'] * ((vehicle_id % 5) - 2) / 2
        speed = min(preferred, lead.v if lead else preferred)
        if lead and lead.x - lead.length - x < 2 + config['merge_headway_s'] * speed:
            return False
        if follow and x - 5 - follow.x < 2 + config['merge_headway_s'] * follow.v + max(0, follow.v - speed) * 2:
            return False
        if follow:
            candidate = Vehicle(vehicle_id, origin, born, t, x, speed, lane, desired_speed=preferred)
            if raw_acceleration(follow, candidate, lane, config, closure) < -config['lane_change_safe_braking_mps2']:
                return False
        queues[queue_key].pop(0)
        vehicles.append(Vehicle(vehicle_id, origin, born, t, x, speed, lane,
                                desired_speed=preferred, previous_lane=lane))
        return True

    snapshot(0)
    for step in range(round(duration / dt)):
        t = step * dt
        demand = config['demand'][0]
        for d in config['demand']:
            if t >= d['start_s']:
                demand = d
        for origin in credit:
            credit[origin] += demand[origin + '_vph'] * dt / 3600
            while credit[origin] >= 1 - 1e-10:
                if origin == 'main':
                    queues[f'main_{arrivals[origin] % config["lanes"]}'].append((t, sum(arrivals.values())))
                else:
                    queues['ramp'].append((t, sum(arrivals.values())))
                arrivals[origin] += 1
                credit[origin] -= 1
        for lane in range(config['lanes']):
            insert('main', lane, t)
        eligible = mode != 'fixed' or t >= next_release - 1e-9
        released = eligible and insert('ramp', config['lanes'] - 1, t)
        if released:
            next_release = t + config['ramp_release_interval_s']
        if step % sample_steps == 0 or released:
            controls.append({'issue_time_s': round(t, 3), 'actuator_id': 'ramp',
                'action': 'release' if released else 'hold', 'waiting_vehicles': len(queues['ramp']),
                'reason': 'safe_gap_release' if released else ('no_safe_gap_or_no_demand' if eligible else 'meter_wait'),
                'scenario': mode})
        if step % lc_steps == 0:
            # Sequential reservation: after every accepted move, later cars see updated
            # lane occupancy. No stale parallel gap claims or same-car double changes.
            priority = sorted(vehicles, key=lambda c: (
                not (closure and c.lane == config['closed_lane'] and config['merge_anticipation_m'] <= c.x < config['closure_start_m']), -c.x, c.id))
            for car in priority:
                if t - car.lane_change_time < config['lane_change_cooldown_s']:
                    continue
                mandatory = closure and car.lane == config['closed_lane'] and config['merge_anticipation_m'] <= car.x < config['closure_start_m']
                targets = [car.lane + 1] if mandatory else [car.lane - 1, car.lane + 1]
                choices = []
                for target in targets:
                    choice = lane_change_candidate(car, target, vehicles, config, closure, mandatory)
                    if choice is not None:
                        choices.append((choice['score'], target, choice))
                if choices:
                    _, target, choice = max(choices, key=lambda x: x[0])
                    if mandatory:
                        # After letting one closing-lane car in, the yielding car
                        # gets its own turn; never hold it for every subsequent car.
                        owed_passage.update(vid for vid, source in requests.items() if source.id == car.id)
                    old = car.lane
                    car.previous_lane, car.lane, car.lane_change_time = old, target, t
                    lane_changes.append({'time_s': round(t, 3), 'vehicle_id': car.id,
                        'position_m': round(car.x, 4), 'from_lane': old, 'to_lane': target,
                        'reason': 'closure_merge' if mandatory else 'incentive',
                        'self_acceleration_after_mps2': round(choice['self_after'], 4),
                        'target_follower_acceleration_after_mps2': round(choice['follower_after'], 4)})
        owed_passage.intersection_update(c.id for c in vehicles if c.x < config['closure_start_m'] + 5)
        requests = courtesy_requests(vehicles, config, closure, owed_passage)
        updates = {}
        for lane in range(config['lanes']):
            ordered = sorted([c for c in vehicles if c.lane == lane], key=lambda c: c.x, reverse=True)
            for i, car in enumerate(ordered):
                lead = ordered[i - 1] if i else None
                raw_a = raw_acceleration(car, lead, lane, config, closure)
                if car.id in requests:
                    merging_car = requests[car.id]
                    yield_a = idm_acceleration(car.v, merging_car.x - merging_car.length - car.x,
                        car.v - merging_car.v, car.desired_speed, car.headway,
                        car.min_gap + config['yield_gap_buffer_m'], car.max_accel, car.comfortable_braking)
                    # Moderate cooperation can reduce speed to create a real gap;
                    # it never overrides more urgent braking for the actual leader.
                    raw_a = min(raw_a, max(-config['yield_braking_mps2'], yield_a))
                    if step % sample_steps == 0:
                        yield_log.append({'time_s': round(t, 3), 'yielding_vehicle_id': car.id,
                            'merging_vehicle_id': merging_car.id, 'position_m': round(car.x, 4),
                            'requested_acceleration_mps2': round(yield_a, 4),
                            'applied_acceleration_mps2': round(raw_a, 4)})
                a = max(-config['max_braking_mps2'], min(car.max_accel, raw_a))
                nx, nv = motion(car.x, car.v, a, dt)
                ceiling = updates[lead.id][0] - lead.length - .01 if lead else float('inf')
                if closure and lane == config['closed_lane'] and car.x < config['closure_start_m']:
                    ceiling = min(ceiling, config['closure_start_m'] - .01)
                if nx > ceiling:
                    if ceiling < car.x:
                        raise AssertionError('Existing overlap cannot be repaired by a guard')
                    nx, nv = ceiling, 0.0
                    guards += 1
                if lead:
                    min_gap = min(min_gap, updates[lead.id][0] - lead.length - nx)
                assert not blocked(config, lane, nx, closure), 'Vehicle entered closed lane'
                updates[car.id] = (nx, nv, a)
        total_time += (len(vehicles) + main_waiting() + len(queues['ramp'])) * dt
        for car in vehicles:
            nx, nv, a = updates[car.id]
            free_time += max(0, min(length, nx) - min(length, car.x)) / car.desired_speed
            for p in config['detectors_m']:
                if car.x < p <= nx:
                    rec = detectors[(p, car.lane)]
                    rec[0] += 1
                    rec[1] += math.sqrt(max(0, car.v ** 2 + 2 * a * (p - car.x)))
            traces.append({'time_s': round(t + dt, 3), 'vehicle_id': car.id,
                'origin': car.origin, 'lane_id': car.lane, 'position_m': round(nx, 4),
                'speed_mps': round(nv, 4), 'acceleration_mps2': round(a, 4)})
            car.x, car.v, car.a = nx, nv, a
            min_accel = min(min_accel, a)
        exits += sum(c.x >= length for c in vehicles)
        vehicles = [c for c in vehicles if c.x < length]
        max_main, max_ramp = max(max_main, main_waiting()), max(max_ramp, len(queues['ramp']))
        max_road = max(max_road, sum(c.v < config['queue_speed_threshold_mps'] for c in vehicles))
        assert sum(arrivals.values()) == exits + len(vehicles) + main_waiting() + len(queues['ramp']), 'Conservation failure'
        assert all(c.v >= 0 for c in vehicles), 'Negative speed'
        if (step + 1) % obs_steps == 0:
            for (p, lane), rec in detectors.items():
                observations.append({'sensor_id': f'loop_{p}_lane_{lane}', 'position_m': p, 'lane_id': lane,
                    'interval_start_s': round(t + dt - interval, 3), 'interval_end_s': round(t + dt, 3),
                    'count': rec[0], 'mean_speed_mps': round(rec[1] / rec[0], 4) if rec[0] else None,
                    'quality_flag': 'ok' if rec[0] else 'no_crossings'})
                rec[0], rec[1] = 0, 0.0
        if (step + 1) % sample_steps == 0:
            snapshot(t + dt)
    metrics = {'mode': mode, 'generated_vehicles': sum(arrivals.values()), 'generated_main': arrivals['main'],
        'generated_ramp': arrivals['ramp'], 'exited_vehicles': exits, 'remaining_on_road': len(vehicles),
        'remaining_main_queue': main_waiting(), 'remaining_ramp_queue': len(queues['ramp']),
        'max_main_queue': max_main, 'max_ramp_queue': max_ramp, 'max_road_queue': max_road,
        'throughput_vph': exits * 3600 / duration,
        'total_time_in_system_vehicle_s': round(total_time, 2),
        'total_delay_vehicle_s': round(total_time - free_time, 2),
        'minimum_gap_m': round(min_gap, 4) if min_gap != float('inf') else None,
        'strongest_braking_mps2': round(min_accel, 4), 'safety_interventions': guards,
        'yield_records': len(yield_log),
        'lane_changes': len(lane_changes), 'mandatory_merges': sum(r['reason'] == 'closure_merge' for r in lane_changes),
        'conservation_passed': True, 'closed_lane_violations': 0}
    return {'metrics': metrics, 'frames': frames, 'observations': observations,
            'traces': traces, 'control_log': controls, 'lane_changes': lane_changes, 'yield_log': yield_log}


def validate(c):
    if c['dt_s'] <= 0 or c['duration_s'] <= 0 or c['lanes'] < 2:
        raise ValueError('Positive time parameters and at least two lanes required')
    for name in ['duration_s', 'frame_interval_s', 'detector_interval_s', 'lane_change_interval_s']:
        n = c[name] / c['dt_s']
        if abs(n - round(n)) > 1e-6 or n < 1:
            raise ValueError(name + ' must be a positive multiple of dt_s')
    if abs(c['duration_s'] / c['detector_interval_s'] - round(c['duration_s'] / c['detector_interval_s'])) > 1e-6:
        raise ValueError('duration must cover complete detector intervals')
    if not 0 < c['merge_position_m'] < c['merge_anticipation_m'] < c['closure_start_m'] < c['closure_end_m'] < c['road_length_m']:
        raise ValueError('Require ramp < merge anticipation < closure start < closure end < road end')
    if c['closed_lane'] != 0:
        raise ValueError('This prototype implements leftmost-lane closure only')
