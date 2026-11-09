"""Approach loop detectors, causal report delivery, queue estimates and arrival forecasts.

Each approach has two virtual inductive loops: an advance loop `advance_detector_m`
upstream of the stop line and a stop-line loop. Every `detector_interval_s` they report
crossing counts, crossing-mean speeds and cumulative counts. Reports can be missing or
delayed (declared faults). The controller receives reports only; true queues, vehicle
positions and future arrivals never enter this module.
"""
from collections import deque
import heapq

APPROACHES = ('N', 'S', 'E', 'W')
GROUPS = {'NS': ('N', 'S'), 'EW': ('E', 'W')}
SENSORS = ('advance', 'stopline')


def sensor_id(approach, sensor):
    return f'{approach}_{sensor}'


class ApproachDetectors:
    """Virtual loops: count crossings and crossing speeds, report at fixed intervals."""

    def __init__(self, interval, positions, faults=()):
        self.interval, self.positions, self.faults = interval, positions, faults
        self.counts = {sensor_id(a, s): 0 for a in APPROACHES for s in SENSORS}
        self.speed_sums = dict.fromkeys(self.counts, 0.0)
        self.previous = self.counts.copy()
        self.pending, self.records = [], []
        self.serial = 0

    def cross(self, approach, sensor, speed):
        key = sensor_id(approach, sensor)
        self.counts[key] += 1
        self.speed_sums[key] += speed

    def report(self, end):
        for approach in APPROACHES:
            fault = next((f for f in self.faults if f['approach'] == approach
                          and f['start_s'] <= end < f['end_s']), None)
            missing = bool(fault and fault['kind'] == 'missing')
            available = end + (fault['delay_s'] if fault and fault['kind'] == 'delay' else 0)
            for sensor in SENSORS:
                key = sensor_id(approach, sensor)
                count = self.counts[key] - self.previous[key]
                row = {'sensor_id': key, 'approach': approach, 'position_m': self.positions[sensor],
                       'interval_start_s': round(end - self.interval, 3), 'interval_end_s': round(end, 3),
                       'available_time_s': round(available, 3),
                       'count': None if missing else count,
                       'cumulative_count': None if missing else self.counts[key],
                       'mean_speed_mps': (None if missing or not count
                                          else round(self.speed_sums[key] / count, 4)),
                       'quality_flag': 'missing' if missing else ('ok' if count else 'no_crossings')}
                self.records.append(row)
                heapq.heappush(self.pending, (available, self.serial, row))
                self.serial += 1
        self.previous = self.counts.copy()
        self.speed_sums = dict.fromkeys(self.counts, 0.0)

    def deliver(self, now):
        while self.pending and self.pending[0][0] <= now + 1e-9:
            yield heapq.heappop(self.pending)[2]


class SignalObservation:
    """Queue estimate per approach from cumulative loop counts, plus arrival forecasts.

    estimated_queue = cumulative advance crossings - cumulative stop-line crossings,
    i.e. vehicles between the advance loop and the stop line (moving or stopped).
    Vehicles queued upstream of the advance loop are invisible to the controller.
    """

    def __init__(self, config):
        self.c = config
        self.state = {a: {s: {'end': 0.0, 'count': 0, 'seen': False, 'missing': -1.0,
                              'last_count': 0, 'ema': 0.0,
                              'rates': deque(maxlen=config['moving_average_intervals'])}
                          for s in SENSORS} for a in APPROACHES}

    def receive(self, row, now):
        if row['available_time_s'] > now + 1e-9:
            raise ValueError('Cannot consume a report before delivery')
        s = self.state[row['approach']][row['sensor_id'].split('_')[1]]
        end = row['interval_end_s']
        if row['quality_flag'] == 'missing':
            s['missing'] = max(s['missing'], end)
            return False
        if end <= s['end']:
            return False                       # an older delayed report never overwrites newer data
        rate = (row['cumulative_count'] - s['count']) / (end - s['end'])
        if rate < 0:
            raise ValueError('Cumulative counters must not decrease')
        alpha = self.c['ema_alpha']
        s['ema'] = alpha * rate + (1 - alpha) * s['ema'] if s['seen'] else rate
        s['rates'].append(rate)
        s.update(end=end, count=row['cumulative_count'], seen=True, last_count=row['count'])
        return True

    def quality(self, approach, now):
        states = self.state[approach].values()
        if not all(s['seen'] for s in states):
            return 'unavailable'
        if any(s['missing'] >= s['end'] for s in states):
            return 'missing'
        if any(now - s['end'] > self.c['stale_after_s'] + 1e-9 for s in states):
            return 'stale'
        return 'ok'

    def estimate(self, now, approach):
        s = self.state[approach]
        quality = self.quality(approach, now)
        available = quality != 'unavailable'
        return {'issue_time_s': now, 'target_time_s': now, 'segment_id': approach,
                'variable_name': 'vehicles_between_advance_loop_and_stop_line',
                'estimated_queue_veh': (max(0, s['advance']['count'] - s['stopline']['count'])
                                        if available else None),
                'recent_arrivals_veh': s['advance']['last_count'] if available else None,
                'units': 'veh', 'quality_flag': quality,
                'observation_end_s': min(x['end'] for x in s.values()) if available else None}

    def estimates(self, now):
        return {a: self.estimate(now, a) for a in APPROACHES}

    def forecast(self, now, approach, method='ema'):
        """Predicted advance-loop arrivals over the next horizon; None without healthy data."""
        if self.quality(approach, now) != 'ok':
            return None
        s = self.state[approach]['advance']
        rate = s['ema'] if method == 'ema' else sum(s['rates']) / len(s['rates'])
        horizon = self.c['forecast_horizon_s']
        return {'issue_time_s': now, 'target_time_s': now + horizon, 'segment_id': approach,
                'variable_name': 'advance_loop_arrivals', 'method': method,
                'predicted_value': rate * horizon, 'units': 'veh', 'quality_flag': 'ok'}

    def forecasts(self, now, method='ema'):
        return {a: self.forecast(now, a, method) for a in APPROACHES}
