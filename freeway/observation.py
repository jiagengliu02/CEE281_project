"""Boundary counters, causal delivery, conservation estimates and flow forecasts.

The controller receives reports only. Simulator queues, trajectories and future
arrivals are never passed to this module. Counters assume an initially empty road.
"""
from collections import deque
import heapq

CHANNELS = ('main_arrivals', 'main_entries', 'ramp_arrivals', 'ramp_entries', 'exits')


class BoundaryDetectors:
    def __init__(self, interval, faults=()):
        self.interval, self.faults = interval, faults
        self.counts = dict.fromkeys(CHANNELS, 0)
        self.previous = self.counts.copy()
        self.pending, self.records = [], []
        self.serial = 0

    def cross(self, channel):
        self.counts[channel] += 1

    def report(self, end):
        fault = next((f for f in self.faults if f['start_s'] <= end < f['end_s']), None)
        missing = fault and fault['kind'] == 'missing'
        available = end + (fault['delay_s'] if fault and fault['kind'] == 'delay' else 0)
        for channel in CHANNELS:
            row = {'sensor_id': channel, 'interval_start_s': end - self.interval,
                   'interval_end_s': end, 'available_time_s': available,
                   'count': None if missing else self.counts[channel] - self.previous[channel],
                   'cumulative_count': None if missing else self.counts[channel],
                   'mean_speed_mps': None, 'quality_flag': 'missing' if missing else 'ok'}
            self.records.append(row)
            heapq.heappush(self.pending, (available, self.serial, row))
            self.serial += 1
        self.previous = self.counts.copy()

    def deliver(self, now):
        while self.pending and self.pending[0][0] <= now + 1e-9:
            yield heapq.heappop(self.pending)[2]


class RampObservation:
    def __init__(self, config):
        self.c = config
        self.state = {key: {'end': 0.0, 'count': 0, 'seen': False, 'missing': -1,
                            'ema': 0.0, 'rates': deque(maxlen=config['moving_average_intervals'])}
                      for key in CHANNELS}

    def receive(self, row, now):
        if row['available_time_s'] > now + 1e-9:
            raise ValueError('Cannot consume a report before delivery')
        s = self.state[row['sensor_id']]
        end = row['interval_end_s']
        if row['quality_flag'] != 'ok':
            s['missing'] = max(s['missing'], end)
            return False
        if end <= s['end']:
            return False
        rate = (row['cumulative_count'] - s['count']) / (end - s['end'])
        if rate < 0:
            raise ValueError('Cumulative counters must not decrease')
        alpha = self.c['ema_alpha']
        s['ema'] = alpha * rate + (1 - alpha) * s['ema'] if s['seen'] else rate
        s['rates'].append(rate)
        s.update(end=end, count=row['cumulative_count'], seen=True)
        return True

    def estimate(self, now):
        states = list(self.state.values())
        if not all(s['seen'] for s in states):
            quality = 'unavailable'
        elif any(s['missing'] >= s['end'] for s in states):
            quality = 'missing'
        elif any(now - s['end'] > self.c['stale_after_s'] + 1e-9 for s in states):
            quality = 'stale'
        else:
            quality = 'ok'
        n = {k: s['count'] for k, s in self.state.items()}
        available = all(s['seen'] for s in states)
        return {'issue_time_s': now, 'target_time_s': now, 'segment_id': 'freeway_and_ramp',
                'road_vehicles': max(0, n['main_entries'] + n['ramp_entries'] - n['exits']) if available else None,
                'ramp_queue': max(0, n['ramp_arrivals'] - n['ramp_entries']) if available else None,
                'main_queue': max(0, n['main_arrivals'] - n['main_entries']) if available else None,
                'units': 'veh', 'quality_flag': quality,
                'observation_end_s': min(s['end'] for s in states) if available else None}

    def forecast(self, now, method='ema'):
        estimate = self.estimate(now)
        if estimate['quality_flag'] != 'ok':
            return None
        horizon = self.c['forecast_horizon_s']
        rates = {key: s['ema'] if method == 'ema' else sum(s['rates']) / len(s['rates'])
                 for key, s in self.state.items()}
        return {'issue_time_s': now, 'target_time_s': now + horizon, 'method': method,
                'counts': {k: v * horizon for k, v in rates.items()},
                'units': 'veh', 'quality_flag': 'ok'}
