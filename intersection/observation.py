"""Turns detector reports into queue estimates and short-term arrival forecasts.

Uses paired entry/stop-line counters, never queue truth:
    estimated queue = cumulative entered - cumulative departed
Cumulative counters let the estimate recover after a missing report. Missing data
stay missing, and a late (out-of-order) report never rewinds the state.
"""
from collections import deque

from .detector import APPROACHES  # noqa: F401  (re-exported for callers)


class ObservationPipeline:
    def __init__(self, config):
        self.c = config
        self.state = {a: {'end': None,              # end time of the newest report used
                          'received': None,
                          'entered': 0, 'departed': 0, 'queue': 0,
                          'missing_end': -1,         # end time of the newest missing report
                          'ema': None,
                          'rates': deque(maxlen=config['moving_average_intervals'])}
                      for a in APPROACHES}
        self.forecasts = []

    def receive(self, row, now):
        """Ingest one detector report delivered at time `now`."""
        s = self.state[row['approach']]
        end = row['interval_end_s']
        if row['quality_flag'] == 'missing':
            s['missing_end'] = max(s['missing_end'], end)
            return
        if s['end'] is not None and end <= s['end']:
            return  # out-of-order delayed report
        # Arrival rate over everything since the last good report (spans any gap).
        previous_end = s['end'] or 0.0
        rate = (row['cumulative_entered'] - s['entered']) / (end - previous_end)
        alpha = self.c['ema_alpha']
        s['ema'] = rate if s['ema'] is None else alpha * rate + (1 - alpha) * s['ema']
        s['rates'].append(rate)
        s.update(end=end, received=now, entered=row['cumulative_entered'],
                 departed=row['cumulative_departed'],
                 queue=max(0, row['cumulative_entered'] - row['cumulative_departed']))

    def estimates(self, now):
        """Current queue estimate and data quality for every approach."""
        result = {}
        for a, s in self.state.items():
            age = now - s['end'] if s['end'] is not None else None
            if age is None:
                quality = 'unavailable'
            elif s['missing_end'] >= s['end']:
                quality = 'missing'
            elif age > self.c['stale_after_s']:
                quality = 'stale'
            else:
                quality = 'ok'
            result[a] = {'issue_time_s': now, 'target_time_s': now, 'segment_id': a,
                         'estimated_queue_veh': s['queue'], 'units': 'veh',
                         'quality_flag': quality, 'observation_age_s': age}
        return result

    def predict(self, now):
        """Forecast admitted arrivals over the next horizon (moving average vs. EMA).

        Advisory only: the controller does not use these forecasts.
        """
        horizon = self.c['forecast_horizon_s']
        estimates = self.estimates(now)
        for a, s in self.state.items():
            if estimates[a]['quality_flag'] != 'ok' or not s['rates']:
                continue
            moving_average = sum(s['rates']) / len(s['rates'])
            for method, rate in [('moving_average', moving_average), ('ema', s['ema'])]:
                self.forecasts.append({'issue_time_s': now, 'target_time_s': now + horizon,
                    'segment_id': a, 'variable_name': 'admitted_arrivals_in_horizon',
                    'method': method, 'predicted_value': rate * horizon, 'units': 'veh',
                    'quality_flag': 'ok'})
