"""Count-based observation adapter, causal queue estimator and arrival forecasts."""
from collections import deque

APPROACHES = ('N', 'S', 'E', 'W')


class ObservationPipeline:
    """Uses paired entry/stop-line counters, not queue truth.

    Cumulative counters allow recovery after a missing report. Rates spanning a gap
    average over that gap. Missing data stay missing; delayed reports never rewind state.
    Forecasts predict admitted arrivals, not hidden external demand, and are advisory.
    """
    def __init__(self, config):
        self.c = config
        self.state = {a: {'end': None, 'received': None, 'entered': 0, 'departed': 0,
                          'queue': 0, 'missing_end': -1, 'ema': None,
                          'rates': deque(maxlen=config['moving_average_intervals'])}
                      for a in APPROACHES}
        self.forecasts = []

    def receive(self, row, now):
        s = self.state[row['approach']]
        end = row['interval_end_s']
        if row['quality_flag'] == 'missing':
            s['missing_end'] = max(s['missing_end'], end)
            return
        if s['end'] is not None and end <= s['end']:
            return  # out-of-order delayed report
        previous_end = s['end'] or 0.0
        rate = (row['cumulative_entered'] - s['entered']) / (end - previous_end)
        alpha = self.c['ema_alpha']
        s['ema'] = rate if s['ema'] is None else alpha * rate + (1 - alpha) * s['ema']
        s['rates'].append(rate)
        s.update(end=end, received=now, entered=row['cumulative_entered'],
                 departed=row['cumulative_departed'],
                 queue=max(0, row['cumulative_entered'] - row['cumulative_departed']))

    def estimates(self, now):
        result = {}
        for a, s in self.state.items():
            age = now - s['end'] if s['end'] is not None else None
            quality = ('unavailable' if age is None else
                       'missing' if s['missing_end'] >= s['end'] else
                       'stale' if age > self.c['stale_after_s'] else 'ok')
            result[a] = {'issue_time_s': now, 'target_time_s': now, 'segment_id': a,
                         'estimated_queue_veh': s['queue'], 'units': 'veh',
                         'quality_flag': quality, 'observation_age_s': age}
        return result

    def predict(self, now):
        estimates = self.estimates(now)
        for a, s in self.state.items():
            if estimates[a]['quality_flag'] != 'ok' or not s['rates']:
                continue
            for method, rate in [('moving_average', sum(s['rates']) / len(s['rates'])),
                                 ('ema', s['ema'])]:
                self.forecasts.append({'issue_time_s': now,
                    'target_time_s': now + self.c['forecast_horizon_s'], 'segment_id': a,
                    'variable_name': 'admitted_arrivals_in_horizon', 'method': method,
                    'predicted_value': rate * self.c['forecast_horizon_s'], 'units': 'veh',
                    'quality_flag': 'ok'})
