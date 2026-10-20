"""Bounded forecast-assisted ramp meter; a project-defined teaching policy.

N_without_ramp = N_est + forecast(main_entries) - forecast(exits).
Desired rate = (target_road_vehicles - N_without_ramp) / horizon.
A soft ramp queue threshold can request faster service. Safe insertion still
belongs to the physical model and can delay an eligible release.
"""


class RampController:
    def __init__(self, config, mode, fixed_interval):
        self.c, self.mode, self.fixed = config, mode, fixed_interval
        self.last_release = None

    def decide(self, now, estimate, forecast):
        adaptive = self.mode in ('adaptive', 'adaptive_fault')
        fallback = adaptive and (estimate['quality_flag'] != 'ok' or forecast is None)
        interval = self.fixed
        projected, predicted_queue = None, None
        if self.mode in ('open', 'closure'):
            interval, reason = 0.0, 'unmetered'
        elif fallback:
            reason = 'fallback_fixed'
        elif not adaptive:
            reason = 'fixed_interval'
        else:
            h = self.c['forecast_horizon_s']
            counts = forecast['counts']
            projected = max(0, estimate['road_vehicles'] + counts['main_entries'] - counts['exits'])
            rate = (self.c['target_road_vehicles'] - projected) / h
            rate = max(1 / self.c['max_release_interval_s'],
                       min(1 / self.c['min_release_interval_s'], rate))
            predicted_queue = max(0, estimate['ramp_queue'] + counts['ramp_arrivals'] - rate * h)
            reason = 'forecast_road_inventory'
            if predicted_queue > self.c['ramp_queue_threshold_veh']:
                needed = (estimate['ramp_queue'] + counts['ramp_arrivals']
                          - self.c['ramp_queue_threshold_veh']) / h
                rate = min(1 / self.c['min_release_interval_s'], max(rate, needed))
                reason = 'ramp_queue_relief'
            interval = 1 / rate
            predicted_queue = max(0, estimate['ramp_queue'] + counts['ramp_arrivals'] - rate * h)
        eligible = self.last_release is None or now - self.last_release >= interval - 1e-9
        return {'issue_time_s': now, 'apply_time_s': now, 'actuator_id': 'ramp',
                'action': 'permit' if eligible else 'hold', 'value': interval, 'units': 's',
                'reason': reason, 'fallback': fallback, 'quality_flag': estimate['quality_flag'],
                'estimated_road_vehicles': estimate['road_vehicles'],
                'estimated_ramp_queue': estimate['ramp_queue'],
                'forecast_road_without_ramp_veh': projected,
                'forecast_ramp_queue_veh': predicted_queue,
                'forecast_ramp_arrivals_veh': forecast['counts']['ramp_arrivals'] if forecast else None,
                'forecast_horizon_s': self.c['forecast_horizon_s'],
                'minimum_interval_s': self.c['min_release_interval_s'] if adaptive else interval,
                'maximum_interval_s': self.c['max_release_interval_s'] if adaptive else interval}

    def executed(self, now):
        self.last_release = now
