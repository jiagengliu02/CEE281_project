"""Two-phase signal controllers: a Webster fixed-time plan, vehicle actuation and a
queue-plus-forecast rule. All share one bounded phase state machine.

The controller sees estimates and forecasts from the observation pipeline only.
Every switch passes through yellow and all-red; minimum green is never cut short.
Maximum green applies whenever an opposing call exists; without a call the signal
rests in the current green (standard actuated behaviour), so no approach with
demand can be denied service indefinitely.
"""
from dataclasses import dataclass

from .observation import APPROACHES, GROUPS

MODES = ('fixed', 'actuated', 'adaptive', 'adaptive_no_forecast')
EPS = 1e-9


def other_group(group):
    return 'EW' if group == 'NS' else 'NS'


def webster_plan(config):
    """Fixed-time plan from Webster (1958) for the declared design demand.

    y_i = max(q/s) over the group's approaches; L = sum of lost time per phase;
    C0 = (1.5 L + 5) / (1 - Y); effective green g_i = (C0 - L) y_i / Y.
    Displayed green = g_i + startup lost time - yellow used, bounded by min/max green.
    The plan is designed once on design_demand_vph and never retuned to a test scenario.
    """
    plan = config['fixed_plan']
    s = config['saturation_flow_vph']
    lost = config['startup_lost_s'] + config['yellow_s'] + config['all_red_s'] - config['yellow_used_s']
    ratios = {g: max(plan['design_demand_vph'][a] / s for a in GROUPS[g]) for g in GROUPS}
    total_ratio = sum(ratios.values())
    if total_ratio >= 0.95:
        raise ValueError('Design demand too close to saturation for a Webster plan')
    total_lost = lost * len(GROUPS)
    cycle = (1.5 * total_lost + 5) / (1 - total_ratio)
    greens = {}
    for g, y in ratios.items():
        effective = (cycle - total_lost) * (y / total_ratio if total_ratio else 1 / len(GROUPS))
        displayed = effective + config['startup_lost_s'] - config['yellow_used_s']
        greens[g] = round(min(config['max_green_s'], max(config['min_green_s'], displayed)), 1)
    return {'method': 'webster', 'flow_ratios': ratios, 'lost_time_per_phase_s': lost,
            'optimal_cycle_s': round(cycle, 2), 'green_s': greens,
            'cycle_s': round(sum(greens.values())
                             + len(GROUPS) * (config['yellow_s'] + config['all_red_s']), 2)}


@dataclass
class SignalState:
    group: str = 'NS'
    stage: str = 'green'          # green -> yellow -> all_red -> other group's green
    started: float = 0.0

    def permits(self, approach):
        return self.stage == 'green' and approach in GROUPS[self.group]

    def phase_for(self, approach):
        """'green' | 'yellow' | 'red' as seen from one approach."""
        if approach in GROUPS[self.group] and self.stage in ('green', 'yellow'):
            return self.stage
        return 'red'


class SignalController:
    def __init__(self, config, mode):
        if mode not in MODES:
            raise ValueError('unknown controller mode')
        self.c, self.mode = config, mode
        self.k = config['control']
        self.plan = webster_plan(config)
        self.use_forecast = mode == 'adaptive'
        self.state = SignalState()

    # ------------------------------------------------------------ helpers

    def group_queue(self, group, estimates):
        return sum(estimates[a]['estimated_queue_veh'] for a in GROUPS[group])

    def group_forecast(self, group, forecasts):
        return sum(forecasts[a]['predicted_value'] for a in GROUPS[group])

    def recent_arrivals(self, group, estimates):
        return sum(estimates[a]['recent_arrivals_veh'] for a in GROUPS[group])

    def opposing_call(self, group, estimates):
        """A real actuated controller only times max green against a call."""
        return self.group_queue(group, estimates) > 0 or self.recent_arrivals(group, estimates) > 0

    # ------------------------------------------------------------ update

    def update(self, t, estimates, forecasts):
        s = self.state
        elapsed = t - s.started
        healthy = all(estimates[a]['quality_flag'] == 'ok' for a in APPROACHES)
        responsive = self.mode != 'fixed'
        fallback = responsive and (not healthy or (self.use_forecast and
                                                    any(forecasts[a] is None for a in APPROACHES)))
        if s.stage == 'green':
            switch, reason = self.green_decision(elapsed, estimates, forecasts, fallback)
            if switch:
                s.stage, s.started = 'yellow', t
            action = 'transition' if switch else 'hold'
        else:
            reason, action = self.clearance_step(elapsed, t)
        cur, oth = s.group, other_group(s.group)
        available = all(e['estimated_queue_veh'] is not None for e in estimates.values())
        f_ok = all(forecasts[a] is not None for a in APPROACHES)
        return {'issue_time_s': t, 'apply_time_s': t, 'actuator_id': 'signal',
                'action': action, 'value': f'{s.group}_{s.stage}', 'units': 'phase_state',
                'reason': reason, 'fallback': fallback, 'phase_elapsed_s': round(t - s.started, 3),
                'min_green_s': self.c['min_green_s'], 'max_green_s': self.c['max_green_s'],
                'fixed_green_s': self.plan['green_s'][cur],
                'current_group': cur,
                'current_queue_est_veh': self.group_queue(cur, estimates) if available else None,
                'opposing_queue_est_veh': self.group_queue(oth, estimates) if available else None,
                'current_forecast_veh': round(self.group_forecast(cur, forecasts), 3) if f_ok else None,
                'opposing_forecast_veh': round(self.group_forecast(oth, forecasts), 3) if f_ok else None,
                'quality_flags': '/'.join(estimates[a]['quality_flag'] for a in APPROACHES)}

    def clearance_step(self, elapsed, t):
        s, c = self.state, self.c
        if s.stage == 'yellow' and elapsed >= c['yellow_s'] - EPS:
            s.stage, s.started = 'all_red', t
            return 'yellow_complete', 'transition'
        if s.stage == 'all_red' and elapsed >= c['all_red_s'] - EPS:
            s.group, s.stage, s.started = other_group(s.group), 'green', t
            return 'clearance_complete', 'transition'
        return 'clearance_in_progress', 'hold'

    def green_decision(self, elapsed, estimates, forecasts, fallback):
        """Return (switch_now, reason) for the current green."""
        c, group = self.c, self.state.group
        if elapsed < c['min_green_s'] - EPS:
            return False, 'minimum_green'
        if self.mode == 'fixed' or fallback:
            switch = elapsed >= self.plan['green_s'][group] - EPS
            return switch, 'fallback_fixed' if fallback else 'fixed_plan'
        oth = other_group(group)
        if not self.opposing_call(oth, estimates):
            return False, 'rest_no_opposing_call'
        if elapsed >= c['max_green_s'] - EPS:
            return True, 'max_out'
        if self.mode == 'actuated':
            # Unit extension: keep green while the served approaches still deliver
            # arrivals at the advance loops; gap out when a whole report is empty.
            if self.recent_arrivals(group, estimates) > 0:
                return False, 'extend_green'
            return True, 'gap_out'
        # Pressure = estimated queue in the detector zone (+ forecast arrivals over the
        # horizon when enabled). Switch when the served approaches are empty and quiet,
        # or when the opposing pressure exceeds the served pressure by the threshold.
        cur_q, oth_q = self.group_queue(group, estimates), self.group_queue(oth, estimates)
        cur_f = self.group_forecast(group, forecasts) if self.use_forecast else 0.0
        oth_f = self.group_forecast(oth, forecasts) if self.use_forecast else 0.0
        if cur_q == 0 and self.recent_arrivals(group, estimates) == 0:
            return True, 'served_approaches_empty'
        if oth_q + oth_f > cur_q + cur_f + self.k['switch_threshold_veh']:
            return True, 'opposing_pressure_larger'
        return False, 'hold_green'
