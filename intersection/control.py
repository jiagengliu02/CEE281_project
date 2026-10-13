"""Controllers receive observation estimates only; this module never imports the plant."""
from dataclasses import dataclass

GROUPS = {'NS': ('N', 'S'), 'EW': ('E', 'W')}


@dataclass
class SignalState:
    group: str = 'NS'
    stage: str = 'green'
    started: float = 0.0

    def permits(self, approach):
        return self.stage == 'green' and approach in GROUPS[self.group]


class SignalController:
    """Bounded two-phase state machine shared by both decision policies.

    Yellow/all-red have no stop-line service in this conservative point-queue model.
    Fallback finishes the existing clearance; it never jumps to a conflicting green.
    """
    def __init__(self, config, mode):
        if mode not in ('fixed', 'adaptive', 'adaptive_no_hysteresis'):
            raise ValueError('unknown controller')
        self.c = dict(config)
        self.mode = 'fixed' if mode == 'fixed' else 'adaptive'
        if mode == 'adaptive_no_hysteresis':
            self.c['switch_threshold_veh'] = 0
        self.state = SignalState()

    def update(self, t, estimates):
        s, c = self.state, self.c
        elapsed = t - s.started
        other = 'EW' if s.group == 'NS' else 'NS'
        fallback = self.mode == 'adaptive' and any(
            estimates[a]['quality_flag'] != 'ok' for a in 'NSEW')
        reason, action = 'clearance_in_progress', 'hold'
        if s.stage == 'yellow' and elapsed >= c['yellow_s'] - 1e-9:
            s.stage, s.started = 'all_red', t
            reason, action = 'yellow_complete', 'transition'
        elif s.stage == 'all_red' and elapsed >= c['all_red_s'] - 1e-9:
            s.group, s.stage, s.started = other, 'green', t
            reason, action = 'clearance_complete', 'transition'
        elif s.stage == 'green':
            reason = 'minimum_green' if elapsed < c['min_green_s'] - 1e-9 else 'hold_green'
            switch = False
            if elapsed >= c['min_green_s'] - 1e-9:
                if self.mode == 'fixed' or fallback:
                    switch = elapsed >= c['fixed_green_s'][s.group] - 1e-9
                    reason = ('fallback_fixed' if fallback else 'fixed_plan')
                else:
                    current_q = sum(estimates[a]['estimated_queue_veh'] for a in GROUPS[s.group])
                    other_q = sum(estimates[a]['estimated_queue_veh'] for a in GROUPS[other])
                    if elapsed >= c['max_green_s'] - 1e-9:
                        switch, reason = True, 'maximum_green'
                    elif other_q > current_q + c['switch_threshold_veh']:
                        switch, reason = True, 'opposing_queue_larger'
                    elif current_q == 0 and other_q > 0:
                        switch, reason = True, 'current_queue_empty'
            if switch:
                s.stage, s.started = 'yellow', t
                action = 'transition'
        return {'issue_time_s': t, 'apply_time_s': t, 'actuator_id': 'intersection',
                'action': action, 'value': f'{s.group}_{s.stage}', 'units': 'phase_state',
                'reason': reason, 'fallback': fallback, 'phase_elapsed_s': t - s.started,
                'ns_estimated_queue_veh': sum(estimates[a]['estimated_queue_veh'] for a in GROUPS['NS']),
                'ew_estimated_queue_veh': sum(estimates[a]['estimated_queue_veh'] for a in GROUPS['EW'])}
