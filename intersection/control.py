"""Signal controllers. They see only estimates from the observation pipeline, never the plant."""
from dataclasses import dataclass

GROUPS = {'NS': ('N', 'S'), 'EW': ('E', 'W')}   # two non-conflicting phase groups
EPS = 1e-9


@dataclass
class SignalState:
    group: str = 'NS'
    stage: str = 'green'        # green -> yellow -> all_red -> next group's green
    started: float = 0.0

    def permits(self, approach):
        return self.stage == 'green' and approach in GROUPS[self.group]


def group_queue(group, estimates):
    return sum(estimates[a]['estimated_queue_veh'] for a in GROUPS[group])


class SignalController:
    """Bounded two-phase state machine shared by all policies.

    fixed                   - fixed green per group (fixed_green_s)
    adaptive                - queue-responsive within [min_green_s, max_green_s];
                              falls back to the fixed plan while any detector is not 'ok'
    adaptive_no_hysteresis  - adaptive with switch_threshold_veh = 0 (ablation)

    No discharge during yellow/all-red. Fallback finishes the current clearance;
    it never jumps straight to a conflicting green.
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
        """Advance the signal to time t and return a command record."""
        s = self.state
        elapsed = t - s.started
        fallback = self.mode == 'adaptive' and any(e['quality_flag'] != 'ok' for e in estimates.values())
        if s.stage == 'green':
            switch, reason = self._green_decision(elapsed, estimates, fallback)
            if switch:
                s.stage, s.started = 'yellow', t
            action = 'transition' if switch else 'hold'
        else:
            reason, action = self._clearance_step(elapsed, t)
        return {'issue_time_s': t, 'apply_time_s': t, 'actuator_id': 'intersection',
                'action': action, 'value': f'{s.group}_{s.stage}', 'units': 'phase_state',
                'reason': reason, 'fallback': fallback, 'phase_elapsed_s': t - s.started,
                'ns_estimated_queue_veh': group_queue('NS', estimates),
                'ew_estimated_queue_veh': group_queue('EW', estimates)}

    def _clearance_step(self, elapsed, t):
        """Yellow -> all-red -> green of the other group, with fixed durations."""
        s, c = self.state, self.c
        if s.stage == 'yellow' and elapsed >= c['yellow_s'] - EPS:
            s.stage, s.started = 'all_red', t
            return 'yellow_complete', 'transition'
        if s.stage == 'all_red' and elapsed >= c['all_red_s'] - EPS:
            s.group = 'EW' if s.group == 'NS' else 'NS'
            s.stage, s.started = 'green', t
            return 'clearance_complete', 'transition'
        return 'clearance_in_progress', 'hold'

    def _green_decision(self, elapsed, estimates, fallback):
        """Return (switch_now, reason) for the current green."""
        c, group = self.c, self.state.group
        if elapsed < c['min_green_s'] - EPS:
            return False, 'minimum_green'
        if self.mode == 'fixed' or fallback:
            switch = elapsed >= c['fixed_green_s'][group] - EPS
            return switch, 'fallback_fixed' if fallback else 'fixed_plan'

        other = 'EW' if group == 'NS' else 'NS'
        current_q, other_q = group_queue(group, estimates), group_queue(other, estimates)
        if elapsed >= c['max_green_s'] - EPS:
            return True, 'maximum_green'
        if other_q > current_q + c['switch_threshold_veh']:     # threshold = hysteresis
            return True, 'opposing_queue_larger'
        if current_q == 0 and other_q > 0:
            return True, 'current_queue_empty'
        return False, 'hold_green'
