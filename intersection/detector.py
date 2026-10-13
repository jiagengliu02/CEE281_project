"""Entry and stop-line counters per approach, with optional injected faults.

Each report covers [t - interval, t). Faults:
    missing - counts are withheld (None) and flagged 'missing'
    delay   - the report only becomes available delay_s after the interval ends
"""
import heapq

APPROACHES = ('N', 'S', 'E', 'W')
COUNT_FIELDS = ('count', 'departure_count', 'cumulative_entered', 'cumulative_departed')


class StopLineDetectors:
    def __init__(self, config, faults=()):
        self.interval = config['detector_interval_s']
        self.faults = list(faults)
        self.last_entered = {a: 0 for a in APPROACHES}
        self.last_departed = {a: 0 for a in APPROACHES}
        self.observations = []      # every report as produced (ground-truth log)
        self._pending = []          # heap of (available_time_s, sequence, report)
        self._sequence = 0

    def report(self, t, entered, departed):
        """Close the current interval for every approach using cumulative counters."""
        for a in APPROACHES:
            row = {'sensor_id': f'entry_exit_{a}', 'approach': a,
                   'interval_start_s': t - self.interval, 'interval_end_s': t,
                   'count': entered[a] - self.last_entered[a],
                   'departure_count': departed[a] - self.last_departed[a],
                   'cumulative_entered': entered[a], 'cumulative_departed': departed[a],
                   'mean_speed_mps': None, 'quality_flag': 'ok', 'available_time_s': t}
            self.last_entered[a], self.last_departed[a] = entered[a], departed[a]
            self._apply_faults(row, t)
            self.observations.append(row)
            heapq.heappush(self._pending, (row['available_time_s'], self._sequence, row))
            self._sequence += 1

    def _apply_faults(self, row, t):
        for fault in self.faults:
            if fault['approach'] != row['approach'] or not fault['start_s'] <= t < fault['end_s']:
                continue
            if fault['kind'] == 'missing':
                row['quality_flag'] = 'missing'
                for key in COUNT_FIELDS:
                    row[key] = None
            else:
                row['available_time_s'] = t + fault['delay_s']
                row['quality_flag'] = 'delayed'

    def deliver(self, t):
        """Reports that have reached the controller by time t, oldest first."""
        delivered = []
        while self._pending and self._pending[0][0] <= t + 1e-9:
            delivered.append(heapq.heappop(self._pending)[2])
        return delivered
