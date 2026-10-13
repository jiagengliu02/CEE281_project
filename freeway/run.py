"""Run the three paired freeway cases and export CSV, JSON, and the HTML viewer."""
import csv
import json
from pathlib import Path

from .model import simulate, validate

ROOT = Path(__file__).resolve().parent
MODES = ('open', 'closure', 'fixed')

CSV_FIELDS = {
    'traces': ['time_s', 'vehicle_id', 'origin', 'lane_id', 'position_m', 'speed_mps',
               'acceleration_mps2'],
    'observations': ['sensor_id', 'position_m', 'lane_id', 'interval_start_s', 'interval_end_s',
                     'count', 'mean_speed_mps', 'quality_flag'],
    'control_log': ['issue_time_s', 'actuator_id', 'action', 'waiting_vehicles', 'reason', 'scenario'],
    'yield_log': ['time_s', 'yielding_vehicle_id', 'merging_vehicle_id', 'position_m',
                  'requested_acceleration_mps2', 'applied_acceleration_mps2'],
    'lane_changes': ['time_s', 'vehicle_id', 'position_m', 'from_lane', 'to_lane', 'reason',
                     'self_acceleration_after_mps2', 'target_follower_acceleration_after_mps2'],
}
# Large logs stay in CSV only; the viewer gets frames, observations, and metrics.
NOT_IN_VIEWER = ('traces', 'control_log', 'lane_changes', 'yield_log')


def write_csv(path, rows, fields):
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run_experiments(config, output):
    """Run open / closure / fixed with identical demand and export the results."""
    validate(config)
    output.mkdir(parents=True, exist_ok=True)
    runs = {}
    for mode in MODES:
        run = simulate(config, mode)
        folder = output / mode
        folder.mkdir(exist_ok=True)
        for key, fields in CSV_FIELDS.items():
            write_csv(folder / f'{key}.csv', run[key], fields)
        (folder / 'metrics.json').write_text(json.dumps(run['metrics'], indent=2))

        runs[mode] = {k: v for k, v in run.items() if k not in NOT_IN_VIEWER}
        runs[mode]['merge_events'] = [r for r in run['lane_changes'] if r['reason'] == 'closure_merge']
        m = run['metrics']
        print(f"{mode}: {m['exited_vehicles']} exits, delay {m['total_delay_vehicle_s']:.2f} veh.s")

    payload = {'config': config, 'runs': runs}
    data = json.dumps(payload, separators=(',', ':'))
    (output / 'data.json').write_text(data)
    template = (ROOT / 'viewer_template.html').read_text()
    (output / 'viewer.html').write_text(template.replace('__SIMULATION_DATA__', data))
    return payload
