"""Freeway experiment runner and CSV/HTML export."""
import csv
import json
from pathlib import Path

from .model import simulate, validate

ROOT = Path(__file__).resolve().parent


def write_csv(path, rows, fields):
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run_experiments(c, output):
    """Run the three freeway cases and export their results."""
    validate(c)
    output.mkdir(parents=True, exist_ok=True)
    runs = {}
    for mode in ['open', 'closure', 'fixed']:
        run = simulate(c, mode)
        folder = output / mode
        folder.mkdir(exist_ok=True)
        fields = {'traces': ['time_s','vehicle_id','origin','lane_id','position_m','speed_mps','acceleration_mps2'],
            'observations': ['sensor_id','position_m','lane_id','interval_start_s','interval_end_s','count','mean_speed_mps','quality_flag'],
            'control_log': ['issue_time_s','actuator_id','action','waiting_vehicles','reason','scenario'],
            'yield_log': ['time_s','yielding_vehicle_id','merging_vehicle_id','position_m','requested_acceleration_mps2','applied_acceleration_mps2'],
            'lane_changes': ['time_s','vehicle_id','position_m','from_lane','to_lane','reason','self_acceleration_after_mps2','target_follower_acceleration_after_mps2']}
        for key, names in fields.items():
            write_csv(folder / (key + '.csv'), run[key], names)
        (folder / 'metrics.json').write_text(json.dumps(run['metrics'], indent=2))
        runs[mode] = {k: v for k, v in run.items() if k not in ['traces', 'control_log', 'lane_changes', 'yield_log']}
        runs[mode]['merge_events'] = [r for r in run['lane_changes'] if r['reason'] == 'closure_merge']
        print(f"{mode}: {run['metrics']['exited_vehicles']} exits, delay {run['metrics']['total_delay_vehicle_s']:.2f} veh.s")
    payload = {'config': c, 'runs': runs}
    (output / 'data.json').write_text(json.dumps(payload, separators=(',', ':')))
    (output / 'viewer.html').write_text((ROOT / 'viewer_template.html').read_text().replace(
        '__SIMULATION_DATA__', json.dumps(payload, separators=(',', ':'))))
    return payload
