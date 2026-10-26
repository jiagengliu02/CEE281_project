"""Run paired freeway cases and export CSV, JSON, and the HTML viewer."""
import csv
import json
from pathlib import Path

from .model import simulate, validate

ROOT = Path(__file__).resolve().parent
MODES = ('open', 'closure', 'fixed', 'adaptive', 'adaptive_fault')

CSV_FIELDS = {
    'traces': ['time_s', 'vehicle_id', 'origin', 'lane_id', 'position_m', 'speed_mps',
               'acceleration_mps2'],
    'observations': ['sensor_id', 'position_m', 'lane_id', 'interval_start_s', 'interval_end_s',
                     'count', 'mean_speed_mps', 'quality_flag'],
    'yield_log': ['time_s', 'yielding_vehicle_id', 'merging_vehicle_id', 'position_m',
                  'requested_acceleration_mps2', 'applied_acceleration_mps2'],
    'lane_changes': ['time_s', 'vehicle_id', 'position_m', 'from_lane', 'to_lane', 'reason',
                     'self_acceleration_after_mps2', 'target_follower_acceleration_after_mps2'],
}
# Large logs stay in CSV only; the viewer gets frames, observations, and metrics.
NOT_IN_VIEWER = ('traces', 'control_log', 'lane_changes', 'yield_log', 'estimates')


def write_csv(path, rows, fields):
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def run_experiments(config, output):
    """Run paired modes with identical demand and export the results."""
    validate(config)
    output.mkdir(parents=True, exist_ok=True)
    runs = {}
    for mode in MODES:
        run = simulate(config, mode)
        folder = output / mode
        folder.mkdir(exist_ok=True)
        for key, fields in CSV_FIELDS.items():
            write_csv(folder / f'{key}.csv', run[key], fields)
        for key in ('control_log', 'boundary_observations', 'estimates', 'forecasts', 'drivers'):
            rows = run[key]
            if rows:
                write_csv(folder / f'{key}.csv', rows, list(rows[0]))
            else:
                (folder / f'{key}.csv').write_text('')
        (folder / 'metrics.json').write_text(json.dumps(run['metrics'], indent=2))

        runs[mode] = {k: v for k, v in run.items() if k not in NOT_IN_VIEWER}
        runs[mode]['merge_events'] = [r for r in run['lane_changes'] if r['reason'] == 'closure_merge']
        m = run['metrics']
        print(f"{mode}: {m['exited_vehicles']} exits, delay {m['total_delay_vehicle_s']:.2f} veh.s")

    (output / 'config.json').write_text(json.dumps(config, indent=2))
    payload = {'config': config, 'runs': runs}
    data = json.dumps(payload, separators=(',', ':'))
    (output / 'data.json').write_text(data)
    template = (ROOT / 'viewer_template.html.in').read_text()
    (output / 'viewer.html').write_text(template.replace('__SIMULATION_DATA__', data))
    write_report(output, payload)
    return payload


def write_report(output, payload):
    """Keep the measured comparison, including negative results, beside each run."""
    fields = ['mode', 'generated_vehicles', 'exited_vehicles', 'total_delay_vehicle_s',
              'remaining_on_road', 'remaining_main_queue', 'remaining_ramp_queue',
              'road_estimate_mae_veh', 'ramp_estimate_mae_veh', 'fallback_time_s']
    metrics = [payload['runs'][mode]['metrics'] for mode in MODES]
    write_csv(output / 'summary.csv', [{k:m[k] for k in fields} for m in metrics], fields)
    lines = ['# Five-stage ramp-control experiment', '',
             'Same deterministic demand and per-vehicle driver parameters in all modes. '
             'Only open has no lane closure; compare closure/fixed/adaptive for control effects.', '',
             '| Mode | Departures | Total delay (veh.min) | Residual main / ramp / road (veh) | Fallback (s) |',
             '|---|---:|---:|---:|---:|']
    for m in metrics:
        lines.append(f"| {m['mode']} | {m['exited_vehicles']} | {m['total_delay_vehicle_s']/60:.2f} | "
                     f"{m['remaining_main_queue']} / {m['remaining_ramp_queue']} / {m['remaining_on_road']} | {m['fallback_time_s']} |")
    fixed = payload['runs']['fixed']['metrics']['total_delay_vehicle_s']
    adaptive = payload['runs']['adaptive']['metrics']['total_delay_vehicle_s']
    lines += ['', f'Adaptive delay minus fixed delay: {(adaptive-fixed)/60:.2f} veh.min (positive means worse).',
              '', 'Forecasts actively change adaptive release intervals. Conservation estimates use measured cumulative '
              'boundary counters; the controller never sees true queues or future demand. '
              'Forecasts are evaluated on matched issue times and complete 30-second horizons, after the declared evaluation start.',
              '', 'The initial adaptive rule is not guaranteed to outperform fixed control. Inventory extrapolation, '
              'queue-relief overrides and merge timing can produce adverse feedback. '
              'A fault run performing better is not evidence that missing data are beneficial: fallback changes the timing policy.',
              '', 'Ramp vehicles use IDM along a finite ramp with unlimited upstream FIFO storage. '
              'The queue threshold is soft, all residual queues and their accrued waiting are included. '
              'No physical ramp-storage or spatial spillback claim is made.',
              '', 'Parameters were declared for a teaching prototype. This is neither empirical validation nor '
              'a safety-effectiveness claim. Preserve these results when developing and testing another controller.']
    (output / 'experiment_report.md').write_text('\n'.join(lines)+'\n')
