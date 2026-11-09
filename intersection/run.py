"""Run every scenario with every controller on identical arrivals; export CSV, JSON, report and viewer."""
import csv
import json
from pathlib import Path

from .control import MODES, webster_plan
from .model import make_arrivals, simulate, validate
from .observation import APPROACHES

ROOT = Path(__file__).resolve().parent
RUN_TABLES = ('traces', 'crossings', 'observations', 'estimates', 'forecasts',
              'control_log', 'phases', 'arrivals', 'drivers')
# Large logs stay in CSV only; the viewer receives frames, phases, metrics and compact
# versions of the control log and the scored forecasts.
VIEWER_COMMAND_FIELDS = ('issue_time_s', 'value', 'reason', 'fallback', 'phase_elapsed_s', 'fixed_green_s',
                         'current_queue_est_veh', 'opposing_queue_est_veh', 'current_forecast_veh',
                         'opposing_forecast_veh', 'quality_flags')


def viewer_payload(run):
    forecasts = [{'issue_time_s': r['issue_time_s'], 'segment_id': r['segment_id'], 'method': r['method'],
                  'predicted_value': round(r['predicted_value'], 2), 'actual_value': r['actual_value'],
                  'absolute_error': round(r['absolute_error'], 2), 'evaluation_period': True}
                 for r in run['forecasts'] if r['evaluation_period']]
    return {'metrics': run['metrics'], 'frames': run['frames'], 'phases': run['phases'],
            'control_log': [{k: r[k] for k in VIEWER_COMMAND_FIELDS} for r in run['control_log']],
            'forecasts': forecasts}


def write_rows(path, rows):
    if not rows:
        path.write_text('', encoding='utf-8')
        return
    fields = []
    for row in rows:
        fields.extend(k for k in row if k not in fields)
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def save_run(folder, run):
    folder.mkdir(parents=True, exist_ok=True)
    for key in RUN_TABLES:
        write_rows(folder / f'{key}.csv', run[key])
    (folder / 'metrics.json').write_text(json.dumps(run['metrics'], indent=2), encoding='utf-8')


def summary_row(scenario, m, fixed):
    return {'scenario': scenario, 'controller': m['mode'], 'demand_id': m['demand_id'],
            'generated_vehicles': m['generated_vehicles'], 'exited_vehicles': m['exited_vehicles'],
            'throughput_vph': m['throughput_vph'],
            'total_delay_vehicle_s': m['total_delay_vehicle_s'],
            'delay_saving_vs_fixed_vehicle_s': round(fixed['total_delay_vehicle_s'] - m['total_delay_vehicle_s'], 2),
            'remaining_on_road': m['remaining_on_road'],
            'remaining_boundary_queue': m['remaining_boundary_queue'],
            'max_boundary_queue': max(m['max_boundary_queue_by_approach'].values()),
            'max_stopped_queue': max(m['max_stopped_queue_by_approach'].values()),
            'service_imbalance_ratio': m['service_imbalance_ratio'],
            'green_phases': m['green_phases'],
            'fallback_time_s': m['fallback_time_s'],
            'queue_estimate_mae_mean_veh': m['queue_estimate_mae_mean_veh'],
            'forecast_mae_moving_average_veh': m['forecast']['moving_average']['mae_veh'],
            'forecast_mae_ema_veh': m['forecast']['ema']['mae_veh'],
            'red_light_runs': m['red_light_runs'], 'all_red_crossings': m['all_red_crossings'],
            'conflict_box_steps_with_both_groups': m['conflict_box_steps_with_both_groups'],
            'strongest_braking_mps2': m['strongest_braking_mps2'],
            'minimum_gap_m': m['minimum_gap_m'], 'safety_interventions': m['safety_interventions']}


def run_experiments(config, output, selected=None):
    """Every selected scenario x controller; all controllers of a scenario share one arrival table."""
    validate(config)
    output.mkdir(parents=True, exist_ok=True)
    selected = list(config['scenarios']) if selected is None else list(selected)
    (output / 'config.json').write_text(json.dumps(config, indent=2), encoding='utf-8')
    viewer_runs, summaries = {}, []
    for scenario in selected:
        arrivals = make_arrivals(config, config['scenarios'][scenario])
        runs = {}
        for mode in MODES:
            run = simulate(config, scenario, mode, arrivals)
            runs[mode] = run
            save_run(output / scenario / mode, run)
            viewer_runs[f'{scenario}/{mode}'] = viewer_payload(run)
            m = run['metrics']
            print(f"{scenario}/{mode}: {m['exited_vehicles']} exits, delay {m['total_delay_vehicle_s']:.1f} veh.s, "
                  f"{m['green_phases']} greens, fallback {m['fallback_time_s']} s")
        fixed = runs['fixed']['metrics']
        summaries.extend(summary_row(scenario, runs[mode]['metrics'], fixed) for mode in MODES)
    write_rows(output / 'summary.csv', summaries)
    payload = {'config': config, 'plan': webster_plan(config), 'modes': MODES, 'approaches': APPROACHES,
               'scenarios': {k: config['scenarios'][k] for k in selected},
               'runs': viewer_runs, 'summary': summaries}
    (output / 'data.json').write_text(json.dumps(payload, separators=(',', ':')), encoding='utf-8')
    data = json.dumps(payload, separators=(',', ':')).replace('<', '\\u003c')
    template = (ROOT / 'viewer_template.html.in').read_text(encoding='utf-8')
    (output / 'viewer.html').write_text(template.replace('__SIGNAL_DATA__', data), encoding='utf-8')
    write_report(output / 'experiment_report.md', payload)
    return payload


def write_report(path, payload):
    """Measured comparison, including negative results, beside the runs."""
    plan = payload['plan']
    lines = ['# Signal-control experiments', '',
             f"Fixed-time baseline: Webster plan on the declared design demand, displayed green "
             f"NS {plan['green_s']['NS']} s / EW {plan['green_s']['EW']} s, cycle {plan['cycle_s']} s "
             f"(saturation flow {payload['config']['saturation_flow_vph']} veh/h measured from the IDM discharge check). "
             'The plan is never retuned to a test scenario.', '',
             'All controllers of a scenario use the same arrival table and driver parameters. '
             'Delay includes boundary waits and unfinished trips (veh.s). Positive saving = less delay than fixed.', '']
    for scenario in payload['scenarios']:
        rows = [r for r in payload['summary'] if r['scenario'] == scenario]
        lines += [f"## {payload['scenarios'][scenario]['label']} (`{scenario}`, demand {rows[0]['demand_id']})", '',
                  '| Controller | Exits | Delay (veh.min) | Saving vs fixed (veh.min) | Residual road / boundary | '
                  'Max boundary | Greens | Fallback (s) | Estimate MAE (veh) | Red runs | Box conflicts |',
                  '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
        for r in rows:
            lines.append(f"| {r['controller']} | {r['exited_vehicles']} | {r['total_delay_vehicle_s']/60:.2f} | "
                         f"{r['delay_saving_vs_fixed_vehicle_s']/60:.2f} | {r['remaining_on_road']} / {r['remaining_boundary_queue']} | "
                         f"{r['max_boundary_queue']} | {r['green_phases']} | {r['fallback_time_s']} | "
                         f"{r['queue_estimate_mae_mean_veh']} | {r['red_light_runs']} | {r['conflict_box_steps_with_both_groups']} |")
        lines.append('')
    lines += [
        'Notes',
        '',
        '- `adaptive_no_forecast` is the CEE281 ablation: the identical pressure rule with every forecast term set to zero.',
        '- The queue estimate counts vehicles between the advance loop and the stop line; queues longer than the detector zone are invisible to the controller, which matters in the heavy case.',
        '- Fallback time counts seconds in which a responsive controller ran the fixed plan because a report was unavailable, missing or stale (startup included).',
        '- Red runs must be zero; all-red crossings are committed vehicles still clearing the line during all-red. Neither is a crash measure.',
        '- Synthetic, uncalibrated teaching model; one arrival seed. Results describe this model, not a real intersection.',
    ]
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
