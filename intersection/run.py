"""Run paired intersection experiments and export CSV, a summary, a report, and the viewer."""
import csv
import json
from pathlib import Path
import statistics

from .model import make_arrivals, simulate, validate

ROOT = Path(__file__).resolve().parent
MODES = ('fixed', 'adaptive', 'adaptive_no_hysteresis')
RUN_TABLES = ('observations', 'estimates', 'forecasts', 'control_log', 'events', 'vehicles')

REPORT_NOTES = [
    'The no-hysteresis ablation sets only the switching queue difference threshold to zero.',
    'These are synthetic, uncalibrated point-queue experiments, not evidence of crash reduction.',
    'Delay includes accrued waits of unfinished vehicles and external queues. No clearance tail is used.',
    'Forecast MAE compares identical issue times/horizons within each run; forecasts are advisory.',
    'Three seeds provide an exploratory sensitivity check, not a statistical significance claim.',
]


def write_rows(path, rows):
    if not rows:
        path.write_text('', encoding='utf-8')
        return
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def save_run(folder, run):
    folder.mkdir(parents=True, exist_ok=True)
    for key in RUN_TABLES:
        write_rows(folder / f'{key}.csv', run[key])
    (folder / 'metrics.json').write_text(json.dumps(run['metrics'], indent=2), encoding='utf-8')


def summarize(scenario, metrics, seeds):
    """Mean metrics per controller; savings are paired against fixed on the same seed."""
    rows = []
    for mode in MODES:
        runs = [metrics[(s, mode)] for s in seeds]
        savings = [metrics[(s, 'fixed')]['total_delay_vehicle_s'] - metrics[(s, mode)]['total_delay_vehicle_s']
                   for s in seeds]
        rows.append({'scenario': scenario, 'controller': mode, 'seeds': len(seeds),
            'mean_total_delay_vehicle_s': statistics.mean(m['total_delay_vehicle_s'] for m in runs),
            'mean_exited_vehicles': statistics.mean(m['exited_vehicles'] for m in runs),
            'mean_residual_queue_veh': statistics.mean(m['residual_queue_veh'] for m in runs),
            'mean_residual_external_veh': statistics.mean(m['residual_external_veh'] for m in runs),
            'mean_paired_delay_saving_vehicle_s': statistics.mean(savings),
            'paired_saving_std_vehicle_s': statistics.stdev(savings) if len(savings) > 1 else None})
    return rows


def run_experiments(config, output, selected=None, seeds=None):
    """Every scenario x seed x controller, all controllers sharing one arrival schedule."""
    validate(config)
    output.mkdir(parents=True, exist_ok=True)
    seeds = config['seeds'] if seeds is None else seeds
    selected = list(config['scenarios']) if selected is None else selected
    if not seeds or len(set(seeds)) != len(seeds):
        raise ValueError('provide distinct seeds')
    (output / 'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')

    viewer_runs, summaries = {}, []
    for scenario in selected:
        metrics = {}
        for seed in seeds:
            arrivals = make_arrivals(config, config['scenarios'][scenario], seed)
            for mode in MODES:
                run = simulate(config, scenario, mode, seed, arrivals)
                metrics[(seed, mode)] = run['metrics']
                save_run(output / scenario / str(seed) / mode, run)
                if seed == seeds[0]:   # the viewer replays the first seed only
                    viewer_runs[f'{scenario}/{mode}'] = {k: run[k] for k in ('metrics', 'frames', 'control_log')}
        rows = summarize(scenario, metrics, seeds)
        summaries.extend(rows)
        fixed, adaptive = rows[0], rows[1]
        print(f"{scenario}: fixed delay {fixed['mean_total_delay_vehicle_s']:.1f}, "
              f"adaptive {adaptive['mean_total_delay_vehicle_s']:.1f} veh.s")

    write_rows(output / 'summary.csv', summaries)
    payload = {'config': config, 'seed': seeds[0], 'runs': viewer_runs, 'summary': summaries,
               'scenarios': {k: config['scenarios'][k] for k in selected}}
    write_viewer(output, payload)
    write_report(output / 'experiment_report.md', summaries, seeds)
    return payload


def write_viewer(output, payload):
    (output / 'data.json').write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')
    data = json.dumps(payload, ensure_ascii=False).replace('<', '\\u003c')   # safe inside <script>
    template = (ROOT / 'viewer_template.html').read_text(encoding='utf-8')
    (output / 'viewer.html').write_text(template.replace('__SIGNAL_DATA__', data), encoding='utf-8')


def write_report(path, summaries, seeds):
    lines = ['# Signal experiments', '',
             f'Seeds: {seeds}. Paired identical arrival schedules; positive saving means lower delay than fixed.',
             '',
             '| Scenario | Controller | Mean delay (veh.s) | Mean exits | Mean external residual | Paired saving (veh.s) |',
             '|---|---|---:|---:|---:|---:|']
    for r in summaries:
        lines.append(f"| {r['scenario']} | {r['controller']} | {r['mean_total_delay_vehicle_s']:.1f} "
                     f"| {r['mean_exited_vehicles']:.1f} | {r['mean_residual_external_veh']:.1f} "
                     f"| {r['mean_paired_delay_saving_vehicle_s']:.1f} |")
    lines += [''] + REPORT_NOTES
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
