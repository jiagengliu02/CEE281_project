"""Run paired isolated-intersection experiments and export an offline viewer (stdlib only)."""
import csv
import json
from pathlib import Path
import statistics

from .model import simulate, make_arrivals, validate

ROOT = Path(__file__).resolve().parent
MODES = ('fixed', 'adaptive', 'adaptive_no_hysteresis')


def write_rows(path, rows):
    if not rows:
        path.write_text('', encoding='utf-8')
        return
    with path.open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_experiments(config, output, selected=None, seeds=None):
    validate(config)
    output.mkdir(parents=True, exist_ok=True)
    seeds = config['seeds'] if seeds is None else seeds
    selected = list(config['scenarios']) if selected is None else selected
    if not seeds or len(set(seeds)) != len(seeds):
        raise ValueError('provide distinct seeds')
    results, viewer_runs, summaries = {}, {}, []
    (output / 'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding='utf-8')
    for scenario in selected:
        results[scenario] = {}
        for seed in seeds:
            arrivals = make_arrivals(config, config['scenarios'][scenario], seed)
            for mode in MODES:
                run = simulate(config, scenario, mode, seed, arrivals)
                results[scenario][(seed, mode)] = run['metrics']
                folder = output / scenario / str(seed) / mode
                folder.mkdir(parents=True, exist_ok=True)
                for key in ('observations', 'estimates', 'forecasts', 'control_log', 'events', 'vehicles'):
                    write_rows(folder / f'{key}.csv', run[key])
                (folder / 'metrics.json').write_text(json.dumps(run['metrics'], indent=2), encoding='utf-8')
                if seed == seeds[0]:
                    viewer_runs[f'{scenario}/{mode}'] = {k: run[k] for k in ('metrics', 'frames', 'control_log')}
        for mode in MODES:
            metrics = [results[scenario][(s, mode)] for s in seeds]
            differences = [results[scenario][(s, 'fixed')]['total_delay_vehicle_s'] - results[scenario][(s, mode)]['total_delay_vehicle_s'] for s in seeds]
            summaries.append({'scenario': scenario, 'controller': mode, 'seeds': len(seeds),
                'mean_total_delay_vehicle_s': statistics.mean(m['total_delay_vehicle_s'] for m in metrics),
                'mean_exited_vehicles': statistics.mean(m['exited_vehicles'] for m in metrics),
                'mean_residual_queue_veh': statistics.mean(m['residual_queue_veh'] for m in metrics),
                'mean_residual_external_veh': statistics.mean(m['residual_external_veh'] for m in metrics),
                'mean_paired_delay_saving_vehicle_s': statistics.mean(differences),
                'paired_saving_std_vehicle_s': statistics.stdev(differences) if len(differences) > 1 else None})
        baseline = next(r for r in summaries if r['scenario'] == scenario and r['controller'] == 'fixed')
        candidate = next(r for r in summaries if r['scenario'] == scenario and r['controller'] == 'adaptive')
        print(f"{scenario}: fixed delay {baseline['mean_total_delay_vehicle_s']:.1f}, adaptive {candidate['mean_total_delay_vehicle_s']:.1f} veh.s")
    write_rows(output / 'summary.csv', summaries)
    payload = {'config': config, 'seed': seeds[0], 'runs': viewer_runs, 'summary': summaries,
               'scenarios': {k: config['scenarios'][k] for k in selected}}
    (output / 'data.json').write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')
    data = json.dumps(payload, ensure_ascii=False).replace('<', '\\u003c')
    (output / 'viewer.html').write_text((ROOT / 'viewer_template.html').read_text(encoding='utf-8').replace('__SIGNAL_DATA__', data), encoding='utf-8')
    lines = ['# Signal experiments', '', f'Seeds: {seeds}. Paired identical arrival schedules; positive saving means lower delay than fixed.',
             '', '| Scenario | Controller | Mean delay (veh.s) | Mean exits | Mean external residual | Paired saving (veh.s) |', '|---|---|---:|---:|---:|---:|']
    for r in summaries:
        lines.append(f"| {r['scenario']} | {r['controller']} | {r['mean_total_delay_vehicle_s']:.1f} | {r['mean_exited_vehicles']:.1f} | {r['mean_residual_external_veh']:.1f} | {r['mean_paired_delay_saving_vehicle_s']:.1f} |")
    lines += ['', 'The no-hysteresis ablation sets only the switching queue difference threshold to zero.',
              'These are synthetic, uncalibrated point-queue experiments, not evidence of crash reduction.',
              'Delay includes accrued waits of unfinished vehicles and external queues. No clearance tail is used.',
              'Forecast MAE compares identical issue times/horizons within each run; forecasts are advisory.',
              'Three seeds provide an exploratory sensitivity check, not a statistical significance claim.']
    (output / 'experiment_report.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return payload
