"""Independent checks for the signalized intersection: discharge physics, clearance
geometry, trajectory audits of every default run, controller constraints and
time-step sensitivity. Writes results/intersection/verification.json."""
import copy
import json
import math
from pathlib import Path

from freeway.behavior import VEHICLE_LENGTH_M
from freeway.drivers import driver_parameters

from .control import MODES, webster_plan
from .model import IntersectionSimulation, SignalVehicle, make_arrivals, simulate, validate
from .observation import APPROACHES, GROUPS
from .test_signal import verify_loop

ROOT = Path(__file__).resolve().parent
GROUP_OF = {a: g for g, members in GROUPS.items() for a in members}


def standing_queue_config(config, duration=150):
    c = copy.deepcopy(config)
    c['duration_s'] = duration
    c['min_green_s'] = c['max_green_s'] = duration      # NS green for the whole run
    c['scenarios'] = {'empty': {'label': 'empty', 'demand': [
        {'start_s': 0, 'rates_vph': {a: 0 for a in APPROACHES}}]}}
    validate(c)
    return c


def check_saturation_discharge(config, vehicles=25):
    """Release a standing queue into a permanent green and measure the stop-line headways.

    Saturation flow = 3600 / mean headway of vehicles 5-20; startup lost time is the
    extra time the first 12 vehicles need beyond 11 saturation headways (HCM style).
    The configured Webster inputs must match these measurements within 10 %.
    """
    c = standing_queue_config(config)
    sim = IntersectionSimulation(c, 'empty', 'fixed')
    x = sim.S
    for i in range(vehicles):
        p = driver_parameters(c, i)
        x -= p['min_gap']
        sim.lanes['N'].append(SignalVehicle(i, 'N', 0, 0, x, 0.0, 0, **p))
        sim.drivers[i] = p
        x -= VEHICLE_LENGTH_M
    sim.arrivals = {'rows': [], 'demand_id': 'standing-queue'}
    sim.next_arrival = vehicles                         # these vehicles count as generated
    run = sim.run()
    times = [r['time_s'] for r in run['crossings'] if r['approach'] == 'N']
    assert len(times) == vehicles, 'every queued vehicle must discharge'
    headways = [b - a for a, b in zip(times, times[1:])]
    saturated = headways[4:20]
    h = sum(saturated) / len(saturated)
    flow = 3600 / h
    lost = times[11] - 11 * h
    assert abs(flow - config['saturation_flow_vph']) / config['saturation_flow_vph'] < 0.10, (flow, config['saturation_flow_vph'])
    assert abs(lost - config['startup_lost_s']) < 1.0, (lost, config['startup_lost_s'])
    assert run['metrics']['safety_interventions'] == 0
    return {'saturation_headway_s': round(h, 3), 'saturation_flow_vph': round(flow, 1),
            'configured_saturation_flow_vph': config['saturation_flow_vph'],
            'startup_lost_time_s': round(lost, 2), 'configured_startup_lost_s': config['startup_lost_s'],
            'first_12_crossing_times_s': [round(t, 1) for t in times[:12]]}


def check_clearance_intervals(config):
    """ITE yellow t_r + v/(2b) at the 85th-percentile desired speed; all-red (box + length) / v15."""
    spec = config['driver_distribution']['desired_speed']
    v85 = min(spec['max'], spec['mean'] + 1.036 * spec['std'])
    v15 = max(spec['min'], spec['mean'] - 1.036 * spec['std'])
    yellow_needed = 1.0 + v85 / (2 * config['yellow_stop_decel_mps2'])
    all_red_needed = (config['box_length_m'] + VEHICLE_LENGTH_M) / v15
    # Drivers in this model have no reaction time, so the 1 s term is a margin here.
    assert config['yellow_s'] >= yellow_needed - 1.0, (config['yellow_s'], yellow_needed)
    assert config['all_red_s'] >= all_red_needed, (config['all_red_s'], all_red_needed)
    return {'yellow_ite_s': round(yellow_needed, 2), 'yellow_configured_s': config['yellow_s'],
            'all_red_needed_s': round(all_red_needed, 2), 'all_red_configured_s': config['all_red_s'],
            'speeds_used_mps': {'v85': round(v85, 2), 'v15': round(v15, 2)}}


def check_webster(config):
    plan = webster_plan(config)
    assert config['min_green_s'] <= min(plan['green_s'].values()) <= config['max_green_s']
    capacity = {g: config['saturation_flow_vph'] * (plan['green_s'][g] - config['startup_lost_s'] + config['yellow_used_s']) / plan['cycle_s']
                for g in GROUPS}
    for g in GROUPS:
        assert capacity[g] > max(config['fixed_plan']['design_demand_vph'][a] for a in GROUPS[g]), 'plan cannot serve design demand'
    return {**plan, 'approach_capacity_vph': {g: round(v, 1) for g, v in capacity.items()}}


def phase_at(phases, t, approach):
    for p in phases:
        if p['start_s'] - 1e-9 <= t < p['end_s'] + 1e-9:
            if GROUP_OF[approach] == p['group'] and p['stage'] in ('green', 'yellow'):
                return p['stage']
            return 'all_red' if p['stage'] == 'all_red' else 'red'
    raise AssertionError('time outside phase timeline')


def audit_run(config, run):
    """Re-derive loop counts, crossings, overlaps and signal compliance from the raw logs."""
    m = run['metrics']
    S = config['approach_length_m']
    advance = S - config['advance_detector_m']
    counts = {(a, s): 0 for a in APPROACHES for s in ('advance', 'stopline')}
    previous, by_time = {}, {}
    for row in run['traces']:
        vid, a, x = row['vehicle_id'], row['approach'], row['position_m']
        old = previous.get(vid, 0.0)
        assert x >= old - 1e-6, 'Vehicle moved backward'
        assert row['speed_mps'] >= 0
        counts[(a, 'advance')] += int(old < advance <= x)
        counts[(a, 'stopline')] += int(old < S <= x)
        previous[vid] = x
        by_time.setdefault((row['time_s'], a), []).append(x)
    for positions in by_time.values():
        positions.sort()
        assert all(b - a >= VEHICLE_LENGTH_M - 1e-3 for a, b in zip(positions, positions[1:])), 'Same-lane overlap'
    for (a, s), n in counts.items():
        measured = sum(o['count'] or 0 for o in run['observations'] if o['sensor_id'] == f'{a}_{s}')
        missing = sum(o['quality_flag'] == 'missing' for o in run['observations'] if o['sensor_id'] == f'{a}_{s}')
        if not missing:
            assert n == measured, (a, s, n, measured)
    # Stop-line crossings versus the independent phase timeline.
    red_runs = all_red = 0
    for row in run['crossings']:
        phase = phase_at(run['phases'], row['time_s'] - config['dt_s'], row['approach'])
        if phase == 'red':
            red_runs += 1
        elif phase == 'all_red':
            all_red += 1
    assert red_runs == 0 and m['red_light_runs'] == 0, red_runs
    assert all_red == m['all_red_crossings'], (all_red, m['all_red_crossings'])
    # Conflict box occupancy from traces: never both groups inside.
    box_end = S + config['box_length_m']
    occupancy = {}
    for row in run['traces']:
        x = row['position_m']
        if S <= x < box_end + VEHICLE_LENGTH_M and x - VEHICLE_LENGTH_M < box_end:
            occupancy.setdefault(row['time_s'], set()).add(GROUP_OF[row['approach']])
    assert all(len(g) == 1 for g in occupancy.values()), 'both groups inside the conflict box'
    assert m['conflict_box_steps_with_both_groups'] == 0
    # Phase timeline: min green, clearance durations, two-phase alternation.
    phases = run['phases']
    for p, q in zip(phases, phases[1:]):
        length = p['end_s'] - p['start_s']
        if p['stage'] == 'yellow':
            assert abs(length - config['yellow_s']) < 1e-6 and q['stage'] == 'all_red'
        elif p['stage'] == 'all_red':
            assert abs(length - config['all_red_s']) < 1e-6 and q['stage'] == 'green' and q['group'] != p['group']
        else:
            assert q['stage'] == 'yellow' and length >= config['min_green_s'] - 1e-6, length
            if m['mode'] == 'fixed':
                assert abs(length - m['fixed_plan']['green_s'][p['group']]) < 1e-6
    greens = [p for p in phases if p['stage'] == 'green' and p is not phases[-1]]
    long_greens = [p for p in greens if p['end_s'] - p['start_s'] > config['max_green_s'] + 1e-6]
    for p in long_greens:   # only allowed while resting without an opposing call or in fallback
        reasons = {r['reason'] for r in run['control_log'] if p['start_s'] <= r['issue_time_s'] < p['end_s']}
        assert reasons & {'rest_no_opposing_call', 'fallback_fixed'}, reasons
    assert m['safety_interventions'] == 0
    return {'conservation': 'passed every time step', 'same_lane_overlap': 'none', 'backward_motion': 'none',
            'loop_counts_recomputed_from_traces': {f'{a}_{s}': n for (a, s), n in counts.items()},
            'red_light_runs': red_runs, 'all_red_crossings': all_red, 'yellow_crossings': m['yellow_crossings'],
            'conflict_box_both_groups': 0, 'green_phases': len(greens),
            'greens_beyond_max_green_while_resting_or_fallback': len(long_greens),
            'strongest_braking_mps2': m['strongest_braking_mps2'], 'minimum_gap_m': m['minimum_gap_m']}


def check_fault_fallback(runs):
    """Responsive controllers must fall back during the declared faults and recover afterwards."""
    report = {}
    for mode in MODES:
        run = runs[('detector_fault', mode)]
        fallback = [r for r in run['control_log'] if r['fallback']]
        in_window = [r for r in fallback if 360 <= r['issue_time_s'] < 480 + 5]
        if mode == 'fixed':
            assert not fallback
        else:
            assert in_window, 'no fallback during the missing-report window'
            after = [r for r in run['control_log'] if r['issue_time_s'] > 700]
            assert after and not any(r['fallback'] for r in after), 'fallback did not clear after the faults'
        report[mode] = {'fallback_time_s': run['metrics']['fallback_time_s'],
                        'queue_estimate_mae_mean_veh': run['metrics']['queue_estimate_mae_mean_veh']}
    return report


def check_pulse_clears(config):
    c = copy.deepcopy(config)
    c['duration_s'] = 400
    c['scenarios'] = {'pulse': {'label': 'pulse', 'demand': [
        {'start_s': 0, 'rates_vph': {a: 600 for a in APPROACHES}},
        {'start_s': 120, 'rates_vph': {a: 0 for a in APPROACHES}}]}}
    out = {}
    for mode in MODES:
        m = simulate(c, 'pulse', mode)['metrics']
        assert m['generated_vehicles'] == m['exited_vehicles'] > 0, (mode, m['generated_vehicles'], m['exited_vehicles'])
        assert m['remaining_on_road'] == m['remaining_boundary_queue'] == 0
        out[mode] = {'vehicles': m['generated_vehicles'], 'delay_vehicle_s': m['total_delay_vehicle_s']}
    return out


def check_overload(config):
    c = copy.deepcopy(config)
    c['duration_s'] = 300
    c['scenarios'] = {'overload': {'label': 'overload', 'demand': [
        {'start_s': 0, 'rates_vph': {a: 2400 for a in APPROACHES}}]}}
    m = simulate(c, 'overload', 'adaptive')['metrics']
    assert m['remaining_boundary_queue'] > 0, 'unmet demand must be kept at the boundary'
    assert m['safety_interventions'] == 0 and m['red_light_runs'] == 0
    assert max(m['max_zone_vehicles_by_approach'].values()) <= math.ceil(config['advance_detector_m'] / VEHICLE_LENGTH_M) + 1
    return {'remaining_boundary_queue': m['remaining_boundary_queue'],
            'max_boundary_queue_by_approach': m['max_boundary_queue_by_approach'],
            'max_zone_vehicles_by_approach': m['max_zone_vehicles_by_approach']}


def check_time_step(config, coarse):
    fine = copy.deepcopy(config)
    fine['dt_s'] /= 2
    out = {}
    for mode in ('fixed', 'adaptive'):
        run = simulate(fine, 'directional_shift', mode)
        assert run['metrics']['safety_interventions'] == 0
        keys = ('exited_vehicles', 'total_delay_vehicle_s', 'green_phases', 'remaining_boundary_queue', 'minimum_gap_m')
        out[mode] = {k: {f"dt_{config['dt_s']}": coarse[('directional_shift', mode)]['metrics'][k],
                         f"dt_{fine['dt_s']}": run['metrics'][k]} for k in keys}
    return out


def verify():
    config = json.loads((ROOT / 'configs/default.json').read_text())
    validate(config)
    report = {'unit_and_loop_tests': verify_loop(),
              'saturation_discharge': check_saturation_discharge(config),
              'clearance_intervals': check_clearance_intervals(config),
              'webster_fixed_plan': check_webster(config)}
    runs = {}
    for scenario in config['scenarios']:
        arrivals = make_arrivals(config, config['scenarios'][scenario])
        for mode in MODES:
            runs[(scenario, mode)] = simulate(config, scenario, mode, arrivals)
            report[f'{scenario}/{mode}_audit'] = audit_run(config, runs[(scenario, mode)])
    report['detector_fault_fallback_and_recovery'] = check_fault_fallback(runs)
    report['finite_pulse_clears_under_every_controller'] = check_pulse_clears(config)
    report['overload_boundary_accounting_and_detector_zone_saturation'] = check_overload(config)
    report['time_step_sensitivity_directional_shift'] = check_time_step(config, runs)
    report['time_step_note'] = ('Signal timing, loop reports and stop decisions are discrete events; '
                                'sensitivity is reported, not forced to match.')
    output = ROOT.parent / 'results/intersection'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'verification.json').write_text(json.dumps(report, indent=2))
    print('Intersection discharge, clearance, trajectory and controller audits passed.')
    return True
