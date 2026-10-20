"""Physical checks and independent audits for the multi-lane work-zone model."""
from pathlib import Path
import copy
import json

from .behavior import (VEHICLE_LENGTH_M, Vehicle, courtesy_requests, idm_acceleration,
                       lane_change_candidate, motion)
from .model import simulate
from .test_tmc import verify_tmc

ROOT = Path(__file__).resolve().parent


def check_idm_basics():
    """Equilibrium gap, free-road acceleration, desired-speed cruise, exact stop."""
    v = 20.0
    equilibrium_gap = (2 + v * 1.5) / (1 - (v / 28) ** 4) ** .5
    assert abs(idm_acceleration(v, equilibrium_gap, 0, 28)) < 1e-10
    assert idm_acceleration(0, 1e9, 0, 28) > 0
    assert abs(idm_acceleration(28, 1e9, 0, 28)) < 1e-10
    assert motion(0, 1, -2, 1) == (.25, 0.0)
    return 'passed'


def check_lane_change_rules(config):
    """A fast follower with a short gap blocks even a mandatory merge."""
    car = Vehicle(0, 'main', 0, 0, 500, 5, 0)
    fast_follower = Vehicle(1, 'main', 0, 0, 490, 28, 1)
    assert lane_change_candidate(car, 1, [car, fast_follower], config, True, True) is None
    assert lane_change_candidate(car, 1, [car], config, True, True) is not None
    in_work_zone = Vehicle(2, 'main', 0, 0, 700, 20, 1)
    assert lane_change_candidate(in_work_zone, 0, [in_work_zone], config, True) is None
    return 'passed'


def check_courtesy(config):
    """A follower yields once, then gets its own turn; no yielding without closure."""
    merger = Vehicle(10, 'main', 0, 0, 600, 5, 0)
    follower = Vehicle(11, 'main', 0, 0, 560, 20, 1)
    assert courtesy_requests([merger, follower], config, True) == {11: merger}
    assert courtesy_requests([merger, follower], config, True, {11}) == {}
    assert courtesy_requests([merger, follower], config, False) == {}
    return 'passed'


def audit_run(config, mode, run):
    """Re-derive detector counts and safety facts from the raw trajectories."""
    assert run['metrics']['safety_interventions'] == 0
    counts = {(p, lane): 0 for p in config['detectors_m'] for lane in range(config['lanes'])}
    previous, occupancy = {}, {}
    for row in run['traces']:
        vid, lane, x = row['vehicle_id'], row['lane_id'], row['position_m']
        start = config['merge_position_m'] if row['origin'] == 'ramp' else 0
        old = previous.get(vid, start)
        assert x >= old - 1e-4, 'Vehicle moved backward'
        assert row['speed_mps'] >= 0
        if mode != 'open':
            assert not (lane == config['closed_lane']
                        and config['closure_start_m'] <= x < config['closure_end_m'])
        for p in config['detectors_m']:
            counts[(p, lane)] += int(old < p <= x)
        previous[vid] = x
        occupancy.setdefault((row['time_s'], lane), []).append(x)

    for positions in occupancy.values():
        positions.sort()
        assert all(b - a >= VEHICLE_LENGTH_M - 1e-3 for a, b in zip(positions, positions[1:])), \
            'Same-lane overlap'
    for (p, lane), count in counts.items():
        measured = sum(o['count'] for o in run['observations']
                       if o['position_m'] == p and o['lane_id'] == lane)
        assert count == measured, (p, lane, count, measured)

    safe_braking = config['lane_change_safe_braking_mps2']
    for lc in run['lane_changes']:
        assert abs(lc['to_lane'] - lc['from_lane']) == 1
        assert lc['target_follower_acceleration_after_mps2'] >= -safe_braking - 1e-4
        if lc['reason'] == 'closure_merge':
            assert config['merge_anticipation_m'] <= lc['position_m'] < config['closure_start_m']

    return {'conservation': 'passed every time step',
            'same_lane_overlap': 'none', 'negative_speed': 'none', 'closed_lane_violation': 'none',
            'detector_counts_by_lane': {f'{p}_lane_{lane}': n for (p, lane), n in counts.items()},
            'guard_interventions': run['metrics']['safety_interventions']}


def check_pulse_clears(config):
    """A finite arrival pulse must merge and exit; no vehicle stays trapped."""
    pulse = copy.deepcopy(config)
    pulse['demand'] = [{'start_s': 0, 'main_vph': 1800, 'ramp_vph': 300},
                       {'start_s': 120, 'main_vph': 0, 'ramp_vph': 0}]
    m = simulate(pulse, 'closure')['metrics']
    assert m['generated_vehicles'] == m['exited_vehicles'] == 70
    assert m['remaining_on_road'] == 0
    assert m['mandatory_merges'] > 0
    return m


def check_time_step(config, coarse_run):
    """Halve dt and report how the closure case changes (not forced to match)."""
    fine = copy.deepcopy(config)
    fine['dt_s'] /= 2
    fine_run = simulate(fine, 'closure')
    assert fine_run['metrics']['safety_interventions'] == 0
    keys = ['exited_vehicles', 'total_delay_vehicle_s', 'minimum_gap_m',
            'remaining_main_queue', 'remaining_ramp_queue', 'mandatory_merges']
    return {k: {f'dt_{config["dt_s"]}': coarse_run['metrics'][k],
                f'dt_{fine["dt_s"]}': fine_run['metrics'][k]} for k in keys}


def check_zero_demand(config):
    empty = copy.deepcopy(config)
    empty['duration_s'] = 60
    empty['demand'] = [{'start_s': 0, 'main_vph': 0, 'ramp_vph': 0}]
    run = simulate(empty)
    assert run['metrics']['generated_vehicles'] == run['metrics']['total_delay_vehicle_s'] == 0
    assert all(o['mean_speed_mps'] is None for o in run['observations'])
    return 'passed'


def check_overload(config):
    """Extreme demand must stay in boundary queues without unsafe insertions."""
    stress = copy.deepcopy(config)
    stress['duration_s'] = 120
    stress['demand'] = [{'start_s': 0, 'main_vph': 12000, 'ramp_vph': 1200}]
    m = simulate(stress)['metrics']
    assert m['remaining_main_queue'] > 0
    assert m['safety_interventions'] == 0
    return m


def verify():
    """Run all checks and write results/freeway/verification.json."""
    config = json.loads((ROOT / 'configs/default.json').read_text())
    report = {
        'equilibrium_free_motion_and_stop': check_idm_basics(),
        'unsafe_merge_rejected_safe_merge_accepted_closed_target_rejected': check_lane_change_rules(config),
        'courtesy_request_and_yielding_vehicle_turn': check_courtesy(config),
    }
    runs = {}
    for mode in ['open', 'closure', 'fixed', 'adaptive', 'adaptive_fault']:
        runs[mode] = simulate(config, mode)
        report[mode + '_audit'] = audit_run(config, mode, runs[mode])
    report['five_stage_loop'] = verify_tmc()
    report['finite_demand_pulse_clears_through_construction'] = check_pulse_clears(config)
    report['closure_time_step_sensitivity'] = check_time_step(config, runs['closure'])
    report['time_step_note'] = 'Lane/gap event timing is discrete; sensitivity is reported, not forced to match.'
    report['zero_demand_and_missing_measurements'] = check_zero_demand(config)
    report['overload_boundary_accounting'] = check_overload(config)

    output = ROOT.parent / 'results/freeway'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'verification.json').write_text(json.dumps(report, indent=2))
    print('Freeway physical audits and timestep sensitivity passed.')
    return True
