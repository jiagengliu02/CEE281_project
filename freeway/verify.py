"""Physical checks and independent audits for the multi-lane work-zone model."""
from pathlib import Path
import copy
import json
from .model import idm_acceleration, motion, simulate, Vehicle, lane_change_candidate, courtesy_requests

ROOT = Path(__file__).resolve().parent


def verify():
    """Run independent physical checks; write a branch-local results report."""
    c = json.loads((ROOT / 'configs/default.json').read_text())
    report = {}
    v = 20.0
    gap = (2 + v * 1.5) / (1 - (v / 28) ** 4) ** .5
    assert abs(idm_acceleration(v, gap, 0, 28)) < 1e-10
    assert idm_acceleration(0, 1e9, 0, 28) > 0
    assert abs(idm_acceleration(28, 1e9, 0, 28)) < 1e-10
    assert motion(0, 1, -2, 1) == (.25, 0.0)
    report['equilibrium_free_motion_and_stop'] = 'passed'

    # A fast target follower with a short gap must prevent even mandatory merging.
    car = Vehicle(0, 'main', 0, 0, 500, 5, 0)
    fast_follower = Vehicle(1, 'main', 0, 0, 490, 28, 1)
    assert lane_change_candidate(car, 1, [car, fast_follower], c, True, True) is None
    assert lane_change_candidate(car, 1, [car], c, True, True) is not None
    closed_candidate = Vehicle(2, 'main', 0, 0, 700, 20, 1)
    assert lane_change_candidate(closed_candidate, 0, [closed_candidate], c, True) is None
    report['unsafe_merge_rejected_safe_merge_accepted_closed_target_rejected'] = 'passed'

    # Yielding is a gap-creation behavior, followed by a turn for the yielding vehicle.
    source = Vehicle(10, 'main', 0, 0, 600, 5, 0)
    follower = Vehicle(11, 'main', 0, 0, 560, 20, 1)
    assert courtesy_requests([source, follower], c, True) == {11: source}
    assert courtesy_requests([source, follower], c, True, {11}) == {}
    assert courtesy_requests([source, follower], c, False) == {}
    report['courtesy_request_and_yielding_vehicle_turn'] = 'passed'

    runs = {}
    for mode in ['open', 'closure', 'fixed']:
        r = simulate(c, mode)
        assert r['metrics']['safety_interventions'] == 0
        counts = {(p, lane): 0 for p in c['detectors_m'] for lane in range(c['lanes'])}
        previous, occupancy = {}, {}
        for row in r['traces']:
            vid, lane = row['vehicle_id'], row['lane_id']
            old = previous.get(vid, c['merge_position_m'] if row['origin'] == 'ramp' else 0)
            assert row['position_m'] >= old - 1e-4
            assert row['speed_mps'] >= 0
            if mode != 'open':
                assert not (lane == c['closed_lane'] and c['closure_start_m'] <= row['position_m'] < c['closure_end_m'])
            for p in c['detectors_m']:
                counts[(p, lane)] += int(old < p <= row['position_m'])
            previous[vid] = row['position_m']
            occupancy.setdefault((row['time_s'], lane), []).append(row['position_m'])
        for positions in occupancy.values():
            positions.sort()
            assert all(b - a >= 5 - 1e-3 for a, b in zip(positions, positions[1:])), 'Same-lane overlap'
        for (p, lane), count in counts.items():
            measured = sum(o['count'] for o in r['observations'] if o['position_m'] == p and o['lane_id'] == lane)
            assert count == measured, (p, lane, count, measured)
        assert all(abs(lc['to_lane'] - lc['from_lane']) == 1 for lc in r['lane_changes'])
        assert all(c['merge_anticipation_m'] <= lc['position_m'] < c['closure_start_m'] for lc in r['lane_changes'] if lc['reason'] == 'closure_merge')
        assert all(lc['target_follower_acceleration_after_mps2'] >= -c['lane_change_safe_braking_mps2'] - 1e-4 for lc in r['lane_changes'])
        runs[mode] = r
        report[mode + '_audit'] = {'conservation': 'passed every time step',
            'same_lane_overlap': 'none', 'negative_speed': 'none', 'closed_lane_violation': 'none',
            'detector_counts_by_lane': {f'{p}_lane_{lane}': count for (p, lane), count in counts.items()},
            'guard_interventions': r['metrics']['safety_interventions']}

    # A finite arrival pulse must clear: cars must merge and exit, not remain trapped.
    pulse = copy.deepcopy(c)
    pulse['demand'] = [{'start_s': 0, 'main_vph': 1800, 'ramp_vph': 300},
                       {'start_s': 120, 'main_vph': 0, 'ramp_vph': 0}]
    cleared = simulate(pulse, 'closure')
    assert cleared['metrics']['generated_vehicles'] == cleared['metrics']['exited_vehicles'] == 70
    assert cleared['metrics']['remaining_on_road'] == 0
    assert cleared['metrics']['mandatory_merges'] > 0
    report['finite_demand_pulse_clears_through_construction'] = cleared['metrics']

    fine = copy.deepcopy(c)
    fine['dt_s'] /= 2
    fine_run = simulate(fine, 'closure')
    assert fine_run['metrics']['safety_interventions'] == 0
    keys = ['exited_vehicles', 'total_delay_vehicle_s', 'minimum_gap_m', 'remaining_main_queue', 'remaining_ramp_queue', 'mandatory_merges']
    report['closure_time_step_sensitivity'] = {k: {f'dt_{c["dt_s"]}': runs['closure']['metrics'][k],
        f'dt_{fine["dt_s"]}': fine_run['metrics'][k]} for k in keys}
    report['time_step_note'] = 'Lane/gap event timing is discrete; sensitivity is reported, not forced to match.'
    empty = copy.deepcopy(c)
    empty['duration_s'] = 60
    empty['demand'] = [{'start_s': 0, 'main_vph': 0, 'ramp_vph': 0}]
    r = simulate(empty)
    assert r['metrics']['generated_vehicles'] == r['metrics']['total_delay_vehicle_s'] == 0
    assert all(o['mean_speed_mps'] is None for o in r['observations'])
    report['zero_demand_and_missing_measurements'] = 'passed'
    stress = copy.deepcopy(c)
    stress['duration_s'] = 120
    stress['demand'] = [{'start_s': 0, 'main_vph': 12000, 'ramp_vph': 1200}]
    r = simulate(stress)
    assert r['metrics']['remaining_main_queue'] > 0
    assert r['metrics']['safety_interventions'] == 0
    report['overload_boundary_accounting'] = r['metrics']
    output = ROOT.parent / 'results/freeway'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'verification.json').write_text(json.dumps(report, indent=2))
    print('Freeway physical audits and timestep sensitivity passed.')
    return True
