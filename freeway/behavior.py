"""Driver behavior: IDM car following, MOBIL-style lane changes, and merge courtesy.

All quantities are SI (m, s, m/s, m/s^2). Lane 0 is the leftmost lane.
"""
from dataclasses import dataclass
import math

VEHICLE_LENGTH_M = 5.0
FREE_ROAD_GAP_M = 1e9          # gap used when there is no leader
MIN_IDM_GAP_M = 0.01           # avoids division by zero in the IDM interaction term
CLOSURE_EXIT_BUFFER_M = 5      # closed lane stays blocked this far past closure_end_m
NO_ENTRY_LEAD_M = 100          # drivers avoid the closing lane from this far before the merge zone


@dataclass
class Vehicle:
    id: int
    origin: str                # 'main' or 'ramp'
    arrival: float             # time the vehicle joined its boundary queue
    entered: float             # time the vehicle was placed on the road
    x: float                   # front-bumper position
    v: float
    lane: int
    a: float = 0.0
    length: float = VEHICLE_LENGTH_M
    desired_speed: float = 28.0
    headway: float = 1.5       # IDM desired time headway T
    min_gap: float = 2.0       # IDM jam distance s0
    max_accel: float = 1.2     # IDM a
    comfortable_braking: float = 2.0   # IDM b
    previous_lane: int = -1
    lane_change_time: float = -100.0
    entry_speed: float = 0.0


# ---------------------------------------------------------------- car following

def idm_acceleration(v, gap, closing_speed, desired_speed, headway=1.5,
                     min_gap=2.0, max_accel=1.2, comfortable_braking=2.0):
    """IDM acceleration (Treiber, Hennecke & Helbing, 2000).

    gap is bumper-to-bumper distance to the leader; closing_speed = v - v_leader.
    """
    desired_gap = min_gap + max(0.0, v * headway +
        v * closing_speed / (2 * math.sqrt(max_accel * comfortable_braking)))
    return max_accel * (1 - (v / desired_speed) ** 4 -
                        (desired_gap / max(gap, MIN_IDM_GAP_M)) ** 2)


def motion(x, v, a, dt):
    """Ballistic update; a vehicle that would stop mid-step stops exactly."""
    if a < 0 and v + a * dt < 0:
        return x - v * v / (2 * a), 0.0
    return x + v * dt + .5 * a * dt * dt, max(0.0, v + a * dt)


def raw_acceleration(car, lead, lane, config, closure):
    """Unclipped IDM acceleration of `car` in `lane` behind `lead`.

    Before the closure, the closed lane ends in a standing obstacle at
    the usable taper end (a physical lane end, not a lower speed limit).
    """
    gap = lead.x - lead.length - car.x if lead else FREE_ROAD_GAP_M
    lead_v = lead.v if lead else car.v
    if closure and lane == config['closed_lane'] and car.x < lane_end(config):
        obstacle_gap = lane_end(config) - car.x
        if obstacle_gap < gap:
            gap, lead_v = obstacle_gap, 0.0
    return idm_acceleration(car.v, gap, car.v - lead_v, car.desired_speed,
                            car.headway, car.min_gap, car.max_accel, car.comfortable_braking)


# ---------------------------------------------------------------- road geometry

def lane_neighbors(vehicles, lane, x, exclude=None):
    """Nearest vehicle ahead (x' >= x) and behind (x' < x) in `lane`."""
    others = [c for c in vehicles if c.lane == lane and c is not exclude]
    ahead = [c for c in others if c.x >= x]
    behind = [c for c in others if c.x < x]
    return (min(ahead, key=lambda c: c.x) if ahead else None,
            max(behind, key=lambda c: c.x) if behind else None)


def lane_end(config):
    """Conservative lane end where the taper cannot fit a car plus side margins.

    Lane membership stays discrete; lateral placement is display-only.
    Old configurations without a taper keep their original lane end.
    """
    start = config.get('taper_start_m')
    if start is None:
        return config['closure_start_m']
    required = config['vehicle_width_m'] + 2 * config['taper_clearance_m']
    return start + (config['closure_start_m'] - start) * (1 - required / config['lane_width_m'])


def merge_urgency(car, config, closure):
    """Distance-based urgency, from zero at anticipation to one at usable end."""
    if not closure or car.lane != config['closed_lane']:
        return 0.0
    return max(0.0, min(1.0, (car.x - config['merge_anticipation_m']) /
                       (lane_end(config) - config['merge_anticipation_m'])))


def blocked(config, lane, x, enabled=True):
    """True if position x in `lane` is inside the work zone."""
    return (enabled and lane == config['closed_lane']
            and lane_end(config) <= x < config['closure_end_m'] + CLOSURE_EXIT_BUFFER_M)


def in_merge_zone(car, config, closure):
    """True if the car is in the closing lane and should seek a merge."""
    return (closure and car.lane == config['closed_lane']
            and config['merge_anticipation_m'] <= car.x < lane_end(config))


def avoids_closing_lane(car, target, config, closure):
    """Drivers do not voluntarily move into the lane that is about to end."""
    return (closure and target == config['closed_lane']
            and config['merge_anticipation_m'] - NO_ENTRY_LEAD_M <= car.x
            < config['closure_end_m'] + car.length)


# ---------------------------------------------------------------- lane changes

def lane_change_candidate(car, target, vehicles, config, closure, mandatory=False):
    """Evaluate a move to an adjacent lane (Kesting, Treiber & Helbing, 2007).

    Returns {'score', 'self_after', 'follower_after'} if the move is safe and
    worthwhile (or mandatory), otherwise None. Safety always uses the unclipped
    IDM acceleration, so braking limits cannot hide an unsafe insertion.
    """
    if not 0 <= target < config['lanes'] or blocked(config, target, car.x, closure):
        return None
    if avoids_closing_lane(car, target, config, closure):
        return None

    new_lead, new_follow = lane_neighbors(vehicles, target, car.x, car)
    min_gap = config['lane_change_min_gap_m']
    if new_lead and new_lead.x - new_lead.length - car.x < min_gap:
        return None
    if new_follow and car.x - car.length - new_follow.x < min_gap:
        return None

    # Safety criterion: neither the mover nor its new follower brakes too hard.
    safe_braking = config['lane_change_safe_braking_mps2']
    self_after = raw_acceleration(car, new_lead, target, config, closure)
    new_follow_after = raw_acceleration(new_follow, car, target, config, closure) if new_follow else 0
    if self_after < -safe_braking or new_follow_after < -safe_braking:
        return None

    # Incentive criterion: own gain plus politeness-weighted change for both followers.
    old_lead, old_follow = lane_neighbors(vehicles, car.lane, car.x, car)
    self_before = raw_acceleration(car, old_lead, car.lane, config, closure)
    old_follow_before = raw_acceleration(old_follow, car, car.lane, config, closure) if old_follow else 0
    old_follow_after = raw_acceleration(old_follow, old_lead, car.lane, config, closure) if old_follow else 0
    new_follow_before = raw_acceleration(new_follow, new_lead, target, config, closure) if new_follow else 0
    incentive = self_after - self_before + config['politeness'] * (
        old_follow_after - old_follow_before + new_follow_after - new_follow_before)
    incentive += config['keep_right_bias_mps2'] * (1 if target > car.lane else -1)

    urgency = merge_urgency(car, config, closure) if mandatory else 0.0
    if config.get('taper_start_m') is not None and mandatory:
        incentive += config['merge_urgency_gain_mps2'] * urgency ** 2
        mandatory = urgency >= config['mandatory_merge_urgency']
    if mandatory or incentive > config['lane_change_threshold_mps2']:
        return {'score': incentive, 'self_after': self_after, 'follower_after': new_follow_after}
    return None


def courtesy_requests(vehicles, config, closure, owed_passage=None):
    """Pick one target-lane follower that should open a gap for the lead merger.

    Declared behavioral assumption, not a calibrated zipper model. A follower
    in `owed_passage` already yielded once and now gets its own turn.
    Returns {yielding_vehicle_id: merging_vehicle}.
    """
    if not closure or not config.get('cooperative_merge', True):
        return {}
    merging = sorted([c for c in vehicles if in_merge_zone(c, config, closure)],
                     key=lambda c: c.x, reverse=True)
    if not merging:
        return {}
    car = merging[0]
    if config.get('taper_start_m') is not None and merge_urgency(car, config, closure) < config['courtesy_start_urgency']:
        return {}
    _, follower = lane_neighbors(vehicles, car.lane + 1, car.x)
    if (config.get('taper_start_m') is not None and follower
            and car.x - follower.x < car.length + config['lane_change_min_gap_m']):
        # A stopped merger cannot create rear clearance by moving forward at
        # the taper end. Let this overlapping follower pass and ask the next.
        return {}
    if (follower and follower.id not in (owed_passage or set())
            and car.x - follower.x <= config['yield_lookahead_m']):
        return {follower.id: car}
    return {}
