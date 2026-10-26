"""Reproducible per-driver parameters from independent truncated Gaussians."""
import math
import random

PARAMETERS = ('desired_speed', 'headway', 'max_accel', 'comfortable_braking', 'min_gap')


def validate_drivers(config):
    settings = config.get('driver_distribution')
    if settings is None:
        return
    if type(settings['seed']) is not int:
        raise ValueError('driver_distribution.seed must be an integer')
    for name in PARAMETERS:
        spec = settings[name]
        mean, std, lower, upper = (spec[k] for k in ('mean', 'std', 'min', 'max'))
        if not all(math.isfinite(v) for v in (mean, std, lower, upper)):
            raise ValueError(f'{name}: parameters must be finite')
        if not (0 < lower < upper and lower <= mean <= upper and std >= 0):
            raise ValueError(f'{name}: require 0 < min < max, min <= mean <= max, std >= 0')


def driver_parameters(config, vehicle_id):
    settings = config.get('driver_distribution')
    if settings is None:
        return dict(desired_speed=config['desired_speed_mps'] +
                    config['speed_variation_mps'] * ((vehicle_id % 5) - 2) / 2,
                    headway=1.5, max_accel=1.2, comfortable_braking=2.0, min_gap=2.0)
    values = {}
    for name in PARAMETERS:
        spec = settings[name]
        # Separate deterministic streams avoid dependence on insertion order or mode.
        rng = random.Random(f"{settings['seed']}:{vehicle_id}:{name}")
        for _ in range(10000):
            value = rng.gauss(spec['mean'], spec['std'])
            if spec['min'] <= value <= spec['max']:
                values[name] = value
                break
        else:
            raise ValueError(f'{name}: truncation range too narrow for configured standard deviation')
    return values
