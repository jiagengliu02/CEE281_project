"""Driver heterogeneity, reproducibility, and actual insertion checks."""
import copy
import json
from pathlib import Path
import unittest
from .drivers import PARAMETERS, driver_parameters, validate_drivers
from .model import FreewaySimulation

CONFIG = json.loads((Path(__file__).parent / 'configs/default.json').read_text())


class DriverTests(unittest.TestCase):
    def test_distribution_and_seed(self):
        samples = [driver_parameters(CONFIG, i) for i in range(3000)]
        for name in PARAMETERS:
            spec = CONFIG['driver_distribution'][name]
            values = [p[name] for p in samples]
            self.assertTrue(all(spec['min'] <= v <= spec['max'] for v in values))
            self.assertGreater(len(set(values)), 2900)
            self.assertAlmostEqual(sum(values) / len(values), spec['mean'], delta=spec['std'] * .08)
        self.assertEqual(samples[9], driver_parameters(CONFIG, 9))
        changed = copy.deepcopy(CONFIG)
        changed['driver_distribution']['seed'] += 1
        self.assertNotEqual(samples[9], driver_parameters(changed, 9))

    def test_paired_modes_and_insertion(self):
        for mode in ('open', 'closure', 'fixed', 'adaptive', 'adaptive_fault'):
            sim = FreewaySimulation(CONFIG, mode)
            sim.queues['main_0'].append((0, 9))
            self.assertTrue(sim.try_enter('main', 0, 0))
            car = sim.vehicles[0]
            expected = driver_parameters(CONFIG, 9)
            for key, value in expected.items():
                self.assertEqual(getattr(car, key), value)
            sim.compute_updates(0, 0)
            for key, value in expected.items():
                self.assertEqual(getattr(car, key), value)

    def test_validation_and_legacy(self):
        c = copy.deepcopy(CONFIG)
        del c['driver_distribution']
        self.assertEqual(driver_parameters(c, 0)['headway'], 1.5)
        for field, value in [('std', -1), ('min', 0), ('mean', float('nan'))]:
            c = copy.deepcopy(CONFIG)
            c['driver_distribution']['headway'][field] = value
            with self.assertRaises(ValueError):
                validate_drivers(c)
