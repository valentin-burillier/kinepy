import math
import unittest
from kinepy.units import Physics


class UnitsTests(unittest.TestCase):
    def test_simple_arg(self):

        mm_to_meter = 1e-3
        Physics.set_unit(Physics.LENGTH, mm_to_meter, 'mm')

        @Physics.function
        def test_f(x: Physics.LENGTH) -> Physics.scalar_type:
            return x

        self.assertEqual(test_f(1000.), 1.)
        self.assertEqual(test_f(x=1000.), 1.)

        @Physics.function
        def test_f(x: Physics.LENGTH = 10.) -> Physics.scalar_type:
            return x

        self.assertEqual(test_f(1000.), 1.)
        self.assertEqual(test_f(x=1000.), 1.)
        self.assertEqual(test_f(), 10.)

    def test_return_value(self):
        mm_to_meter = 1e-3
        Physics.set_unit(Physics.LENGTH, mm_to_meter, 'mm')

        @Physics.function
        def test_f(x: Physics.scalar_type) -> Physics.LENGTH:
            return x

        self.assertEqual(test_f(1.), 1000.)

    def test_wild_args(self):
        mm_to_meter = 1e-3
        Physics.set_unit(Physics.LENGTH, mm_to_meter, 'mm')

        @Physics.function
        def test_f(*values: Physics.LENGTH) -> tuple[Physics.scalar_type, ...]:
            return values

        self.assertEqual(test_f(1000., 0.0, 500.), (1., 0.0, 0.5))

    def test_every_type_of_arg(self):
        mm_to_meter = 1e-3
        deg_to_rad = math.pi / 180.
        g_to_kg = 1e-3

        Physics.set_unit(Physics.LENGTH, mm_to_meter, 'mm')
        Physics.set_unit(Physics.ANGLE, deg_to_rad, '°')
        Physics.set_unit(Physics.MASS, g_to_kg, 'g')

        @Physics.function
        def test_f(x: Physics.LENGTH, /, y: Physics.ANGLE, *args, z: Physics.MASS) -> tuple[Physics.scalar_type, ...]:
            return x, y, *args, z

        self.assertEqual(test_f(1000., 180., z=1000.), (1., math.pi, 1.))
        self.assertEqual(test_f(1000., z=1000., y=180.), (1., math.pi, 1.))
        self.assertRaises(TypeError, test_f, x=1000., z=1000., y=180.)
