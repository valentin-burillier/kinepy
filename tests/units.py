import math
import unittest
import kinepy.units as u


class UnitsTests(unittest.TestCase):
    def test_simple_arg(self):

        u.UnitSystem.use(u.Length.MILLIMETRE)

        @u.UnitSystem.function
        def test_f(x: u.Length.phy) -> u.scalar_type:
            return x

        self.assertEqual(test_f(1000.), 1.)
        self.assertEqual(test_f(x=1000.), 1.)

        @u.UnitSystem.function
        def test_f(x: u.Length.phy = 10.) -> u.scalar_type:
            return x

        self.assertEqual(test_f(1000.), 1.)
        self.assertEqual(test_f(x=1000.), 1.)
        self.assertEqual(test_f(), 10.)

    def test_return_value(self):

        u.UnitSystem.use(u.Length.MILLIMETRE)

        @u.UnitSystem.function
        def test_f(x: u.scalar_type) -> u.Length.phy:
            return x

        self.assertEqual(test_f(1.), 1000.)

    def test_wild_args(self):

        u.UnitSystem.use(u.Length.MILLIMETRE)

        @u.UnitSystem.function
        def test_f(*values: u.Length.phy) -> tuple[u.scalar_type, ...]:
            return values

        self.assertEqual(test_f(1000., 0.0, 500.), (1., 0.0, 0.5))

    def test_every_type_of_arg(self):

        u.UnitSystem.use(u.Length.MILLIMETRE)
        u.UnitSystem.use(u.Angle.DEGREE)
        u.UnitSystem.use(u.Mass.GRAM)

        @u.UnitSystem.function
        def test_f(x: u.Length.phy, /, y: u.Angle.phy, *args, z: u.Mass.phy) -> tuple[u.scalar_type, ...]:
            return x, y, *args, z

        self.assertEqual(test_f(1000., 180., z=1000.), (1., math.pi, 1.))
        self.assertEqual(test_f(1000., z=1000., y=180.), (1., math.pi, 1.))
        self.assertRaises(TypeError, test_f, x=1000., z=1000., y=180.)


if __name__ == '__main__':
    unittest.main()
