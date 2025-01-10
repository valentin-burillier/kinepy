import unittest
import kinepy as kp
from kinepy.objects.system import OverDeterminationError


class ConfigurationTests(unittest.TestCase):
    def test_weird_and_wrong_usage(self):
        # same solid
        system: kp.System = kp.System()
        self.assertRaises(ValueError, system.add_prismatic, system.get_ground(), system.get_ground())

        # foreign solid
        system2: kp.System = kp.System()
        self.assertRaises(ValueError, system.add_prismatic, system.get_ground(), system2.get_ground())

    def test_over_determination(self):
        system: kp.System = kp.System()
        s1 = system.add_solid()
        self.assertRaises(OverDeterminationError, system.determine_computation_order)

        r0 = system.add_revolute(system.get_ground(), s1)
        r1 = system.add_revolute(system.get_ground(), s1)
        self.assertRaises(OverDeterminationError, system.determine_computation_order)


if __name__ == '__main__':
    unittest.main()
