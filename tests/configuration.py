import unittest
import kinepy as kp
import kinepy.exceptions as ex


class ConfigurationTests(unittest.TestCase):
    def test_weird_and_wrong_usage(self):
        # same solid
        system: kp.System = kp.System()
        self.assertRaises(ex.ConstraintOnSameObjectError, system.add_prismatic, system.get_ground(), system.get_ground())

        # foreign solid
        system2: kp.System = kp.System()
        self.assertRaises(ex.UnrelatedObjectsError, system.add_prismatic, system.get_ground(), system2.get_ground())

    def test_over_determination(self):
        system: kp.System = kp.System()
        s1 = system.add_solid()
        self.assertRaises(ex.UnderDeterminationError, system.determine_computation_order)

        r0 = system.add_revolute(system.get_ground(), s1)
        r1 = system.add_revolute(system.get_ground(), s1)
        self.assertRaises(ex.OverDeterminationError, system.determine_computation_order)


if __name__ == '__main__':
    unittest.main()
