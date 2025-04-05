import unittest
import numpy as np
import warnings
import kinepy as kp

"""
Mostly crash tests, expected high code coverage
Very few correctness tests
"""


class InterfaceTests(unittest.TestCase):
    def test_ghost_solid(self):
        system = kp.System()
        gr = system.ground

        self.assertEqual(gr.mass, 0.0)
        self.assertEqual(gr.moment_of_inertia, 0.0)
        self.assertTrue(np.all(gr.g == np.zeros((2,))))

        self.assertRaises(AttributeError, lambda attr, v: setattr(gr, attr, v), 'mass', 1)
        self.assertRaises(AttributeError, lambda attr, v: setattr(gr, attr, v), 'moment_of_inertia', 1)
        self.assertRaises(AttributeError, lambda attr, v: setattr(gr, attr, v), 'g', (1, 0))

        self.assertRaises(ValueError, lambda: gr.x)

    def test_solid(self):
        system = kp.System()
        s1 = system.add_solid('BLOCK')
        s1.mass = 1.0
        s1.moment_of_inertia = 1.0
        s1.g = 1.0, 0

        s1.x.pilot()
        s1.y.pilot()
        s1.angle.pilot()

        s1.x.work()
        s1.y.work()
        s1.angle.work()

        n = 5
        system.set_frame_count(n)
        system.determine_computation_order()
        system.solve_kinematics()

        self.assertEqual(s1.get_angle().shape, (n,))
        self.assertEqual(s1.get_origin().shape, (2, n))
        self.assertEqual(s1.get_point((0, 0)).shape, (2, n))
        self.assertEqual(s1.get_vector((0, 0)).shape, (2, n))

    def test_joint(self):
        system = kp.System()
        s0, s1 = system.ground, system.add_solid('BLOCK')

        # work with any joint
        j = system.add_prismatic(s0, s1)

        j.pilot()
        j.work()

        n = 5
        system.set_frame_count(n)
        system.determine_computation_order()
        system.solve_kinematics()

        j.set_input(np.zeros((n,)))
        self.assertEqual(j.get_value().shape, (n,))
        self.assertEqual(j.get_force().shape, (2, n))
        self.assertEqual(j.get_torque().shape, (n,))

    def test_prismatic(self):
        system = kp.System()
        s0, s1 = system.ground, system.add_solid('BLOCK')

        p = system.add_prismatic(s0, s1)
        p.angle1 = 1.0
        p.distance1 = 1.0
        p.angle2 = 1.0
        p.distance2 = 1.0

    def test_revolute(self):
        system = kp.System()
        s0, s1 = system.ground, system.add_solid('BLOCK')

        r = system.add_revolute(s0, s1)
        r.p1 = 1, 0
        r.p2 = 1, 0

    def test_pin_slot(self):
        system = kp.System()
        s0, s1 = system.ground, system.add_solid('BLOCK')

        ps = system.add_pin_slot(s0, s1)
        ps.angle.p2 = 1, 0
        self.assertRaises(AttributeError, lambda: setattr(ps.angle, 'p1', (0, 0)))
        ps.angle.pilot()

        ps.sliding.angle1 = 1.0
        self.assertEqual(ps.sliding.angle2, 1.0)
        ps.sliding.distance1 = 1.0
        self.assertRaises(AttributeError, lambda: setattr(ps.sliding, 'distance2', 0))
        ps.sliding.pilot()

        system.set_frame_count(5)
        system.determine_computation_order()

        # piloting Revolut and Prismatic
        system.solve_kinematics()

    def test_translation(self):
        system = kp.System()
        s0, s1 = system.ground, system.add_solid('BLOCK')

        tt = system.add_translation(s0, s1)

        tt.y.angle1 = 0
        tt.y.angle2 = 2
        tt.y.distance2 = 2
        self.assertRaises(AttributeError, lambda: setattr(tt.y, 'distance1', 0))

        tt.x.pilot()
        tt.y.pilot()
        system.determine_computation_order()

    def test_rrr(self):
        warnings.filterwarnings('ignore')

        system = kp.System()
        s0, s1, s2 = system.ground, system.add_solid('BLOCK'), system.add_solid('BLOCK2')

        r = system.add_revolute(s0, s1)
        system.add_revolute(s0, s2)
        system.add_revolute(s1, s2)

        system.set_frame_count(5)
        system.determine_computation_order()

        system.solve_kinematics()
        # non user-set revolute value
        r.get_value()
        system.solve_dynamics()

    def test_rrp(self):
        warnings.filterwarnings('ignore')

        system = kp.System()
        s0, s1, s2 = system.ground, system.add_solid('BLOCK'), system.add_solid('BLOCK2')

        r = system.add_revolute(s0, s1)
        system.add_revolute(s0, s2)
        system.add_prismatic(s1, s2)

        system.set_frame_count(5)
        system.determine_computation_order()

        system.solve_kinematics()
        system.solve_dynamics()

    def test_ppr(self):
        warnings.filterwarnings('ignore')

        system = kp.System()
        s0, s1, s2 = system.ground, system.add_solid('BLOCK'), system.add_solid('BLOCK2')

        system.add_prismatic(s0, s1)
        system.add_prismatic(s0, s2)
        system.add_revolute(s1, s2)

        system.set_frame_count(5)
        system.determine_computation_order()

        system.solve_kinematics()
        system.solve_dynamics()
