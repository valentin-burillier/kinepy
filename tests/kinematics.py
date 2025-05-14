import unittest
import kinepy as kp
import kinepy.objects.joints_solid as jo_so
import kinepy.objects.config as cfg
import kinepy.math.geometry as geo
import numpy as np
import functools

"""
Correctness testing for kinepy kinematics:

Rules:
    Never use "zero" as a value if value is relevant (points can't be (0, 0), ratios can't be 1);
    No identical values
"""


class Kinematics(unittest.TestCase):
    def _assert_point_distance(self, p1, p2, /, *, distance, epsilon=1e-6):
        dd = np.linalg.norm(p1 - p2, axis=0)
        self.assertTrue(np.all(np.abs(dd - distance) < epsilon))
    
    def _assert_aligned(self, v1, v2, epsilon=1e-6):
        sin_angle = geo.Geometry.det(v1, v2)
        self.assertTrue(np.all(np.abs(sin_angle) <= epsilon))

    def _assert_revolute_constraints(self, revolute: jo_so.Revolute):
        # Force-Cast the joint to Revolute to get all properties 
        self.assertEqual(cfg.Joints.Type(revolute._type).primitive(), cfg.Joints.Type.REVOLUTE, "[Test]: You messed up the test")
        revolute.__class__ = jo_so.Revolute

        p1, p2 = revolute.s1.get_point(revolute.p1), revolute.s2.get_point(revolute.p2)
        self._assert_point_distance(p1, p2, distance=0)

    def _assert_prismatic_constraints(self, prismatic: jo_so.Prismatic):
        # Force-Cast the joint to Prismatic to get all properties 
        self.assertEqual(cfg.Joints.Type(prismatic._type).primitive(), cfg.Joints.Type.PRISMATIC, "[Test]: You messed up the test")
        prismatic.__class__ = jo_so.Prismatic

        _v1, _v2 = geo.Orientation.from_angle(prismatic.angle1), geo.Orientation.from_angle(prismatic.angle2)
        v1, v2 = prismatic.s1.get_vector(_v1), prismatic.s2.get_vector(_v2)
        self._assert_aligned(v1, v2)

        p1, p2 = prismatic.s1.get_point(prismatic.distance1 * geo.Geometry.z_det(_v1)), prismatic.s2.get_point(prismatic.distance2 * geo.Geometry.z_det(_v2))
        self._assert_aligned(p2 - p1, v1)


    _joint_assertions = {
        cfg.Joints.Type.REVOLUTE: _assert_revolute_constraints,
        cfg.Joints.Type.PRISMATIC: _assert_prismatic_constraints
    }

    def _assert_ground_constraints(self, ground: jo_so.GhostSolid):
        self._assert_point_distance(ground.get_origin(), (0, 0), distance=0)
        self._assert_point_distance(ground.get_angle(), (0,), distance=0)

    def assert_system_validity(self, system: kp.System):
        """
        Checks that a system has been properly computed for kinematics:
            ground is at the origin
            revolute points taken from s1 and s2 match
            prismatic direction vectors taken from s1 and s2 are aligned
            prismatic application points taken from s1 and s2 are on a ligne directed by the direction vector
        """
        config: cfg.Config = system._System__config

        self._assert_ground_constraints(system.ground)
        for joint_index, type_ in enumerate(config.joints.type_):
            jj = jo_so.Joint(config, joint_index)
            self._joint_assertions[cfg.Joints.Type(type_).primitive()](self, jj)

    def allocate_resources(self, system: kp.System, n=1001):
        system.determine_computation_order()
        system.set_sim_parameters(n)
        return np.linspace(0, 1, n)

    @staticmethod
    def enhance_with_joint_orders(method, order_cnt=0):
        """
        Take a configuring method and exchange orders of solids or joints every time declared
        """

        _orders = 1, -1

        @functools.wraps(method)
        def n_method(self):
            for index in range(1 << order_cnt):
                order = tuple(_orders[(index >> i) & 1] for i in range(order_cnt))

                with self.subTest(order=order):
                    system = kp.System()
                    method(self, system, order)
                    system.solve_kinematics()
                    self.assert_system_validity(system)

        return n_method


    def _pilot_r(self, system: kp.System, order):
        ground = system.ground
        _s1 = system.add_solid()

        s1, s2 = (ground, _s1)[::order[0]]
        _p1, _p2 = np.array(((-1, 3), (2, 1)))
        r = system.add_revolute(s1, s2, p1=_p1, p2=_p2)
        r.pilot()

        t = self.allocate_resources(system)

        angle = t * 4 * np.pi
        r.set_input(angle)

    def _pilot_p(self, system: kp.System, order):
        ground = system.ground
        _s1 = system.add_solid()

        s1, s2 = (ground, _s1)[::order[0]]
        p = system.add_prismatic(s1, s2, alpha1=np.pi / 4, distance1=1, alpha2=3 * np.pi / 5, distance2=-0.5)
        p.pilot()

        t = self.allocate_resources(system)

        sliding = t * 4
        p.set_input(sliding)

    def _rrr(self, system: kp.System, order):
        _s0 = system.ground
        _s1 = system.add_solid()
        _s2 = system.add_solid()
        _s3 = system.add_solid()
        _s4 = system.add_solid()
        _s5 = system.add_solid()

        l1 = system.add_prismatic(_s0, _s1)
        l2 = system.add_prismatic(_s2, _s3)
        l3 = system.add_prismatic(_s4, _s5)
        l1.pilot()
        l2.pilot()
        l3.pilot()

        s1, s2 = (_s0, _s2)[::order[0]]
        p1, p2 = ((1, 0), (0, 1))[::order[0]]
        system.add_revolute(s1, s2, p1=p1, p2=p2)
        s1, s2 = (_s3, _s4)[::order[1]]
        p1, p2 = ((-1, -1), (1, -1))[::order[1]]
        system.add_revolute(s1, s2, p1=p1, p2=p2)
        s1, s2 = (_s1, _s5)[::order[2]]
        p1, p2 = ((5, 3), (2, 1))[::order[2]]
        system.add_revolute(s1, s2, p1=p1, p2=p2)

        t = self.allocate_resources(system)

        l1.set_input(np.sin(3 * t) - 4)
        l2.set_input(np.sin(4 * t) - 1)
        l3.set_input(np.sin(2 * t) + 1)
    
    def test_rrr_declaration(self):
        system = kp.System()
        s1, s2, s3 = system.ground, system.add_solid(), system.add_solid()
        point = 1, 0
        r1, r2, r3 = system.add_revolute(s1, s2), system.add_revolute(s1, s3, p1=point), system.add_revolute(s2, s3, p1=point, p2=point)

        system.determine_computation_order()
        system.set_sim_parameters(2)

        system.declare_direct_triangle(r1, r2, r3)
        system.solve_kinematics()
        p1, p2, p3 = r1.s1.get_point(r1.p1), r2.s1.get_point(r2.p1), r3.s1.get_point(r3.p1)
        self.assertTrue(np.all(geo.Geometry.det(p2 - p1, p3) >= 0))

        system.clear_declarations()
        system.declare_direct_triangle(r3, r2, r1)
        system.solve_kinematics()
        p1, p2, p3 = r1.s1.get_point(r1.p1), r2.s1.get_point(r2.p1), r3.s1.get_point(r3.p1)
        self.assertTrue(np.all(geo.Geometry.det(p2 - p1, p3) <= 0))


    test_pilot_r = enhance_with_joint_orders(_pilot_r, 1)
    test_pilot_p = enhance_with_joint_orders(_pilot_p, 1)
    test_rrr = enhance_with_joint_orders(_rrr, 3)


if __name__ == '__main__':
    unittest.main()