import unittest
import kinepy as kp
import kinepy.objects.joints_solid as jo_so
import kinepy.objects.relations as rel
import kinepy.objects.config as cfg
import kinepy.math.geometry as geo
import numpy as np

"""
Correctness testing for kinepy kinematics:

Rules:
    Never use "zero" as a value if value is relevant (points can't be (0, 0), ratios can't be 1);
    No identical values
"""


class Kinematics(unittest.TestCase):
    def _assert_equal_f64(self, v1, v2, *, epsilon=1e-6):
        self.assertTrue(np.all(np.abs(v1 - v2) <= epsilon))

    def _assert_point_distance(self, p1, p2, *, distance, epsilon=1e-6):
        dd = np.linalg.norm(p1 - p2, axis=1)
        self.assertTrue(np.all(np.abs(dd - distance) < epsilon))
    
    def _assert_aligned(self, v1, v2, epsilon=1e-6):
        sin_angle = geo.Geometry.det(v1, v2)
        self.assertTrue(np.all(np.abs(sin_angle) <= epsilon))

    def _assert_revolute_constraints(self, revolute: jo_so.Revolute):
        # Force-Cast the joint to Revolute to get all properties 
        self.assertEqual(cfg.Joints.Type(revolute._type).primitive(), cfg.Joints.Type.REVOLUTE, "[Test]: You can't even write tests properly")
        revolute.__class__ = jo_so.Revolute

        p1, p2 = revolute.s1.get_point(revolute.p1), revolute.s2.get_point(revolute.p2)
        self._assert_point_distance(p1, p2, distance=0)

    def _assert_prismatic_constraints(self, prismatic: jo_so.Prismatic):
        # Force-Cast the joint to Prismatic to get all properties 
        self.assertEqual(cfg.Joints.Type(prismatic._type).primitive(), cfg.Joints.Type.PRISMATIC, "[Test]: You can't even write tests properly")
        prismatic.__class__ = jo_so.Prismatic

        _v1, _v2 = geo.Orientation.from_angle(prismatic.angle1), geo.Orientation.from_angle(prismatic.angle2)
        v1, v2 = prismatic.s1.get_vector(_v1), prismatic.s2.get_vector(_v2)
        self._assert_aligned(v1, v2)

        p1, p2 = prismatic.s1.get_point(prismatic.distance1 * geo.Geometry.z_det(_v1)), prismatic.s2.get_point(prismatic.distance2 * geo.Geometry.z_det(_v2))
        self._assert_aligned(p2 - p1, v1)

        y = geo.Geometry.det(v1, prismatic.s2.get_origin() - prismatic.s1.get_origin())
        self._assert_equal_f64(y, prismatic.distance1 - prismatic.distance2)

    _joint_assertions = {
        cfg.Joints.Type.REVOLUTE: _assert_revolute_constraints,
        cfg.Joints.Type.PRISMATIC: _assert_prismatic_constraints
    }

    def _assert_belt_constraints(self, belt: rel.Belt):
        self._assert_equal_f64(belt.j2.get_value(), belt.r1 / belt.r2 * belt.j1.get_value() + belt.v0)

    def _assert_relation_constraints(self, rel: rel.Relation):
        self._assert_equal_f64(rel.j2.get_value(), rel.r * rel.j1.get_value() + rel.v0)


    _relation_assertions = {
        cfg.Relations.Type.BELT: _assert_belt_constraints
    }

    def _assert_ground_constraints(self, ground: jo_so.GhostSolid):
        self._assert_point_distance(ground.get_origin(), (0, 0), distance=0)
        self._assert_equal_f64(ground.get_angle(), 0)

    def assert_system_validity(self, system: kp.System):
        """
        Checks that a system has been properly computed for kinematics:
            ground is at the origin
            revolute points taken from s1 and s2 match
            prismatic direction vectors taken from s1 and s2 are aligned
            prismatic application points taken from s1 and s2 are on a line directed by the direction vector
            prismatic distances conditions
            absolutely no NaN

        This ensures correctness of closed loop systems
        """
        config: cfg.Config = system._System__config

        self.assertFalse(np.any(np.isnan(config.solids.result_array)))
        self.assertFalse(np.any(np.isnan(config.joints.result_array)))

        self._assert_ground_constraints(system.ground)
        for joint_index, type_ in enumerate(config.joints.type_):
            jj = jo_so.Joint(config, joint_index)
            self._joint_assertions[cfg.Joints.Type(type_).primitive()](self, jj)

        for rel_index, type_ in enumerate(config.relations.type_):
            rr = rel.Relation(config, rel_index)
            self._relation_assertions.get(cfg.Relations.Type(type_), Kinematics._assert_relation_constraints)(self, rr)


    def allocate_resources(self, system: kp.System, n=1001):
        system.determine_computation_order()
        system.set_sim_parameters(n)
        return np.linspace(0, 1, n)

    @staticmethod
    def enhance_with_joint_orders(method, order_cnt=0, **variations):
        """
        Take a configuring method and exchange solids in joint creations or joints in relation creations everywhere that is declared
        """

        _orders = 1, -1

        total_var = 1
        for ll in variations.values():
            total_var *= len(ll)

        def n_method(self):
            for var_index in range(total_var):
                dd = {}
                for key, ll in variations.items():
                    var_index, ll_index = divmod(var_index, len(ll))
                    dd[key] = ll[ll_index]
                
                for index in range(1 << order_cnt):
                    order = tuple(_orders[(index >> i) & 1] for i in range(order_cnt))

                    with self.subTest(order=order, **dd):
                        system = kp.System()
                        method(self, system, order, **dd)
                        system.solve_kinematics()
                        self.assert_system_validity(system)

        return n_method


    def _pilot_r(self, system: kp.System, order=(1,)):
        ground = system.ground
        _s1 = system.add_solid()

        s1, s2 = (ground, _s1)[::order[0]]
        _p1, _p2 = np.array(((-1, 3), (2, 1)))
        r = system.add_revolute(s1, s2, p1=_p1, p2=_p2)
        r.pilot()

        t = self.allocate_resources(system)

        angle = t * 4 * np.pi
        r.set_input(angle)
        return r

    def _pilot_p(self, system: kp.System, order=(1,)):
        ground = system.ground
        _s1 = system.add_solid()

        s1, s2 = (ground, _s1)[::order[0]]
        p = system.add_prismatic(s1, s2, alpha1=np.pi / 4, distance1=1, alpha2=3 * np.pi / 5, distance2=-0.5)
        p.pilot()

        t = self.allocate_resources(system)

        sliding = t * 4
        p.set_input(sliding)
        return p

    test_pilot_r = enhance_with_joint_orders(_pilot_r, 1)
    test_pilot_p = enhance_with_joint_orders(_pilot_p, 1)

    def test_r(self):
        system = kp.System()
        r = self._pilot_r(system)
        system.solve_kinematics()
        self._assert_equal_f64(r.get_value(), r.s2.get_angle())

    def test_p(self):
        system = kp.System()
        p = self._pilot_p(system)
        system.solve_kinematics()
        sliding = p.get_value()[..., np.newaxis]

        _v1 = geo.Orientation.from_angle(p.angle1)
        v1 = p.s1.get_vector(_v1)
        self._assert_equal_f64(sliding, geo.Geometry.dot(v1, p.s2.get_origin() - p.s1.get_origin()))


    def _3dof(self, system: kp.System, order=()):
        s1 = system.add_solid()
        s1.x.pilot()
        s1.y.pilot()
        s1.angle.pilot()
        s1.angle.p2 = 1, 0

        t = self.allocate_resources(system)
        s1.x.set_input(4 * np.sin(t))
        s1.y.set_input(2 * np.cos(t))
        s1.angle.set_input(4 * np.pi * t)
        return s1
    
    test_3dof = enhance_with_joint_orders(_3dof, 0)

    def _rrr(self, system: kp.System, order=(1, 1, 1)):
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
        r1 = system.add_revolute(s1, s2, p1=p1, p2=p2)
        s1, s2 = (_s3, _s4)[::order[1]]
        p1, p2 = ((-1, -1), (1, -1))[::order[1]]
        r2 = system.add_revolute(s1, s2, p1=p1, p2=p2)
        s1, s2 = (_s1, _s5)[::order[2]]
        p1, p2 = ((5, 3), (2, 1))[::order[2]]
        r3 = system.add_revolute(s1, s2, p1=p1, p2=p2)

        t = self.allocate_resources(system)

        l1.set_input(np.sin(3 * t) - 4)
        l2.set_input(np.sin(4 * t) - 1)
        l3.set_input(np.sin(2 * t) + 1)
    
        return r1, r2, r3

    def test_rrr_declaration(self):
        system = kp.System()
        r1, r2, r3 = self._rrr(system)
        system.declare_direct_triangle(r1, r2, r3)
        system.solve_kinematics()
        p1, p2, p3 = r1.s1.get_point(r1.p1), r2.s1.get_point(r2.p1), r3.s1.get_point(r3.p1)
        self.assertTrue(np.all(geo.Geometry.det(p2 - p1, p3) >= 0))

        system = kp.System()
        r1, r2, r3 = self._rrr(system)
        system.declare_direct_triangle(r3, r2, r1)
        system.solve_kinematics()
        p1, p2, p3 = r1.s1.get_point(r1.p1), r2.s1.get_point(r2.p1), r3.s1.get_point(r3.p1)
        self.assertTrue(np.all(geo.Geometry.det(p2 - p1, p3) <= 0))

    test_rrr = enhance_with_joint_orders(_rrr, 3)

    def _rrp(self, system: kp.System, order=(1, 1, 1)):
        _s0 = system.ground
        _s1 = system.add_solid()
        _s2 = system.add_solid()
        _s3 = system.add_solid()

        l1 = system.add_prismatic(_s1, _s2)
        l1.pilot()
        
        s1, s2 = (_s0, _s1)[::order[0]]
        p1, p2 = ((1, 0), (0, -1))[::order[0]]
        system.add_revolute(s1, s2, p1=p1, p2=p2)
        s1, s2 = (_s2, _s3)[::order[1]]
        p1, p2 = ((1, 1), (1, -1))[::order[1]]
        system.add_revolute(s1, s2, p1=p1, p2=p2)
        s1, s2 = (_s0, _s3)[::order[2]]
        (a1, d1), (a2, d2) = ((np.pi / 6, 1), (5 * np.pi / 8, -2))[::order[2]]
        p = system.add_prismatic(s1, s2, alpha1=a1, distance1=d1, alpha2=a2, distance2=d2)

        t = self.allocate_resources(system)
        l1.set_input(4 + np.sin(np.pi * (t - 0.5)))
        return p

    def test_rrp_declaration(self):
        system = kp.System()
        p = self._rrp(system)
        system.solve_kinematics()
        v = np.array(p.get_value())

        system.declare_chose_lowest_value(p)
        system.solve_kinematics()
        self.assertTrue(np.all(v >= p.get_value()))

    test_rrp = enhance_with_joint_orders(_rrp, 3)

    def _ppr(self, system: kp.System, order=(1, 1, 1)):
        _s0 = system.ground
        _s1 = system.add_solid()
        _s2 = system.add_solid()
        _s3 = system.add_solid()

        s1, s2 = (_s0, _s1)[::order[0]]
        p1, p2 = ((1, 0), (0, -1))[::order[0]]
        system.add_revolute(s1, s2, p1=p1, p2=p2)
        s1, s2 = (_s1, _s2)[::order[1]]
        (a1, d1), (a2, d2) = ((np.pi / 6, 1), (5 * np.pi / 8, -2))[::order[1]]
        system.add_prismatic(s1, s2, alpha1=a1, distance1=d1, alpha2=a2, distance2=d2)
        s1, s2 = (_s2, _s3)[::order[2]]
        (a1, d1), (a2, d2) = ((-np.pi / 3, 0.5), (2 * np.pi / 7, 3))[::order[2]]
        system.add_prismatic(s1, s2, alpha1=a1, distance1=d1, alpha2=a2, distance2=d2)

        _s3.x.pilot()
        _s3.y.pilot()
        _s3.angle.pilot()

        t = self.allocate_resources(system)
        _s3.x.set_input(4 * np.sin(t))
        _s3.y.set_input(2 * np.cos(t))
        _s3.angle.set_input(4 * np.pi * t)

    test_ppr = enhance_with_joint_orders(_ppr, 3)

    def _relation(self, system: kp.System, order=(1, 1, 1, 1), relation=kp.System.add_distant_relation, joint1=kp.System.add_revolute, joint2=kp.System.add_revolute):
        _s0 = system.ground
        _s1 = system.add_solid()
        _s2 = system.add_solid()

        s1, s2 = (_s0, _s1)[::order[0]]
        _r1 = joint1(system, s1, s2)
        s1, s2 = (_s0, _s2)[::order[1]]
        _r2 = joint2(system, s1, s2)


        j1, j2 = (_r1, _r2)[::order[2]]
        rel = relation(system, j1, j2, v0=1.0, r=0.5)
        j1, j2 = (_r1, _r2)[::order[3]]
        j1.pilot()

        t = self.allocate_resources(system)

        j1.set_input(2 * np.pi * t)
        
    test_relation = enhance_with_joint_orders(
        _relation, 4, 
        relation=[
            kp.System.add_distant_relation,
            kp.System.add_effortless_relation,
        ],
        joint1 = [
            kp.System.add_revolute,
            kp.System.add_prismatic
        ],
        joint2 = [
            kp.System.add_revolute,
            kp.System.add_prismatic
        ]
    )

    def _belt(self, system: kp.System, order=(1, 1, 1, 1)):
        _s0 = system.ground
        _s1 = system.add_solid()
        _s2 = system.add_solid()

        s1, s2 = (_s0, _s1)[::order[0]]
        _r1 = system.add_revolute(s1, s2)
        s1, s2 = (_s0, _s2)[::order[1]]
        _r2 = system.add_revolute(s1, s2)


        j1, j2 = (_r1, _r2)[::order[2]]
        rel = system.add_belt(j1, j2, v0=1.0, r1=0.5, r2=3)
        j1, j2 = (_r1, _r2)[::order[3]]
        j1.pilot()

        t = self.allocate_resources(system)

        j1.set_input(2 * np.pi * t)

    test_belt = enhance_with_joint_orders(_belt, 4)


if __name__ == '__main__':
    unittest.main()