import numpy as np

from kinepy.math.geometry import *
from kinepy.objects.config import Config


class System:
    @staticmethod
    def set_up(config: Config):
        # OG
        config.results.solid_dynamics[:] = 0.0
        config.results.solid_dynamics[:, Config.SOLID_DYN_G, :] = Position.point(config, slice(None), config.solid_physics[:,  Config.SOLID_CFG_G, np.newaxis])

        config.results.joint_dynamics[:] = 0.0

    @staticmethod
    def clean_up(config: Config):
        """Nothing to do"""


class Newtons2ndLaw:
    _ground_is_not_a_free_body = "Clearly, zero does not belong here, yet there it is anyway !?"

    @staticmethod
    def force(config: Config, eq: tuple[int, ...]) -> np.ndarray:
        # -(sum(known_forces) - m.a) = sum(unknown_forces(Ext/eq))
        assert 0 not in eq, Newtons2ndLaw._ground_is_not_a_free_body
        return -np.sum(config.results.solid_dynamics[eq, Config.SOLID_DYN_FORCE, :], axis=0)

    @staticmethod
    def torque(config: Config, eq: tuple[int, ...], point: np.ndarray) -> np.ndarray:
        # -(sum(known_torques(g) - J.aa + pg x (sum(known_forces) - m.a)) = sum(unknown_torques(p))
        assert 0 not in eq, Newtons2ndLaw._ground_is_not_a_free_body
        # babar
        # np.cross: shape ((eq, 2, n) - (1, 2, n)) x (eq, 2, n) -> (eq, n)
        return -np.sum(config.results.solid_dynamics[eq, Config.SOLID_MOMENT_OF_INERTIA, :] + np.cross(config.results.solid_dynamics[eq, Config.SOLID_DYN_G, :] - point[np.newaxis, ...], config.results.solid_dynamics[eq, Config.SOLID_DYN_FORCE, :], axis=1), axis=0)

    @staticmethod
    def select_group(all_eqs: tuple[tuple[int, ...], ...], target_indices: tuple[int, ...], ground_eq: int) -> tuple[tuple[int, ...], float]:
        target_mask = sum(1 << eq_index for eq_index in target_indices)
        sign = 1.0
        if (target_mask >> ground_eq) & 1:
            target_mask ^= (1 << len(all_eqs)) - 1
            sign = -1.0
        return sum((eq for i, eq in enumerate(all_eqs) if (i >> target_mask) & 1), ()), sign


class Solid:
    @staticmethod
    def add_action(config: Config, solid: int, force: np.ndarray, torque: np.ndarray, point: np.ndarray):
        config.results.solid_dynamics[solid, Config.SOLID_DYN_FORCE, :] += force
        config.results.solid_dynamics[solid, Config.SOLID_DYN_TORQUE, :] += torque + np.cross(point - config.results.solid_dynamics[solid,Config.SOLID_DYN_G, :], force, axis=0) # noqa: false positive code is unreachable with np.cross

    @staticmethod
    def add_force(config: Config, solid: int, force: np.ndarray, point: np.ndarray):
        config.results.solid_dynamics[solid, Config.SOLID_DYN_FORCE, :] += force
        config.results.solid_dynamics[solid, Config.SOLID_DYN_TORQUE, :] += np.cross(point - config.results.solid_dynamics[solid,Config.SOLID_DYN_G, :], force, axis=0) # noqa: false positive code is unreachable with np.cross

    @staticmethod
    def add_torque(config: Config, solid: int, torque: np.ndarray):
        config.results.solid_dynamics[solid, Config.SOLID_DYN_TORQUE, :] += torque


class Joint(Joint):
    @staticmethod
    def set_oriented_action(config: Config, joint: OrientedJoint, force_1_2: np.ndarray, torque_1_2: np.ndarray, point: np.ndarray):
        Solid.add_action(config, Joint.get_solid(config, joint, True), force_1_2, torque_1_2, point)
        Solid.add_action(config, Joint.get_solid(config, joint), -force_1_2, -torque_1_2, point)
        Joint.set_force(config, joint[0], force_1_2 * (-1, 1)[joint[1]])
        Joint.set_torque(config, joint[0], torque_1_2 * (-1, 1)[joint[1]])

    @staticmethod
    def set_oriented_force(config: Config, joint: OrientedJoint, force_1_2: np.ndarray, point: np.ndarray):
        Solid.add_force(config, Joint.get_solid(config, joint, True), force_1_2, point)
        Solid.add_force(config, Joint.get_solid(config, joint), -force_1_2, point)
        return Joint.set_force(config, joint[0], force_1_2 * (-1, 1)[joint[1]])

    @staticmethod
    def set_oriented_torque(config: Config, joint: OrientedJoint, torque_1_2: np.ndarray):
        return Joint.set_torque(config, joint[0], torque_1_2 * (-1, 1)[joint[1]])

    @staticmethod
    def set_force(config: Config, joint: int, force_1_2: np.ndarray):
        config.results.joint_dynamics[joint, Config.JOINT_DYN_FORCE, :] = force_1_2

    @staticmethod
    def set_torque(config: Config, joint: int, torque_1_2: np.ndarray):
        config.results.joint_dynamics[joint, Config.JOINT_DYN_TORQUE, :] = torque_1_2


class JointInput:
    @staticmethod
    def solve_joint(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int, _point):
        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (1,), zero_holder)
        force_1_2 = sign * Newtons2ndLaw.force(config, eq)
        point = Position.point(config, s1, _point)
        torque_1_2 = sign * Newtons2ndLaw.torque(config, eq, point)

        Joint.set_oriented_action(config, (joint, True), force_1_2, torque_1_2, point)

    @staticmethod
    def solve_revolute(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        JointInput.solve_joint(config, s1, s2, joint, eq1, eq2, zero_holder, config.joint_physics[joint, Config.JOINT_P1, np.newaxis])

    @staticmethod
    def solve_prismatic(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        angle, dist = config.joint_physics[joint, (Config.JOINT_A1, Config.JOINT_D1), np.newaxis]
        JointInput.solve_joint(config, s1, s2, joint, eq1, eq2, zero_holder, dist * Orientation.from_angle(angle + np.pi * 0.5))


class Graph:
    @staticmethod
    def solve_rrr(config: Config, edges: tuple[OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], zero_holder: int):
        r"""
                0
               / \
              R0  R1
             /     \
            1 - R2- 2
        """
        r0, r1, r2 = edges
        p0, p1, p2 = Joint.get_revolute_application_point(config, r0), Joint.get_revolute_application_point(config, r1), Joint.get_revolute_application_point(config, r2)

        eq0, sign0 = Newtons2ndLaw.select_group(eqs, (2, 0), zero_holder)
        torque_1_2_p0 = sign0 * Newtons2ndLaw.torque(config, eq0, p0)

        eq1, sign1 = Newtons2ndLaw.select_group(eqs, (2,), zero_holder)
        torque_1_2_p1 = sign1 * Newtons2ndLaw.torque(config, eq1, p1)

        vec0, vec1 = p2 - p0, p2 - p1
        d = Geometry.dot(vec0, vec1)
        x, y = torque_1_2_p1 / d, torque_1_2_p0 / d
        force_1_2 = Geometry.z_det(vec0) * x + Geometry.z_det(vec1) * y
        Joint.set_oriented_force(config, r2, force_1_2, p2)

        force_1_0 = sign0 * Newtons2ndLaw.force(config, eq0)
        Joint.set_oriented_force(config, r0, -force_1_0, p0)

        force_0_2 = sign1 * Newtons2ndLaw.force(config, eq1)
        Joint.set_oriented_force(config, r1, force_0_2, p1)

    @staticmethod
    def solve_rrp(config: Config, edges: tuple[OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], zero_holder: int):
        r"""
                0
               / \
              R0  R1
             /     \
            1 - P2- 2
        """
        r0, r1, p2 = edges
        p0, p1, p2_ = Joint.get_revolute_application_point(config, r0), Joint.get_revolute_application_point(config, r1), Joint.get_prismatic_application_point(config, p2)

        eq0, sign0 = Newtons2ndLaw.select_group(eqs, (0, 2), zero_holder)
        torque_1_2_p0 = sign0 * Newtons2ndLaw.torque(config, eq0, p0)

        eq1, sign1 = Newtons2ndLaw.select_group(eqs, (2,), zero_holder)
        torque_1_2_p1 = sign1 * Newtons2ndLaw.torque(config, eq1, p1)

        p2_normal = Joint.get_prismatic_normal(config, p2)
        force_1_2_s = (torque_1_2_p0 - torque_1_2_p1) / Geometry.det(p1 - p0, p2_normal)
        force_1_2 = force_1_2_s[np.newaxis, :] * p2_normal
        torque_1_2_p2_ = torque_1_2_p0 - Geometry.det(p2_ - p0, p2_normal) * force_1_2_s

        Joint.set_oriented_action(config, p2, force_1_2, torque_1_2_p2_, p2_)

        force_1_0 = sign0 * Newtons2ndLaw.force(config, eq0)
        Joint.set_oriented_force(config, r0, -force_1_0, p0)

        force_0_2 = sign1 * Newtons2ndLaw.force(config, eq1)
        Joint.set_oriented_force(config, r1, force_0_2, p1)

    @staticmethod
    def solve_ppr(config: Config, edges: tuple[OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], zero_holder: int):
        r"""
                0
               / \
              P0  P1
             /     \
            1 - R2- 2
        """
        p0, p1, r2 = edges
        p0_, p1_, p2 = Joint.get_prismatic_application_point(config, p0), Joint.get_prismatic_application_point(config, p1), Joint.get_revolute_application_point(config, r2)

        n0, n1 = Joint.get_prismatic_normal(config, p0), Joint.get_prismatic_normal(config, p1)
        eq, sign = Newtons2ndLaw.select_group(eqs, (1, 2), zero_holder)
        force_0_12 = sign * Newtons2ndLaw.force(config, eq)
        d = Geometry.det(n0, n1)
        force_0_1, force_0_2 = (Geometry.det(force_0_12, n1) / d)[np.newaxis, :] * n0, (Geometry.det(n0, force_0_12) / d)[np.newaxis, :] * n1

        Joint.set_oriented_force(config, p0, force_0_1, p0_)
        Joint.set_oriented_force(config, p1, force_0_2, p1_)

        eq1, sign1 = Newtons2ndLaw.select_group(eqs, (2,), zero_holder)
        force_1_2 = sign1 * Newtons2ndLaw.force(config, eq1)
        Joint.set_oriented_force(config, r2, force_1_2, p2)

        torque_0_2 = sign1 * Newtons2ndLaw.torque(config, eq1, p1_)
        Joint.set_oriented_torque(config, p1, torque_0_2)
        torque_0_1 = sign * Newtons2ndLaw.torque(config, eq, p1_)
        Joint.set_oriented_torque(config, p0, torque_0_1)



