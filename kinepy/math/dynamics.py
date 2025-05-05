import numpy as np

from kinepy.math.geometry import *
from kinepy.objects.config import Config


class System:
    @staticmethod
    def set_up(config: Config):
        # OG
        config.results.solid_dynamics[:] = 0.0
        config.results.solid_dynamics[:, :, Config.SOLID_DYN_G] = Position.point(config, slice(None), config.solid_physics[:, np.newaxis, Config.SOLID_CFG_G])

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
        return -np.sum(config.results.solid_dynamics[eq, :, Config.SOLID_DYN_FORCE], axis=0)

    @staticmethod
    def torque(config: Config, eq: tuple[int, ...], point: np.ndarray) -> np.ndarray:
        # -(sum(known_torques(g) - J.aa + pg x (sum(known_forces) - m.a)) = sum(unknown_torques(p))
        assert 0 not in eq, Newtons2ndLaw._ground_is_not_a_free_body
        # babar
        t = -np.sum(config.results.solid_dynamics[eq, :, Config.SOLID_DYN_TORQUE] + np.cross(config.results.solid_dynamics[eq, :, Config.SOLID_DYN_G] - point, config.results.solid_dynamics[eq, :, Config.SOLID_DYN_FORCE], axis=2), axis=0)
        return t[..., np.newaxis]

    @staticmethod
    def select_group(all_eqs: tuple[tuple[int, ...], ...], target_indices: tuple[int, ...], ground_eq: int) -> tuple[tuple[int, ...], float]:
        target_mask = sum(1 << eq_index for eq_index in target_indices)
        sign = 1.0
        if (target_mask >> ground_eq) & 1:
            target_mask ^= (1 << len(all_eqs)) - 1
            sign = -1.0
        return sum((eq for i, eq in enumerate(all_eqs) if (target_mask >> i) & 1), ()), sign


class Solid:
    @staticmethod
    def add_action(config: Config, solid: int, force: np.ndarray, torque: np.ndarray, point: np.ndarray):
        config.results.solid_dynamics[solid, :, Config.SOLID_DYN_FORCE] += force
        config.results.solid_dynamics[solid, :, Config.SOLID_DYN_TORQUE, np.newaxis] += torque + np.cross(point - config.results.solid_dynamics[solid, :, Config.SOLID_DYN_G], force, axis=-1)[..., np.newaxis] # noqa: false positive code is unreachable with np.cross

    @staticmethod
    def add_force(config: Config, solid: int, force: np.ndarray, point: np.ndarray):
        config.results.solid_dynamics[solid, :, Config.SOLID_DYN_FORCE] += force
        config.results.solid_dynamics[solid, :, Config.SOLID_DYN_TORQUE, np.newaxis] += np.cross(point - config.results.solid_dynamics[solid, :, Config.SOLID_DYN_G], force, axis=-1)[..., np.newaxis] # noqa: false positive code is unreachable with np.cross

    @staticmethod
    def add_torque(config: Config, solid: int, torque: np.ndarray):
        config.results.solid_dynamics[solid, :, Config.SOLID_DYN_TORQUE, np.newaxis] += torque


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
        config.results.joint_dynamics[joint, :, Config.JOINT_DYN_FORCE] = force_1_2

    @staticmethod
    def set_torque(config: Config, joint: int, torque_1_2: np.ndarray):
        config.results.joint_dynamics[joint, :, Config.JOINT_DYN_TORQUE, np.newaxis] = torque_1_2


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
        d, n0, n1 = Geometry.dot(vec0, vec1), Geometry.dot(vec0, vec0), Geometry.dot(vec1, vec1)
        x, y = (torque_1_2_p1 * d - torque_1_2_p0 * n1) / (d * d - n0 * n1), (torque_1_2_p0 * d - torque_1_2_p1 * n0) / (d * d - n0 * n1)
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
        force_1_2 = force_1_2_s * p2_normal
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
        force_0_1, force_0_2 = (Geometry.det(force_0_12, n1) / d) * n0, (Geometry.det(n0, force_0_12) / d) * n1

        Joint.set_oriented_force(config, p0, force_0_1, p0_)
        Joint.set_oriented_force(config, p1, force_0_2, p1_)

        eq1, sign1 = Newtons2ndLaw.select_group(eqs, (2,), zero_holder)
        force_1_2 = sign1 * Newtons2ndLaw.force(config, eq1)
        Joint.set_oriented_force(config, r2, force_1_2, p2)

        torque_0_2 = sign1 * Newtons2ndLaw.torque(config, eq1, p1_)
        Joint.set_oriented_torque(config, p1, torque_0_2)
        torque_0_1 = sign * Newtons2ndLaw.torque(config, eq, p1_)
        Joint.set_oriented_torque(config, p0, torque_0_1)


class JointInput:
    @staticmethod
    def solve_joint(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int, _point):
        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (1,), zero_holder)
        force_1_2 = sign * Newtons2ndLaw.force(config, eq)
        point = Position.point(config, s1, _point)
        torque_1_2 = sign * Newtons2ndLaw.torque(config, eq, point)

        Joint.set_oriented_action(config, (joint, True), force_1_2, torque_1_2, point)
        return force_1_2, torque_1_2

    @staticmethod
    def solve_revolute(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        return JointInput.solve_joint(config, s1, s2, joint, eq1, eq2, zero_holder, config.joint_physics[joint, Config.JOINT_P1])

    @staticmethod
    def solve_prismatic(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        angle, dist = config.joint_physics[joint, (Config.JOINT_A1, Config.JOINT_D1)]
        return JointInput.solve_joint(config, s1, s2, joint, eq1, eq2, zero_holder, dist * Orientation.from_angle(angle + np.pi * 0.5))


class Relation:
    @staticmethod
    def get_prismatic_effort(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        force_1_2, _ = JointInput.solve_prismatic(config, s1, s2, joint, eq1, eq2, zero_holder)
        director = Orientation.add(Orientation.get(config, s1), Orientation.from_angle(config.joint_physics[joint, Config.JOINT_A1]))
        return Geometry.dot(director, force_1_2)

    @staticmethod
    def get_revolute_effort(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        _, torque_1_2 = JointInput.solve_revolute(config, s1, s2, joint, eq1, eq2, zero_holder)
        return torque_1_2

    _effort_getter = {
        1: get_prismatic_effort,
        2: get_revolute_effort
    }

    @staticmethod
    def add_prismatic_effort(config: Config, joint, s1, s2, value):
        angle, dist = config.joint_physics[joint, (Config.JOINT_A1, Config.JOINT_D1)]
        point = Position.point(config, s1, dist * Orientation.from_angle(angle + np.pi * 0.5))
        force = Position.local_point(config, s1, Orientation.from_angle(config.joint_physics[joint, Config.JOINT_A1])) * value
        Solid.add_force(config, s1, force, point)
        Solid.add_force(config, s2, -force, point)

    @staticmethod
    def add_revolute_effort(config: Config, joint, s1, s2, value):
        Solid.add_torque(config, s1, value)
        Solid.add_torque(config, s2, -value)

    _effort_setter = {
        1: add_prismatic_effort,
        2: add_revolute_effort
    }

    @staticmethod
    def solve_effortless_relation(config: Config, relation: int, source: int, target: int, target_type: int, eq1, eq2, is_1_to_2, zero_holder):
        t1, t2 = config.joint_config[target, Config.JOINT_SOLIDS]
        return Relation._effort_getter[target_type](config, t1, t2, target, eq1, eq2, zero_holder)

    @staticmethod
    def solve_distant_relation(config: Config, relation: int, source: int, target: int, target_type: int, eq1, eq2, is_1_to_2, zero_holder):
        effort = Relation.solve_effortless_relation(config, relation, source, target, target_type, eq1, eq2, is_1_to_2, zero_holder)
        _r = config.relation_physics[relation, Config.RELATION_R]
        if not is_1_to_2:
            _r = 1 / _r
        source_type = config.joint_config[source, Config.JOINT_TYPE] & 3
        s1, s2 = config.joint_config[source, Config.JOINT_SOLIDS]
        Relation._effort_setter[source_type](config, source, s1, s2, effort * _r)

    @staticmethod
    def solve_gear_pair(config: Config, relation: int, source: int, target: int, target_type: int, eq1, eq2, is_1_to_2, zero_holder):
        p1, p2 = Joint.get_solid_point(config, (source, True)), Joint.get_solid_point(config, (target, True))
        _r, _pa = config.relation_physics[relation, (Config.RELATION_R, Config.RELATION_PRESSURE_ANGLE)]
        vec_1_2 = p1 - p2
        if is_1_to_2:
            r1, r2 = _r / (_r - 1), -1 / (_r - 1)
        else:
            r1, r2 = 1 / (_r - 1), -_r / (_r - 1)
        # ap = p1 + r1 * vec_1_2 = p2 + r2 * vec_1_2
        application_point = p1 + r1 * vec_1_2

        gear1, gear2 = config.relation_config[relation, (Config.RELATION_G1, Config.RELATION_G2) if is_1_to_2 else (Config.RELATION_G2, Config.RELATION_G1)]
        t2 = config.joint_config[target, Config.JOINT_S2]

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (t2 == gear2,), zero_holder)
        torque_1_2 = sign * Newtons2ndLaw.torque(config, eq, p2)

        force = Geometry.det_z(vec_1_2) / r2 / Geometry.sq_mag(vec_1_2) * torque_1_2
        rotation = np.zeros_like(force)
        rotation[:] = 1, np.tan(_pa)
        rotation[..., 1, np.newaxis] *= np.sign(torque_1_2)
        force = Orientation.add(force, rotation)

        Solid.add_force(config, gear2, force, application_point)
        Solid.add_force(config, gear1, -force, application_point)

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (1,), zero_holder)
        force_1_2 = sign * Newtons2ndLaw.force(config, eq)
        Joint.set_force(config, target, force_1_2)

    @staticmethod
    def solve_gear_rack(config: Config, relation: int, source: int, target: int, target_type: int, eq1, eq2, is_1_to_2, zero_holder):
        _r, _pa = config.relation_physics[relation, (Config.RELATION_R, Config.RELATION_PRESSURE_ANGLE)]
        if is_1_to_2:
            n = Joint.get_prismatic_normal(config, (target, True))
            p = Joint.get_solid_point(config, (source, True))
        else:
            n = Joint.get_prismatic_normal(config, (source, True))
            p = Joint.get_solid_point(config, (target, True))

        application_point = p - _r * n

        gear1, gear2 = config.relation_config[relation, (Config.RELATION_G1, Config.RELATION_G2) if is_1_to_2 else (Config.RELATION_G2, Config.RELATION_G1)]
        t2 = config.joint_config[target, Config.JOINT_S2]

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (gear2 == t2,), zero_holder)

        if is_1_to_2:
            effort_1_2 = Geometry.det(sign * Newtons2ndLaw.force(config, eq), n)
        else:
            effort_1_2 = sign * Newtons2ndLaw.torque(config, eq, p) / _r

        force = Geometry.det_z(n) * effort_1_2
        rotation = np.zeros_like(force)
        rotation[:] = 1, np.tan(_pa)
        rotation[..., 1, np.newaxis] *= np.sign(effort_1_2)
        force = Orientation.add(force, rotation)

        Solid.add_force(config, gear2, force, application_point)
        Solid.add_force(config, gear1, -force, application_point)

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (1,), zero_holder)
        force_1_2 = sign * Newtons2ndLaw.force(config, eq)
        Joint.set_force(config, target, force_1_2)
        if is_1_to_2:
            _ap = Joint.get_prismatic_application_point(config, (target, True))
            torque_1_2 = sign * Newtons2ndLaw.torque(config, eq, _ap)
            Joint.set_torque(config, target, torque_1_2)

    @staticmethod
    def solve_belt(config: Config, relation: int, source: int, target: int, target_type: int, eq1, eq2, is_1_to_2, zero_holder):
        p1, p2 = Joint.get_solid_point(config, (source, True)), Joint.get_solid_point(config, (target, True))
        r1, r2 = config.relation_physics[relation, (Config.RELATION_R1, Config.RELATION_R2) if is_1_to_2 else (Config.RELATION_R2, Config.RELATION_R1)]
        t0 = config.relation_physics[relation, Config.RELATION_T0]

        shaft1, shaft2 = config.relation_config[relation, (Config.RELATION_G1, Config.RELATION_G2) if is_1_to_2 else (Config.RELATION_G2, Config.RELATION_G1)]
        t2 = config.joint_config[target, Config.JOINT_S2]
        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (shaft2 == t2,), zero_holder)
        torque_1_2 = sign * Newtons2ndLaw.torque(config, eq, p2)

        _f1 = t0 + 0.5 * torque_1_2 / r2
        _f2 = t0 + 0.5 * torque_1_2 / r2

        vec_2_1 = p1 - p2
        ll = Geometry.sq_mag(vec_2_1)
        vec_2_1 /= ll

        rot = np.array(((ll[0, 0] - (r2 - r1) ** 2) ** 0.5, r1 - r2))
        f1 = Orientation.add(vec_2_1 * _f1, rot)
        f2 = Orientation.sub(vec_2_1 * _f2, rot)

        pa1 = p2 + Orientation.add(Geometry.z_det(vec_2_1), rot)
        pa2 = p2 + Orientation.sub(Geometry.det_z(vec_2_1), rot)

        Solid.add_force(config, shaft2, f1, pa1)
        Solid.add_force(config, shaft2, f2, pa2)

        Solid.add_force(config, shaft1, -f1, pa1)
        Solid.add_force(config, shaft1, -f2, pa2)


