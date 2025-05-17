import kinepy.objects.config as cfg
import kinepy.math.geometry as geo
import kinepy.math.calculus as cal

import numpy as np


class Interaction:
    @staticmethod
    def gravity(config: cfg.Config, index):
        first_index = config.interactions.first_action[index]
        actions = slice(first_index, first_index+config.solids.count)

        config.actions.application_point[actions] = config.solids.g_value[:]
        config.actions.force[actions] = config.solids.mass[:, np.newaxis, np.newaxis] * config.interactions.g_fields[index]
        config.actions.torque[actions] = 0.0

    @staticmethod
    def inertia(config: cfg.Config, index):
        first_index = config.interactions.first_action[index]
        actions = slice(first_index, first_index+config.solids.count)

        config.actions.application_point[actions] = config.solids.g_value[:]

        ori = config.solids.orientation
        _angle = np.arctan2(ori[..., 1], ori[..., 0])
        geo.Orientation.make_angle_continuous(_angle)
        config.actions.torque[actions] = cal.Derivation.second_derivative(_angle, -1, config.frame_time) * config.solids.moment_of_inertia[:, np.newaxis]
        config.actions.force[actions] = cal.Derivation.second_derivative(config.solids.g_value, -2, config.frame_time) * config.solids.mass[:, np.newaxis, np.newaxis]

    @staticmethod
    def linear_spring(config: cfg.Config, index):
        first_index = config.interactions.first_action[index]
        actions = slice(first_index, first_index+2)

        p1 = geo.Position.point(config, config.interactions.linear_spring_s1[index], config.interactions.linear_spring_p1[index])
        p2 = geo.Position.point(config, config.interactions.linear_spring_s2[index], config.interactions.linear_spring_p2[index])
        config.actions.application_point[actions] = p1, p2

        vec_1_2 = p2 - p1
        length = geo.Geometry.mag(vec_1_2)
        force_1_2 = -vec_1_2 * (length - config.interactions.spring_equilibrium_position[index]) * config.interactions.spring_stiffness[index] / length

        config.actions.force[actions] = (
            -force_1_2,
            force_1_2,
        )
        config.actions.torque[actions] = 0.0

    @staticmethod
    def twisting_spring(config: cfg.Config, index):
        first_index = config.interactions.first_action[index]
        actions = slice(first_index, first_index+2)

        r = config.interactions.twisting_spring_revolute[index]
        angle = config.joints.value[r]
        torque_1_2 = -(angle - config.interactions.spring_equilibrium_position[index]) * config.interactions.spring_stiffness[index]

        config.actions.torque[actions] = -torque_1_2, torque_1_2
        config.actions.force[actions] = 0.0

        p1 = geo.Position.point(config, config.joints.s1[r], config.joints.revolute_p1[r])
        config.actions.application_point[actions] = p1, p1

    mapping = dict(zip(cfg.Interactions.Type, (gravity, inertia, linear_spring, twisting_spring)))


class System:
    @staticmethod
    def set_up(config: cfg.Config):
        # OG
        config.solids.newtons_2nd_law_force[:] = 0.0
        config.solids.newtons_2nd_law_torque[:] = 0.0
        config.solids.g_value[:] = geo.Position.point(config, slice(None), config.solids.g[:, np.newaxis, :])

        config.joints.force[:] = 0.0
        config.joints.torque[:] = 0.0

        for index, type_ in enumerate(config.interactions.type_):
            Interaction.mapping[cfg.Interactions.Type(type_)](config, index)

        user_actions = config.actions.type_ == cfg.Actions.Type.USER
        ap = config.actions.user_point[user_actions, np.newaxis, :]
        solids = config.actions.solid[user_actions]
        config.actions.application_point[user_actions] = geo.Position.point(config, solids, ap)

        for index, type_ in enumerate(config.actions.type_):
            t = cfg.Actions.Type(type_)
            if t == cfg.Actions.Type.INTERNAL_OUTPUT:
                continue
            Solid.add_action(config, config.actions.solid[index], config.actions.force[index], config.actions.torque[index, :, np.newaxis], config.actions.application_point[index])

    @staticmethod
    def clean_up(config: cfg.Config):
        """Nothing to do"""


class Newtons2ndLaw:
    _ground_is_not_a_free_body = "Clearly, zero does not belong here, yet there it is anyway !?"

    @staticmethod
    def force(config: cfg.Config, eq: tuple[int, ...]) -> np.ndarray:
        # -(sum(known_forces) - m.a) = sum(unknown_forces(Ext/eq))
        assert 0 not in eq, Newtons2ndLaw._ground_is_not_a_free_body
        return -np.sum(config.solids.newtons_2nd_law_force[eq, ...], axis=0)

    @staticmethod
    def torque(config: cfg.Config, eq: tuple[int, ...], point: np.ndarray) -> np.ndarray:
        # -(sum(known_torques(g) - J.aa + pg x (sum(known_forces) - m.a)) = sum(unknown_torques(p))
        assert 0 not in eq, Newtons2ndLaw._ground_is_not_a_free_body
        # babar
        return -np.sum(config.solids.newtons_2nd_law_torque[eq, :, np.newaxis] + geo.Geometry.det(config.solids.g_value[eq, ...] - point, config.solids.newtons_2nd_law_force[eq, ...]), axis=0)

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
    def add_action(config: cfg.Config, solid: int, force: np.ndarray, torque: np.ndarray, point: np.ndarray):
        config.solids.newtons_2nd_law_force[solid] += force
        config.solids.newtons_2nd_law_torque[solid, :, np.newaxis] += torque + geo.Geometry.det(point - config.solids.g_value[solid], force)

    @staticmethod
    def add_force(config: cfg.Config, solid: int, force: np.ndarray, point: np.ndarray):
        config.solids.newtons_2nd_law_force[solid] += force
        config.solids.newtons_2nd_law_torque[solid, :, np.newaxis] += geo.Geometry.det(point - config.solids.g_value[solid], force) # noqa: false positive code is unreachable with np.cross

    @staticmethod
    def add_torque(config: cfg.Config, solid: int, torque: np.ndarray):
        config.solids.newtons_2nd_law_torque[solid, :, np.newaxis] += torque


class Joint(geo.Joint):
    @staticmethod
    def set_oriented_action(config: cfg.Config, joint: geo.OrientedJoint, force_1_2: np.ndarray, torque_1_2: np.ndarray, point: np.ndarray):
        Solid.add_action(config, Joint.get_solid(config, joint, Joint.Direction.TARGET), force_1_2, torque_1_2, point)
        Solid.add_action(config, Joint.get_solid(config, joint), -force_1_2, -torque_1_2, point)
        Joint.set_force(config, joint[0], force_1_2 * (-1, 1)[joint[1]])
        Joint.set_torque(config, joint[0], torque_1_2 * (-1, 1)[joint[1]])

    @staticmethod
    def set_oriented_force(config: cfg.Config, joint: geo.OrientedJoint, force_1_2: np.ndarray, point: np.ndarray):
        Solid.add_force(config, Joint.get_solid(config, joint, Joint.Direction.TARGET), force_1_2, point)
        Solid.add_force(config, Joint.get_solid(config, joint), -force_1_2, point)
        return Joint.set_force(config, joint[0], force_1_2 * (-1, 1)[joint[1]])

    @staticmethod
    def set_oriented_torque(config: cfg.Config, joint: geo.OrientedJoint, torque_1_2: np.ndarray):
        return Joint.set_torque(config, joint[0], torque_1_2 * (-1, 1)[joint[1]])

    @staticmethod
    def set_force(config: cfg.Config, joint: int, force_1_2: np.ndarray):
        config.joints.force[joint] = force_1_2

    @staticmethod
    def set_torque(config: cfg.Config, joint: int, torque_1_2: np.ndarray):
        config.joints.torque[joint, :, np.newaxis] = torque_1_2


class Graph:
    @staticmethod
    def solve_rrr(config: cfg.Config, edges: tuple[geo.OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], zero_holder: int):
        r"""
                0
               / \
              R0  R1
             /     \
            1 - R2- 2
        """
        r0, r1, r2 = edges
        p0, p1, p2 = Joint.get_revolute_application_point(config, r0), Joint.get_revolute_application_point(config, r1), Joint.get_revolute_application_point(config, r2)

        eq0, sign0 = Newtons2ndLaw.select_group(eqs, (0, 2), zero_holder)
        
        torque_1_2_p0 = sign0 * Newtons2ndLaw.torque(config, eq0, p0)

        eq1, sign1 = Newtons2ndLaw.select_group(eqs, (2,), zero_holder)
        torque_1_2_p1 = sign1 * Newtons2ndLaw.torque(config, eq1, p1)

        vec0, vec1 = p2 - p0, p2 - p1
        d, n0, n1 = geo.Geometry.dot(vec0, vec1), geo.Geometry.dot(vec0, vec0), geo.Geometry.dot(vec1, vec1)
        x, y = (torque_1_2_p1 * d - torque_1_2_p0 * n1) / (d * d - n0 * n1), (torque_1_2_p0 * d - torque_1_2_p1 * n0) / (d * d - n0 * n1)
        force_1_2 = geo.Geometry.z_det(vec0) * x + geo.Geometry.z_det(vec1) * y
        Joint.set_oriented_force(config, r2, force_1_2, p2)

        force_1_0 = sign0 * Newtons2ndLaw.force(config, eq0)
        Joint.set_oriented_force(config, r0, -force_1_0, p0)

        force_0_2 = sign1 * Newtons2ndLaw.force(config, eq1)
        Joint.set_oriented_force(config, r1, force_0_2, p1)

    @staticmethod
    def solve_rrp(config: cfg.Config, edges: tuple[geo.OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], zero_holder: int):
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
        force_1_2_s = (torque_1_2_p0 - torque_1_2_p1) / geo.Geometry.det(p1 - p0, p2_normal)
        force_1_2 = force_1_2_s * p2_normal
        torque_1_2_p2_ = torque_1_2_p0 - geo.Geometry.det(p2_ - p0, p2_normal) * force_1_2_s

        Joint.set_oriented_action(config, p2, force_1_2, torque_1_2_p2_, p2_)

        force_1_0 = sign0 * Newtons2ndLaw.force(config, eq0)
        Joint.set_oriented_force(config, r0, -force_1_0, p0)

        force_0_2 = sign1 * Newtons2ndLaw.force(config, eq1)
        Joint.set_oriented_force(config, r1, force_0_2, p1)

    @staticmethod
    def solve_ppr(config: cfg.Config, edges: tuple[geo.OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], zero_holder: int):
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
        d = geo.Geometry.det(n0, n1)
        force_0_1, force_0_2 = (geo.Geometry.det(force_0_12, n1) / d) * n0, (geo.Geometry.det(n0, force_0_12) / d) * n1

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
    def solve_joint(config: cfg.Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int, _point):
        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (1,), zero_holder)
        force_1_2 = sign * Newtons2ndLaw.force(config, eq)
        point = geo.Position.point(config, s1, _point)
        torque_1_2 = sign * Newtons2ndLaw.torque(config, eq, point)

        Joint.set_oriented_action(config, (joint, True), force_1_2, torque_1_2, point)
        return force_1_2, torque_1_2

    @staticmethod
    def solve_revolute(config: cfg.Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        return JointInput.solve_joint(config, s1, s2, joint, eq1, eq2, zero_holder, config.joints.revolute_p1[joint])

    @staticmethod
    def solve_prismatic(config: cfg.Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        angle, dist = config.joints.prismatic_angle1[joint], config.joints.prismatic_distance1[joint]
        return JointInput.solve_joint(config, s1, s2, joint, eq1, eq2, zero_holder, dist * geo.Orientation.from_angle(angle + np.pi * 0.5))


class Relation:
    @staticmethod
    def get_prismatic_effort(config: cfg.Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        force_1_2, _ = JointInput.solve_prismatic(config, s1, s2, joint, eq1, eq2, zero_holder)
        director = geo.Orientation.add(config.solids.orientation[s1], geo.Orientation.from_angle(config.joints.prismatic_angle1[joint]))
        return geo.Geometry.dot(director, force_1_2)

    @staticmethod
    def get_revolute_effort(config: cfg.Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        _, torque_1_2 = JointInput.solve_revolute(config, s1, s2, joint, eq1, eq2, zero_holder)
        return torque_1_2

    _effort_getter = {
        cfg.Joints.Type.PRISMATIC: get_prismatic_effort,
        cfg.Joints.Type.REVOLUTE: get_revolute_effort
    }

    @staticmethod
    def add_prismatic_effort(config: cfg.Config, joint, s1, s2, value):
        angle, dist = config.joints.prismatic_angle1[joint], config.joints.prismatic_distance1[joint]
        point = geo.Position.point(config, s1, dist * geo.Orientation.from_angle(angle + np.pi * 0.5))
        force = geo.Position.vector(config, s1, geo.Orientation.from_angle(angle)) * value
        Solid.add_force(config, s1, force, point)
        Solid.add_force(config, s2, -force, point)

    @staticmethod
    def add_revolute_effort(config: cfg.Config, joint, s1, s2, value):
        Solid.add_torque(config, s1, value)
        Solid.add_torque(config, s2, -value)

    _effort_setter = {
        cfg.Joints.Type.PRISMATIC: add_prismatic_effort,
        cfg.Joints.Type.REVOLUTE: add_revolute_effort
    }

    @staticmethod
    def solve_effortless_relation(config: cfg.Config, relation: int, source: int, target: int, target_type: cfg.Joints.Type, eq1, eq2, is_1_to_2, zero_holder):
        t1, t2 = config.joints.solids[target]
        return Relation._effort_getter[target_type](config, t1, t2, target, eq1, eq2, zero_holder)

    @staticmethod
    def solve_distant_relation(config: cfg.Config, relation: int, source: int, target: int, target_type: cfg.Joints.Type, eq1, eq2, is_1_to_2, zero_holder):
        effort = Relation.solve_effortless_relation(config, relation, source, target, target_type, eq1, eq2, is_1_to_2, zero_holder)
        _r = config.relations.r[relation]
        if not is_1_to_2:
            _r = 1 / _r
        source_type = cfg.Joints.Type(config.joints.type_[source]).primitive()
        s1, s2 = config.joints.solids[source]
        Relation._effort_setter[source_type](config, source, s1, s2, effort * _r)

    @staticmethod
    def solve_gear_pair(config: cfg.Config, relation: int, source: int, target: int, target_type: int, eq1, eq2, is_1_to_2, zero_holder):
        p1, p2 = Joint.get_solid_point(config, (source, True)), Joint.get_solid_point(config, (target, True))
        _r, _pa = config.relations.r[relation], config.relations.gear_pressure_angle[relation]
        vec_1_2 = p2 - p1
        if is_1_to_2:
            r1, r2 = _r / (_r - 1), 1 / (_r - 1)
        else:
            r1, r2 = -1 / (_r - 1), -_r / (_r - 1)
        # ap = p1 + r1 * vec_1_2 = p2 + r2 * vec_1_2
        application_point = p1 + r1 * vec_1_2

        _g1, _g2 = config.relations.g1[relation], config.relations.g2[relation]
        gear1, gear2 = (_g1, _g2) if is_1_to_2 else (_g2, _g1)
        t2 = config.joints.s2[target]

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (t2 == gear2,), zero_holder)
        torque_1_2 = sign * Newtons2ndLaw.torque(config, eq, p2)

        force = geo.Geometry.z_det(vec_1_2) / r2 / geo.Geometry.sq_mag(vec_1_2) * torque_1_2
        rotation = np.zeros_like(force)
        rotation[:] = 1, np.tan(_pa)
        rotation[..., 1, np.newaxis] *= np.sign(torque_1_2)
        force = geo.Orientation.add(force, rotation)

        Solid.add_force(config, gear2, force, application_point)
        Solid.add_force(config, gear1, -force, application_point)

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (1,), zero_holder)
        force_1_2 = sign * Newtons2ndLaw.force(config, eq)
        Joint.set_force(config, target, force_1_2)

    @staticmethod
    def solve_gear_rack(config: cfg.Config, relation: int, source: int, target: int, target_type: int, eq1, eq2, is_1_to_2, zero_holder):
        _r, _pa = config.relations.r[relation], config.relations.gear_pressure_angle[relation]
        if is_1_to_2:
            n = Joint.get_prismatic_normal(config, (target, True))
            p = Joint.get_solid_point(config, (source, True))
        else:
            n = Joint.get_prismatic_normal(config, (source, True))
            p = Joint.get_solid_point(config, (target, True))
        application_point = p - _r * n

        _g1, _g2 = config.relations.g1[relation], config.relations.g2[relation]
        gear1, gear2 = (_g1, _g2) if is_1_to_2 else (_g2, _g1)
        t2 = config.joints.s2[target]

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (gear2 == t2,), zero_holder)
        if is_1_to_2:
            effort_1_2 = geo.Geometry.det(sign * Newtons2ndLaw.force(config, eq), n)
        else:
            effort_1_2 = sign * Newtons2ndLaw.torque(config, eq, p) / _r

        force = geo.Geometry.det_z(n) * effort_1_2
        rotation = np.zeros_like(force)
        rotation[:] = 1, np.tan(_pa)
        rotation[..., 1, np.newaxis] *= -np.sign(effort_1_2)
        force = geo.Orientation.add(force, rotation)

        Solid.add_force(config, gear2, force, application_point)
        Solid.add_force(config, gear1, -force, application_point)

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (1,), zero_holder)
        force_1_2 = sign * Newtons2ndLaw.force(config, eq)
        Joint.set_force(config, target, force_1_2)
        if is_1_to_2:
            _ap = Joint.get_prismatic_application_point(config, (target, True))
            torque_1_2 = sign * Newtons2ndLaw.torque(config, eq, application_point)
            Joint.set_torque(config, target, torque_1_2)

    @staticmethod
    def solve_belt(config: cfg.Config, relation: int, source: int, target: int, target_type: int, eq1, eq2, is_1_to_2, zero_holder):
        p1, p2 = Joint.get_solid_point(config, (source, True)), Joint.get_solid_point(config, (target, True))

        _r1, _r2 = config.relations.belt_r1[relation], config.relations.belt_r2[relation]
        r1, r2 = (_r1, _r2) if is_1_to_2 else (_r2, _r1)
        t0 = config.relations.belt_t0[relation]

        _g1, _g2 = config.relations.g1[relation], config.relations.g2[relation]
        pulley1, pulley2 = (_g1, _g2) if is_1_to_2 else (_g2, _g1)
        t2 = config.joints.s2[target]

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (pulley2 == t2,), zero_holder)
        torque_1_2 = sign * Newtons2ndLaw.torque(config, eq, p2)

        _f1 = t0 - 0.5 * torque_1_2 / r2
        _f2 = t0 + 0.5 * torque_1_2 / r2

        vec_2_1 = p1 - p2
        ll = geo.Geometry.sq_mag(vec_2_1)
        vec_2_1 /= ll

        assert np.all((r1 - r2) ** 2 <= ll), "Definitely impossible pulley disposition: (r1 - r2) ^ 2 > d ^ 2"

        rot = np.array(((ll[0, 0] - (r2 - r1) ** 2) ** 0.5, r1 - r2))
        f1 = geo.Orientation.add(vec_2_1 * _f1, rot)
        f2 = geo.Orientation.sub(vec_2_1 * _f2, rot)

        pa1 = p2 + geo.Orientation.add(r2 * geo.Geometry.z_det(vec_2_1), rot)
        pa2 = p2 + geo.Orientation.sub(r2 * geo.Geometry.det_z(vec_2_1), rot)

        Solid.add_force(config, pulley2, f1, pa1)
        Solid.add_force(config, pulley2, f2, pa2)

        Solid.add_force(config, pulley1, -f1, pa1)
        Solid.add_force(config, pulley1, -f2, pa2)

        eq, sign = Newtons2ndLaw.select_group((eq1, eq2), (1,), zero_holder)
        force_1_2 = sign * Newtons2ndLaw.force(config, eq)
        Joint.set_force(config, target, force_1_2)

