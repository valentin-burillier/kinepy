import kinepy.objects.config as cfg
import kinepy.math.geometry as geo

import numpy as np


class JointValueComputation:
    @staticmethod
    def do_not_compute_value(config: cfg.Config, joint: int, s1: int, s2: int) -> None:
        """Nothing to do"""

    @staticmethod
    def compute_revolute_value(config: cfg.Config, joint: int, s1: int, s2: int) -> None:
        s1_orientation = config.solids.orientation[s1]
        s2_orientation = config.solids.orientation[s2]

        diff = geo.Orientation.sub(s2_orientation, s1_orientation)
        config.joints.value[joint] = np.arctan2(diff[..., 1], diff[..., 0])

    @staticmethod
    def compute_prismatic_value(config: cfg.Config, joint: int, s1: int, s2: int) -> None:
        angle = config.joints.prismatic_angle1[joint]
        director = geo.Orientation.add(config.solids.orientation[s1], geo.Orientation.from_angle(angle))
        config.joints.value[joint] = geo.Geometry.dot(director, config.solids.position[s2] - config.solids.position[s1])[..., 0]

    @staticmethod
    def do_not_compute_continuity(config: cfg.Config, joint: int):
        pass

    @staticmethod
    def compute_revolute_continuity(config: cfg.Config, joint: int):
        geo.Orientation.make_angle_continuous(config.joints.value[joint])


class JointInput:
    @staticmethod
    def solve_revolute(config: cfg.Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...]):

        s1_point = geo.Position.point(config, s1, config.joints.revolute_p1[joint])
        s2_point = geo.Position.point(config, s2, config.joints.revolute_p2[joint])

        s1_ori = config.solids.orientation[s1]
        s2_ori = config.solids.orientation[s2]

        rotation = geo.Orientation.sub(s1_ori, s2_ori)
        total_rotation = geo.Orientation.add(rotation, geo.Orientation.from_angle(config.joints.value[joint]))

        geo.Geometry.rotate_eq(eq2, config, total_rotation)
        geo.Geometry.move_eq(eq2, config, s1_point + geo.Orientation.add(-s2_point, total_rotation))

    @staticmethod
    def solve_prismatic(config: cfg.Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...]):
        angle1, distance1, angle2, distance2 = config.joints.prismatic_angle1[joint], config.joints.prismatic_distance1[joint], config.joints.prismatic_angle2[joint], config.joints.prismatic_distance2[joint]
        s1_point = geo.Position.vector(config, s1, geo.Orientation.from_angle(angle1))
        s2_point = geo.Position.vector(config, s2, geo.Orientation.from_angle(angle2))

        total_rotation = geo.Orientation.sub(s1_point, s2_point)

        p1 = config.solids.position[s1] + geo.Geometry.z_det(s1_point) * distance1 + s1_point * config.joints.value[joint, ..., np.newaxis]
        p2 = geo.Geometry.z_det(s2_point) * distance2 + config.solids.position[s2]
        geo.Geometry.rotate_eq(eq2, config, total_rotation)
        geo.Geometry.move_eq(eq2, config, p1 + geo.Orientation.add(total_rotation, -p2))


class System:
    @staticmethod
    def set_up(config: cfg.Config):
        config.solids.position[...] = 0.0, 0.0
        config.solids.orientation[...] = 1.0, 0.0
        config.joint_states[:] = config.final_joint_states

    @staticmethod
    def clean_up(config: cfg.Config):
        eq = tuple(range(config.solids.count))
        geo.Geometry.move_eq(eq, config, -config.solids.position[0])
        geo.Geometry.rotate_eq(eq, config, config.solids.orientation[0] * (1, -1))


class Graph:
    @staticmethod
    def solve_rrr(config: cfg.Config, edges: tuple[geo.OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], solution_index: int):
        r"""
                0
               / \
              R0  R1
             /     \
            1 - R2- 2
        """
        eq0, eq1, eq2 = eqs
        r0, r1, r2 = edges

        # vectors in each eq
        v0 = geo.Joint.get_solid_point(config, r1) - geo.Joint.get_solid_point(config, r0)
        v1 = geo.Joint.get_solid_point(config, r2) - geo.Joint.get_solid_point(config, r0, geo.Joint.Direction.TARGET)
        v2 = geo.Joint.get_solid_point(config, r2, geo.Joint.Direction.TARGET) - geo.Joint.get_solid_point(config, r1, geo.Joint.Direction.TARGET)

        sq_a = geo.Geometry.sq_mag(v0)
        sq_b = geo.Geometry.sq_mag(v1)
        sq_c = geo.Geometry.sq_mag(v2)
        inv_ab = (sq_a * sq_b) ** -0.5

        sign = (1, -1)[solution_index]
        cos_angle = 0.5 * (sq_a + sq_b - sq_c) * inv_ab
        sin_angle = sign * (1 - cos_angle * cos_angle) ** 0.5

        total_rotation = geo.Orientation.add(geo.Orientation.sub(v0, v1) * inv_ab, np.r_['-1', cos_angle, sin_angle])
        geo.Geometry.rotate_eq(eq1, config, total_rotation)
        geo.Geometry.move_eq(eq1, config, geo.Joint.get_solid_point(config, r0) - geo.Joint.get_solid_point(config, r0, geo.Joint.Direction.TARGET))

        _v1 = geo.Joint.get_solid_point(config, r2) - geo.Joint.get_solid_point(config, r1)
        eq2_rotation = geo.Orientation.sub(_v1, v2) / sq_c
        geo.Geometry.rotate_eq(eq2, config, eq2_rotation)
        geo.Geometry.move_eq(eq2, config, geo.Joint.get_solid_point(config, r1) - geo.Joint.get_solid_point(config, r1, geo.Joint.Direction.TARGET))

    @staticmethod
    def solve_rrp(config: cfg.Config, edges: tuple[geo.OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], solution_index: int):
        r"""
                0
               / \
              R0  R1
             /     \
            1 - P2- 2
        """
        eq0, eq1, eq2 = eqs
        r0, r1, p2 = edges

        v0 = geo.Joint.get_solid_point(config, r1) - geo.Joint.get_solid_point(config, r0)

        _angle21, _distance21 = geo.Joint.get_point(config, p2)
        _angle22, _distance22 = geo.Joint.get_point(config, p2, geo.Joint.Direction.TARGET)
        v1 = geo.Orientation.add(geo.Joint.get_solid_orientation(config, p2), geo.Orientation.from_angle(_angle21))

        eq2_rotation = geo.Orientation.sub(v1, geo.Orientation.add(geo.Joint.get_solid_orientation(config, p2, geo.Joint.Direction.TARGET), geo.Orientation.from_angle(_angle22)))
        geo.Geometry.rotate_eq(eq2, config, eq2_rotation)

        sq_v0_v1 = geo.Geometry.sq_mag(v0)  # * geo.Geometry.sq_mag(v1) = 1

        sign = (1, -1)[solution_index]
        v0_v1_cos_angle = geo.Geometry.det(
            v1,
            geo.Joint.get_solid_position(config, p2) - geo.Joint.get_solid_point(config, r0, geo.Joint.Direction.TARGET) +
            geo.Joint.get_solid_point(config, r1, geo.Joint.Direction.TARGET) - geo.Joint.get_solid_position(config, p2, geo.Joint.Direction.TARGET)
        ) + (_distance21 - _distance22)

        v1_v0_sin_angle = sign * (sq_v0_v1 - v0_v1_cos_angle * v0_v1_cos_angle) ** 0.5
        total_rotation = geo.Orientation.add(geo.Orientation.sub(geo.Geometry.z_det(v1), v0), np.r_['-1', v0_v1_cos_angle, v1_v0_sin_angle]) / sq_v0_v1

        geo.Geometry.rotate_eq(eq0, config, total_rotation)
        geo.Geometry.move_eq(eq0, config, geo.Joint.get_solid_point(config, r0, geo.Joint.Direction.TARGET) - geo.Joint.get_solid_point(config, r0))
        geo.Geometry.move_eq(eq2, config, geo.Joint.get_solid_point(config, r1) - geo.Joint.get_solid_point(config, r1, geo.Joint.Direction.TARGET))

    @staticmethod
    def solve_ppr(config: cfg.Config, edges: tuple[geo.OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], solution_index: int):
        r"""
                0
               / \
              P0  P1
             /     \
            1 - R2- 2
        """
        eq0, eq1, eq2 = eqs
        p0, p1, r2 = edges

        _angle10, _distance10 = geo.Joint.get_point(config, p0)
        _angle20, _distance20 = geo.Joint.get_point(config, p0, geo.Joint.Direction.TARGET)
        v1 = geo.Orientation.add(geo.Joint.get_solid_orientation(config, p0), geo.Orientation.from_angle(_angle10))

        _angle11, _distance11 = geo.Joint.get_point(config, p1)
        _angle21, _distance21 = geo.Joint.get_point(config, p1, geo.Joint.Direction.TARGET)
        v2 = geo.Orientation.add(geo.Joint.get_solid_orientation(config, p1), geo.Orientation.from_angle(_angle11))

        eq1_rotation = geo.Orientation.sub(v1, geo.Orientation.add(geo.Joint.get_solid_orientation(config, p0, geo.Joint.Direction.TARGET), geo.Orientation.from_angle(_angle20)))
        geo.Geometry.rotate_eq(eq1, config, eq1_rotation)

        eq2_rotation = geo.Orientation.sub(v2, geo.Orientation.add(geo.Joint.get_solid_orientation(config, p1, geo.Joint.Direction.TARGET), geo.Orientation.from_angle(_angle21)))
        geo.Geometry.rotate_eq(eq2, config, eq2_rotation)

        vec_1 = geo.Joint.get_solid_position(config, p0) + (_distance10 - _distance20) * geo.Geometry.z_det(v1) + geo.Joint.get_solid_point(config, r2) - geo.Joint.get_solid_position(config, p0, geo.Joint.Direction.TARGET)
        vec_2 = geo.Joint.get_solid_position(config, p1) + (_distance11 - _distance21) * geo.Geometry.z_det(v2) + geo.Joint.get_solid_point(config, r2, geo.Joint.Direction.TARGET) - geo.Joint.get_solid_position(config, p1, geo.Joint.Direction.TARGET)

        target_point = vec_1 + (geo.Geometry.det(vec_2 - vec_1, v2) / geo.Geometry.det(v1, v2)) * v1
        geo.Geometry.move_eq(eq1, config, target_point - geo.Joint.get_solid_point(config, r2))
        geo.Geometry.move_eq(eq2, config, target_point - geo.Joint.get_solid_point(config, r2, geo.Joint.Direction.TARGET))


class Relation:
    @staticmethod
    def forward(value, r, v0):
        return value * r + v0

    @staticmethod
    def backward(value, r, v0):
        return (value - v0) / r

    transformations = backward, forward
    joint_solvers = {
        cfg.Joints.Type.REVOLUTE: JointInput.solve_revolute,
        cfg.Joints.Type.PRISMATIC: JointInput.solve_prismatic
    }

    @staticmethod
    def solve_standard_relation(config: cfg.Config, relation: int, source: int, destination: int, destination_type: int, eq1: tuple[int, ...], eq2: tuple[int, ...], direction: bool):
        v0 = config.relations.v0[relation]
        r = config.relations.r[relation]
        config.joints.value[destination] = Relation.transformations[direction](config.joints.value[source], r, v0)
        s1, s2 = config.joints.solids[destination]
        Relation.joint_solvers[destination_type](config, s1, s2, destination, eq1, eq2)

    @staticmethod
    def solve_belt(config: cfg.Config, relation: int, source: int, destination: int, _: int, eq1: tuple[int, ...], eq2: tuple[int, ...], direction: bool):
        v0 = config.relations.v0[relation]
        r1 = config.relations.belt_r1[relation]
        r2 = config.relations.belt_r2[relation]

        config.joints.value[destination] = Relation.transformations[direction](config.joints.value[source], r1 / r2, v0)
        s1, s2 = config.joints.solids[destination]
        JointInput.solve_revolute(config, s1, s2, destination, eq1, eq2)
