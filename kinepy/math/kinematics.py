import numpy as np

from kinepy.math.geometry import *
from kinepy.objects.config import Config


class JointValueComputation:
    @staticmethod
    def do_not_compute_value(config: Config, joint: int, s1: int, s2: int) -> None:
        pass

    @staticmethod
    def compute_revolute_value(config: Config, joint: int, s1: int, s2: int) -> None:

        s1_orientation = Orientation.get(config, s1)
        s2_orientation = Orientation.get(config, s2)

        diff = Orientation.sub(s1_orientation, s2_orientation)

        config.results.joint_values[joint] = np.atan2(diff[1, ...], diff[0, ...])

    @staticmethod
    def compute_prismatic_value(config: Config, joint: int, s1: int, s2: int) -> None:
        pass
        # Geometry.det()
        # normal = Position.local_point(solid_values, s1, p1)
        # s1_point = normal + Position.get(solid_values, s1)
        # s2_point = Position.point(solid_values, s2, p2)
#
        # return Geometry.dot(normal, s2_point - s1_point) * Geometry.inv_mag(normal)

    @staticmethod
    def do_not_compute_continuity(value: np.ndarray):
        pass

    @staticmethod
    def compute_revolute_continuity(value: np.ndarray):
        Orientation.make_angle_continuous(value)


class JointInput:
    @staticmethod
    def solve_revolute(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...]):

        s1_point = Position.point(config, s1, config.joint_physics[joint, :2])
        s2_point = Position.point(config, s2, config.joint_physics[joint, 2:])

        s1_ori = Orientation.get(config, s1)
        s2_ori = Orientation.get(config, s2)

        rotation = Orientation.sub(s1_ori, s2_ori)
        total_rotation = Orientation.add(rotation, Orientation.from_angle(config.results.joint_values[joint]))

        Geometry.move_eq(eq2, config, -s2_point)
        Geometry.rotate_eq(eq2, config, total_rotation)
        Geometry.move_eq(eq2, config, s1_point)

    @staticmethod
    def solve_prismatic(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...]):
        angle1, distance1, angle2, distance2 = config.joint_physics[joint][:, np.newaxis]
        s1_point = Orientation.add(Orientation.get(config, s1), Orientation.from_angle(angle1))
        s2_point = Orientation.add(Orientation.get(config, s2), Orientation.from_angle(angle2))

        total_rotation = Orientation.sub(s1_point, s2_point)

        Geometry.move_eq(eq2, config, -s2_point * distance2 - Position.get(config, s2))
        Geometry.rotate_eq(eq2, config, total_rotation)
        Geometry.move_eq(eq2, config, Position.get(config, s1) + Geometry.det_z(s1_point) * distance1 + s1_point * config.results.joint_values[np.newaxis, joint, :])


class System:
    @staticmethod
    def set_up(solid_values, joint_values, n_solid, n_joint, frame_count):
        solid_values.resize((n_solid, 4, frame_count), refcheck=False)
        # shapes: m, 4, n <-  1, 4, 1
        solid_values[:] = ((0.,), (0.,), (1.0,), (0.,)),

        joint_values.resize((n_joint, frame_count), refcheck=False)
        joint_values[:] = 0.0

    @staticmethod
    def clean_up(solid_values):
        Geometry.move_eq(tuple(range(solid_values.shape[0])), solid_values, -Position.get(solid_values, 0))
        Geometry.rotate_eq(tuple(range(solid_values.shape[0])), solid_values, Orientation.get(solid_values, 0) * np.array([[1], [-1]]))


class Graph:
    @staticmethod
    def solve_rrr(config: Config, edges: tuple[OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], solution_index: int):
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
        v0 = Joint.get_solid_point(config, r1) - Joint.get_solid_point(config, r0)
        v1 = Joint.get_solid_point(config, r2) - Joint.get_solid_point(config, r0, True)
        v2 = Joint.get_solid_point(config, r2, True) - Joint.get_solid_point(config, r1, True)

        sq_a = Geometry.sq_mag(v0)
        sq_b = Geometry.sq_mag(v1)
        sq_c = Geometry.sq_mag(v2)
        inv_ab = (sq_a * sq_b) ** -0.5

        sign = (1, -1)[solution_index]
        cos_angle = 0.5 * (sq_a + sq_b - sq_c) * inv_ab
        sin_angle = sign * (1 - cos_angle * cos_angle) ** 0.5

        total_rotation = Orientation.add(Orientation.sub(v0, v1) * inv_ab, np.r_[cos_angle[np.newaxis, :], sin_angle[np.newaxis, :]])
        Geometry.rotate_eq(eq1, config, total_rotation)
        Geometry.move_eq(eq1, config, Joint.get_solid_point(config, r0) - Joint.get_solid_point(config, r0, True))

        _v1 = Joint.get_solid_point(config, r2) - Joint.get_solid_point(config, r1)
        eq2_rotation = Orientation.sub(v2, _v1) / sq_c
        Geometry.rotate_eq(eq2, config, eq2_rotation)
        Geometry.move_eq(eq2, config, Joint.get_solid_point(config, r1) - Joint.get_solid_point(config, r1, True))

    @staticmethod
    def solve_rrp(config: Config, edges: tuple[OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], solution_index: int):
        r"""
                0
               / \
              R0  R1
             /     \
            1 - P2- 2
        """
        eq0, eq1, eq2 = eqs
        r0, r1, p2 = edges

        v0 = Joint.get_solid_point(config, r0) - Joint.get_solid_point(config, r1)

        _angle21, _distance21 = Joint.get_point(config, p2)
        _angle22, _distance22 = Joint.get_point(config, p2, True)
        v1 = Orientation.add(Joint.get_solid_orientation(config, p2), Orientation.from_angle(_angle21))

        eq2_rotation = Orientation.sub(v1, Orientation.add(Joint.get_solid_orientation(config, p2, True), Orientation.from_angle(_angle22)))
        Geometry.rotate_eq(eq2, config, eq2_rotation)

        sq_v0_v1 = Geometry.sq_mag(v0)  # * Geometry.sq_mag(v1) = 1

        sign = (1, -1)[solution_index]
        v0_v1_cos_angle = Geometry.dot(
            v1,
            Joint.get_solid_position(config, p2) - Joint.get_solid_point(config, r0, True) +
            Joint.get_solid_point(config, r1, True) - Joint.get_solid_position(config, p2, True)
        ) + (_distance21 - _distance22)
        v1_v0_sin_angle = sign * (sq_v0_v1 - v0_v1_cos_angle * v0_v1_cos_angle) ** 0.5
        total_rotation = Orientation.add(Orientation.sub(v1, v0), np.r_[v0_v1_cos_angle[np.newaxis, :], v1_v0_sin_angle[np.newaxis, :]]) / sq_v0_v1

        Geometry.rotate_eq(eq0, config, total_rotation)
        Geometry.move_eq(eq0, config, Joint.get_solid_point(config, r0, True) - Joint.get_solid_point(config, r0))
        Geometry.move_eq(eq2, config, Joint.get_solid_point(config, r1) - Joint.get_solid_point(config, r1, True))

    @staticmethod
    def solve_ppr(config: Config, edges: tuple[OrientedJoint, ...], eqs: tuple[tuple[int, ...], ...], solution_index: int):
        r"""
                0
               / \
              P0  P1
             /     \
            1 - R2- 2
        """
        eq0, eq1, eq2 = eqs
        p0, p1, r2 = edges

        _angle10, _distance10 = Joint.get_point(config, p0)
        _angle20, _distance20 = Joint.get_point(config, p0, True)
        v1 = Orientation.add(Joint.get_solid_orientation(config, p0), Orientation.from_angle(_angle10))

        _angle11, _distance11 = Joint.get_point(config, p1)
        _angle21, _distance21 = Joint.get_point(config, p1, True)
        v2 = Orientation.add(Joint.get_solid_orientation(config, p1), Orientation.from_angle(_angle11))

        eq1_rotation = Orientation.sub(v1, Orientation.add(Joint.get_solid_orientation(config, p0, True), Orientation.from_angle(_angle20)))
        Geometry.rotate_eq(eq1, config, eq1_rotation)

        eq2_rotation = Orientation.sub(v2, Orientation.add(Joint.get_solid_orientation(config, p1, True), Orientation.from_angle(_angle21)))
        Geometry.rotate_eq(eq2, config, eq2_rotation)

        vec_1 = Joint.get_solid_position(config, p0) + (_distance10 - _distance20) * v1 + Joint.get_solid_point(config, r2) - Joint.get_solid_position(config, p0, True)
        vec_2 = Joint.get_solid_position(config, p1) + (_distance11 - _distance21) * v2 + Joint.get_solid_point(config, r2, True) - Joint.get_solid_position(config, p1, True)

        target_point = vec_1 + (Geometry.dot(vec_2 - vec_1, v2) / Geometry.det(v2, v1)) * Geometry.det_z(v1)
        Geometry.move_eq(eq1, config, target_point - Joint.get_solid_point(config, r2))
        Geometry.move_eq(eq2, config, target_point - Joint.get_solid_point(config, r2, True))
