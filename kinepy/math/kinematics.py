import numpy as np
from typing import Any


class Joint:
    @staticmethod
    def get_point(oriented_joint, direction=False):
        """
        Get the point according to the edge it represents; direction = False point is source; True point is destination

        shape (2, 1)
        """
        return getattr(oriented_joint[0], ('_p2', '_p1')[oriented_joint[1] ^ direction])[:, np.newaxis]

    @staticmethod
    def get_solid_point(solid_values, oriented_joint, direction=False):
        """
        Get the solid according to the edge it represents; direction = False point is source; True point is destination
        """
        _s_index = getattr(oriented_joint[0], ('_s2', '_s1')[oriented_joint[1] ^ direction])._index
        return Position.get(solid_values, _s_index) + Orientation.add(Orientation.get(solid_values, _s_index), Joint.get_point(oriented_joint, direction))

    @staticmethod
    def get_solid_local_point(solid_values, oriented_joint, direction=False):
        """
        Get the solid according to the edge it represents; direction = False point is source; True point is destination
        """
        _s_index = getattr(oriented_joint[0], ('_s2', '_s1')[oriented_joint[1] ^ direction])._index
        return Orientation.add(Orientation.get(solid_values, _s_index), Joint.get_point(oriented_joint, direction))


class Orientation:
    @staticmethod
    def index(solid: Any):
        return solid, slice(2, 4), Ellipsis

    @staticmethod
    def get(solid_values: np.ndarray, solid: Any) -> np.ndarray:
        return solid_values[Orientation.index(solid)]

    @staticmethod
    def add(x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        complex product
        Shapes (2, n) * (2, n) -> (2, n)
        """
        out = np.zeros(x.shape, dtype=x.dtype)
        out[0, ...] = x[0, ...] * y[0, ...] - x[1, ...] * y[1, ...]
        out[1, ...] = x[0, ...] * y[1, ...] + x[1, ...] * y[0, ...]
        return out

    @staticmethod
    def add_m(xm: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        complex product
        Shapes (m, 2, n) * (2, n) -> (m, 2, n)
        """
        out = np.zeros(xm.shape, dtype=xm.dtype)
        out[:, 0, ...] = xm[:, 0, ...] * y[np.newaxis, 0, ...] - xm[:, 1, ...] * y[np.newaxis, 1, ...]
        out[:, 1, ...] = xm[:, 0, ...] * y[np.newaxis, 1, ...] + xm[:, 1, ...] * y[np.newaxis, 0, ...]
        return out

    @staticmethod
    def sub(x: np.ndarray, y: np.ndarray) -> np.ndarray:
        out = np.zeros(x.shape, dtype=x.dtype)
        out[0, ...] = x[0, ...] * y[0, ...] + x[1, ...] * y[1, ...]
        out[1, ...] = x[1, ...] * y[0, ...] - x[0, ...] * y[1, ...]
        return out

    @staticmethod
    def from_angle(angle: np.ndarray) -> np.ndarray:
        out = np.zeros((2, *angle.shape), dtype=angle.dtype)
        out[0, ...] = np.cos(angle)
        out[1, ...] = np.sin(angle)
        return out


class Position:
    @staticmethod
    def index(solid: Any):
        return solid, slice(0, 2), Ellipsis

    @staticmethod
    def get(solid_values: np.ndarray, solid: Any) -> np.ndarray:
        return solid_values[Position.index(solid)]

    @staticmethod
    def local_point(solid_values: np.ndarray, solid: int, point: np.ndarray) -> np.ndarray:
        ori = Orientation.get(solid_values, solid)
        return Orientation.add(ori, point)

    @staticmethod
    def point(solid_values: np.ndarray, solid: int, point: np.ndarray) -> np.ndarray:
        return Position.local_point(solid_values, solid, point).__iadd__(Position.get(solid_values, solid))


class Geometry:
    @staticmethod
    def dot(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
        return np.sum(v1 * v2, axis=0)

    @staticmethod
    def inv_mag(vec: np.ndarray) -> np.ndarray:
        return Geometry.sq_mag(vec) ** -0.5

    @staticmethod
    def sq_mag(vec: np.ndarray) -> np.ndarray:
        return Geometry.dot(vec, vec)

    @staticmethod
    def move_eq(eq: tuple[int, ...], solid_values: np.ndarray, vec: np.ndarray):
        # shape: (m, 2, n) + (1, 2, n)
        Position.get(solid_values, eq).__iadd__(vec[np.newaxis, ...])

    @staticmethod
    def rotate_eq(eq: tuple[int, ...], solid_values: np.ndarray, rot: np.ndarray):
        solid_values[Position.index(eq)] = Orientation.add_m(Position.get(solid_values, eq), rot)
        solid_values[Orientation.index(eq)] = Orientation.add_m(Orientation.get(solid_values, eq), rot)

    @staticmethod
    def det_z(vec: np.ndarray) -> np.ndarray:
        out = np.zeros(vec.shape, vec.dtype)
        out[0, ...] = vec[1, ...]
        out[1, ...] = -vec[0, ...]
        return out


class JointValueComputation:
    @staticmethod
    def do_not_compute_value(solid_values: np.ndarray, s1: int, s2: int, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
        pass

    @staticmethod
    def compute_revolute_value(solid_values: np.ndarray, s1: int, s2: int, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
        s1_orientation = Orientation.get(solid_values, s1)
        s2_orientation = Orientation.get(solid_values, s2)

        diff = Orientation.sub(s1_orientation, s2_orientation)

        return np.arccos(diff[0, ...]) * np.sign(diff[1, ...])

    @staticmethod
    def compute_prismatic_value(solid_values: np.ndarray, s1: int, s2: int, p1: np.ndarray, p2: np.ndarray) -> np.ndarray:
        normal = Position.local_point(solid_values, s1, p1)
        s1_point = normal + Position.get(solid_values, s1)
        s2_point = Position.point(solid_values, s2, p2)

        return Geometry.dot(normal, s2_point - s1_point) * Geometry.inv_mag(normal)

    @staticmethod
    def do_not_compute_continuity(value: np.ndarray):
        pass

    @staticmethod
    def compute_revolute_continuity(value: np.ndarray):
        indices = ~np.isnan(value)
        value[indices & (indices.cumsum() > 1)] += ((np.diff(value[indices]) + np.pi) // (2 * np.pi)).cumsum() * (2 * np.pi)


class JointInput:
    @staticmethod
    def solve_revolute(solid_values: np.ndarray, value: np.ndarray, s1: int, s2: int, p1: np.ndarray, p2: np.ndarray, eq1: tuple[int, ...], eq2: tuple[int, ...]):
        s1_point = Position.point(solid_values, s1, p1)
        s2_point = Position.point(solid_values, s2, p2)

        s1_ori = Orientation.get(solid_values, s1)
        s2_ori = Orientation.get(solid_values, s2)

        rotation = Orientation.sub(s1_ori, s2_ori)
        total_rotation = Orientation.add(rotation, Orientation.from_angle(value))

        Geometry.move_eq(eq2, solid_values, -s2_point)
        Geometry.rotate_eq(eq2, solid_values, total_rotation)
        Geometry.move_eq(eq2, solid_values, s1_point)

    @staticmethod
    def solve_prismatic(solid_values: np.ndarray, value: np.ndarray, s1: int, s2: int, p1: np.ndarray, p2: np.ndarray, eq1: tuple[int, ...], eq2: tuple[int, ...]):
        s1_point = Position.local_point(solid_values, s1, p1)
        s2_point = Position.local_point(solid_values, s2, p2)

        total_rotation = Orientation.sub(s1_point, s2_point) * (Geometry.inv_mag(p1) * Geometry.inv_mag(p2))

        Geometry.move_eq(eq2, solid_values, -s2_point - Position.get(solid_values, s2))
        Geometry.rotate_eq(eq2, solid_values, total_rotation)
        Geometry.move_eq(eq2, solid_values, Position.get(solid_values, s1) + s1_point + Geometry.det_z(s1_point) * Geometry.inv_mag(p1) * value[np.newaxis, ...])


class System:
    @staticmethod
    def set_up(solid_values, joint_values, n_solid, n_joint, frame_count):
        solid_values.reshape((n_solid, 4, frame_count))
        # shapes: m, 4, n <-  1, 4, 1
        solid_values[:] = ((0.,), (0.,), (1.0,), (0.,)),

        joint_values.reshape((n_joint, frame_count))
        joint_values[:] = 0.0

    @staticmethod
    def clean_up(solid_values):
        Geometry.move_eq(tuple(range(solid_values.shape[1])), solid_values, -Position.get(solid_values, 0))
        Geometry.rotate_eq(tuple(range(solid_values.shape[1])), solid_values, Orientation.get(solid_values, 0) * np.array([[1], [-1]]))



class Graph:
    @staticmethod
    def solve_rrr(solid_values, edges, eqs, solution_index):
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
        v0 = Joint.get_solid_point(solid_values, r1) - Joint.get_solid_point(solid_values, r0)
        v1 = Joint.get_solid_point(solid_values, r2) - Joint.get_solid_point(solid_values, r0, True)
        v2 = Joint.get_solid_point(solid_values, r2, True) - Joint.get_solid_point(solid_values, r1, True)

        sq_a = Geometry.sq_mag(v0)
        sq_b = Geometry.sq_mag(v1)
        sq_c = Geometry.sq_mag(v2)
        inv_ab = (sq_a * sq_b) ** -0.5

        sign = (1, -1)[solution_index]
        cos_angle = 0.5 * (sq_a + sq_b - sq_c) * inv_ab
        sin_angle = sign * (1 - cos_angle * cos_angle) ** 0.5

        total_rotation = Orientation.add(Orientation.sub(v0, v1) * inv_ab, np.r_[cos_angle[np.newaxis, :], sin_angle[np.newaxis, :]])
        Geometry.rotate_eq(eq1, solid_values, total_rotation)
        Geometry.move_eq(eq1, solid_values, Joint.get_solid_point(solid_values, r0) - Joint.get_solid_point(solid_values, r0, True))

        _v1 = Joint.get_solid_point(solid_values, r2) - Joint.get_solid_point(solid_values, r1)
        eq2_rotation = Orientation.sub(v2, _v1) / sq_c
        Geometry.rotate_eq(eq2, solid_values, eq2_rotation)
        Geometry.move_eq(eq2, solid_values, Joint.get_solid_point(solid_values, r1) - Joint.get_solid_point(solid_values, r1, True))

    @staticmethod
    def solve_rrp(solid_values, edges, eqs, solution_index):
        r"""
                0
               / \
              R0  R1
             /     \
            1 - P2- 2
        """
        eq0, eq1, eq2 = eqs
        r0, r1, p2 = edges

        v0 = Joint.get_solid_point(solid_values, r0) - Joint.get_solid_point(solid_values, r1)

        v1 = Joint.get_solid_local_point(solid_values, p2)
        v2 = Joint.get_solid_local_point(solid_values, p2, True)
        eq2_rotation = Orientation.sub(v1, v2) / (Geometry.sq_mag(v1) * Geometry.sq_mag(v2)) ** 0.5
        Geometry.rotate_eq(eq2, solid_values, eq2_rotation)

        sq_v0_v1 = Geometry.sq_mag(v0) * Geometry.sq_mag(v1)

        sign = (1, -1)[solution_index]
        v1_v0_cos_angle = Geometry.dot(
            v1,
            Joint.get_solid_point(solid_values, p2) - Joint.get_solid_point(solid_values, r0, True) +
            Joint.get_solid_point(solid_values, r1, True) - Joint.get_solid_point(solid_values, p2, True)
        )
        v1_v0_sin_angle = sign * (sq_v0_v1 - v1_v0_cos_angle * v1_v0_cos_angle) ** 0.5
        total_rotation = Orientation.add(Orientation.sub(v1, v0), np.r_[v1_v0_cos_angle[np.newaxis, :], v1_v0_sin_angle[np.newaxis, :]]) / sq_v0_v1
        Geometry.rotate_eq(eq0, solid_values, total_rotation)
        Geometry.move_eq(eq0, solid_values, Joint.get_solid_point(solid_values, r0, True) - Joint.get_solid_point(solid_values, r0))
        Geometry.move_eq(eq2, solid_values, Joint.get_solid_point(solid_values, r1) - Joint.get_solid_point(solid_values, r1, True))
