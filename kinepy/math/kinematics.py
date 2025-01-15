import numpy as np
from typing import Any


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