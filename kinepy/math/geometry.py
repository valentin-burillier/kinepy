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

    @staticmethod
    def make_angle_continuous(angle):
        indices = ~np.isnan(angle)
        angle[indices & (indices.cumsum() > 1)] -= ((np.diff(angle[indices]) + np.pi) // (2 * np.pi)).cumsum() * (2 * np.pi)


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
        return np.sum(v1 * v2, axis=0)[np.newaxis, ...]

    @staticmethod
    def det(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
        return v1[np.newaxis, 0, :] * v2[np.newaxis, 1, :] - v1[np.newaxis, 1, :] * v2[np.newaxis, 0, :]

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
