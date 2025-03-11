import numpy as np
from typing import Any
from kinepy.objects.config import Config

OrientedJoint = tuple[int, bool]


class Joint:
    @staticmethod
    def get_point(config: Config, oriented_joint: OrientedJoint, direction=False):
        """
        Get the point according to the edge it represents; direction = False point is source; True point is destination

        shape (2, 1)
        """
        j_index, orientation = oriented_joint
        start_index = 2 - 2 * (orientation ^ direction)
        return config.joint_physics[j_index, start_index:start_index+2, np.newaxis]

    @staticmethod
    def get_solid_point(config: Config, oriented_joint: OrientedJoint, direction=False):
        """
        Get the solid according to the edge it represents; direction = False point is source; True point is destination
        """
        j_index, orientation = oriented_joint
        _s_index: int = config.joint_config[j_index, 2 - (orientation ^ direction)]
        return Position.get(config, _s_index) + Orientation.add(Orientation.get(config, _s_index), Joint.get_point(config, oriented_joint, direction))

    @staticmethod
    def get_solid_local_point(config: Config, oriented_joint: OrientedJoint, direction=False):
        """
        Get the solid according to the edge it represents; direction = False point is source; True point is destination
        """
        j_index, orientation = oriented_joint
        _s_index: int = config.joint_config[j_index, 2 - (orientation ^ direction)]
        return Orientation.add(Orientation.get(config, _s_index), Joint.get_point(config, oriented_joint, direction))

    @staticmethod
    def get_solid_orientation(config: Config, oriented_joint: OrientedJoint, direction=False):
        j_index, orientation = oriented_joint
        _s_index: int = config.joint_config[j_index, 2 - (orientation ^ direction)]
        return Orientation.get(config, _s_index)

    @staticmethod
    def get_solid_position(config: Config, oriented_joint: OrientedJoint, direction=False):
        j_index, orientation = oriented_joint
        _s_index: int = config.joint_config[j_index, 2 - (orientation ^ direction)]
        return Position.get(config, _s_index)


class Orientation:
    @staticmethod
    def index(solid: Any):
        return solid, slice(2, 4), Ellipsis

    @staticmethod
    def get(config: Config, solid: Any) -> np.ndarray:
        return config.results.solid_values[Orientation.index(solid)]

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
    def get(config: Config, solid: Any) -> np.ndarray:
        return config.results.solid_values[Position.index(solid)]

    @staticmethod
    def local_point(config: Config, solid: int, point: np.ndarray) -> np.ndarray:
        ori = Orientation.get(config, solid)
        return Orientation.add(ori, point)

    @staticmethod
    def point(config: Config, solid: int, point: np.ndarray) -> np.ndarray:
        return Position.local_point(config, solid, point) + Position.get(config, solid)


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
    def move_eq(eq: tuple[int, ...], config: Config, vec: np.ndarray):
        # shape: (m, 2, n) + (1, 2, n)
        config.results.solid_values[Position.index(eq)] += vec[np.newaxis, ...]

    @staticmethod
    def rotate_eq(eq: tuple[int, ...], config: Config, rot: np.ndarray):
        config.results.solid_values[Position.index(eq)] = Orientation.add_m(Position.get(config, eq), rot)
        config.results.solid_values[Orientation.index(eq)] = Orientation.add_m(Orientation.get(config, eq), rot)

    @staticmethod
    def det_z(vec: np.ndarray) -> np.ndarray:
        out = np.zeros(vec.shape, vec.dtype)
        out[0, ...] = vec[1, ...]
        out[1, ...] = -vec[0, ...]
        return out

    @staticmethod
    def z_det(vec: np.ndarray) -> np.ndarray:
        out = np.zeros(vec.shape, vec.dtype)
        out[0, ...] = -vec[1, ...]
        out[1, ...] = vec[0, ...]
        return out
