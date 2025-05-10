import numpy as np
import kinepy.objects.config as cfg
import enum


type OrientedJoint = tuple[int, bool]


class Joint:
    class Direction(enum.Enum):
        SOURCE, TARGET = range(2)

    @staticmethod
    def get_solid(config: cfg.Config, oriented_joint: OrientedJoint, direction: Direction = Direction.SOURCE) -> int:
        """
        Get s1 or s2 depending on joint orientation and the desired direction
        """
        j_index, orientation = oriented_joint
        return config.joints.solids[j_index, orientation ^ direction.value]

    @staticmethod
    def get_point(config: cfg.Config, oriented_joint: OrientedJoint, direction: Direction = Direction.SOURCE):
        """
        Get p1 or p2 depending on joint orientation and the desired direction
        """
        j_index, orientation = oriented_joint
        point_slice = config.joints.revolute_p1, config.joints.revolute_p2
        return point_slice[orientation ^ direction.value][j_index]

    @staticmethod
    def get_solid_point(config: cfg.Config, oriented_joint: OrientedJoint, direction: Direction = Direction.SOURCE):
        _s_index: int = Joint.get_solid(config, oriented_joint, direction)
        return Position.point(config, _s_index, Joint.get_point(config, oriented_joint, direction))

    @staticmethod
    def get_solid_vector(config: cfg.Config, oriented_joint: OrientedJoint, direction: Direction = Direction.SOURCE):
        _s_index: int = Joint.get_solid(config, oriented_joint, direction)
        return Position.vector(config, _s_index, Joint.get_point(config, oriented_joint, direction))

    @staticmethod
    def get_solid_orientation(config: cfg.Config, oriented_joint: OrientedJoint, direction: Direction = Direction.SOURCE):
        _s_index: int = Joint.get_solid(config, oriented_joint, direction)
        return config.solids.orientation[_s_index]

    @staticmethod
    def get_solid_position(config: cfg.Config, oriented_joint: OrientedJoint, direction: Direction = Direction.SOURCE):
        _s_index: int = Joint.get_solid(config, oriented_joint, direction)
        return config.solids.position[_s_index]

    @staticmethod
    def get_revolute_application_point(config: cfg.Config, joint: OrientedJoint):
        point = config.joints.p1[joint[0]]
        return Position.point(config, config.joints.s1[joint[0]], point)

    @staticmethod
    def get_prismatic_application_point(config: cfg.Config, joint: OrientedJoint):
        angle, dist = config.joints.revolute_p1[joint[0]]
        return Position.point(config, config.joints.s1[joint[0]], dist * Orientation.from_angle(angle + np.pi * 0.5))

    @staticmethod
    def get_prismatic_normal(config: cfg.Config, joint: OrientedJoint):
        angle, dist = config.joints.revolute_p1[joint[0]]
        return Position.vector(config, config.joints.s1[joint[0]], Orientation.from_angle(angle + np.pi * 0.5))


class Orientation:
    @staticmethod
    def add(x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """
        complex product
        Shapes (..., 2, n) * (..., 2, n) -> (..., 2, n)
        """
        out = np.zeros(x.shape, dtype=x.dtype)
        out[..., 0] = x[..., 0] * y[..., 0] - x[..., 1] * y[..., 1]
        out[..., 1] = x[..., 0] * y[..., 1] + x[..., 1] * y[..., 0]
        return out

    @staticmethod
    def sub(x: np.ndarray, y: np.ndarray) -> np.ndarray:
        out = np.zeros(x.shape, dtype=x.dtype)
        out[..., 0] = x[..., 0] * y[..., 0] + x[..., 1] * y[..., 1]
        out[..., 1] = x[..., 1] * y[..., 0] - x[..., 0] * y[..., 1]
        return out

    @staticmethod
    def from_angle(angle: np.ndarray) -> np.ndarray:
        out = np.zeros((*angle.shape, 2), dtype=angle.dtype)
        out[..., 0] = np.cos(angle)
        out[..., 1] = np.sin(angle)
        return out

    @staticmethod
    def _make_angle_continuous(angle: np.ndarray):
        indices = ~np.isnan(angle)
        angle[indices & (indices.cumsum() > 1)] -= ((np.diff(angle[indices]) + np.pi) // (2 * np.pi)).cumsum() * (2 * np.pi)
        return angle

    @staticmethod
    def make_angle_continuous(angle: np.ndarray):
        return np.apply_along_axis(Orientation._make_angle_continuous, -1, angle)


class Position:
    @staticmethod
    def vector(config: cfg.Config, solid: int | slice, point: np.ndarray) -> np.ndarray:
        return Orientation.add(config.solids.orientation[solid], point)

    @staticmethod
    def point(config: cfg.Config, solid: int | slice, point: np.ndarray) -> np.ndarray:
        return Position.vector(config, solid, point) + config.solids.position[solid]


class Geometry:
    @staticmethod
    def dot(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
        return np.sum(v1 * v2, axis=-1)[..., np.newaxis]

    @staticmethod
    def det(v1: np.ndarray, v2: np.ndarray) -> np.ndarray:
        return np.cross(v1, v2, axis=-1)[..., np.newaxis] # noqa: False positive, unreachable code with numpy.cross

    @staticmethod
    def inv_mag(vec: np.ndarray) -> np.ndarray:
        return Geometry.sq_mag(vec) ** -0.5

    @staticmethod
    def sq_mag(vec: np.ndarray) -> np.ndarray:
        return Geometry.dot(vec, vec)

    @staticmethod
    def mag(vec: np.ndarray) -> np.ndarray:
        return Geometry.dot(vec, vec) ** 0.5

    @staticmethod
    def move_eq(eq: tuple[int, ...], config: cfg.Config, vec: np.ndarray):
        config.solids.position[eq, ...] += vec

    @staticmethod
    def rotate_eq(eq: tuple[int, ...], config: cfg.Config, rot: np.ndarray):
        config.solids.position[eq, ...] = Orientation.add(config.solids.position[eq, ...], rot)
        config.solids.orientation[eq, ...] = Orientation.add(config.solids.orientation[eq, ...], rot)

    @staticmethod
    def det_z(vec: np.ndarray) -> np.ndarray:
        out = np.zeros(vec.shape, vec.dtype)
        out[..., 0] = vec[..., 1]
        out[..., 1] = -vec[..., 0]
        return out

    @staticmethod
    def z_det(vec: np.ndarray) -> np.ndarray:
        out = np.zeros(vec.shape, vec.dtype)
        out[..., 0] = -vec[..., 1]
        out[..., 1] = vec[..., 0]
        return out
