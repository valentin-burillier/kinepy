from collections.abc import Iterable

import numpy as np
import enum


class Result:
    solid_values: np.ndarray
    solid_dynamics: np.ndarray

    joint_values: np.ndarray
    joint_dynamics: np.ndarray

    action_values: np.ndarray


class ConfigState(enum.Enum):
    NO_READ_ALLOWED, STRATEGY_OK, ALLOCATED_RESOURCES, KINEMATICS_OK, DYNAMICS_OK = range(5)

    def __ge__(self, other):
        return self.value >= other.value

    def __le__(self, other):
        return self.value <= other.value

    def __gt__(self, other):
        return self.value > other.value


class ActionMode(enum.Enum):
    NO_INDIRECTION, SOLID_G, JOINT_POINT = range(3)


class Config:
    SOLID_3DOF = 0
    SOLID_MASS = 0
    SOLID_MOMENT_OF_INERTIA = 1
    SOLID_CFG_G = slice(2, 4)

    SOLID_DYN_FORCE = slice(0, 2)
    SOLID_DYN_G = slice(2, 4)
    SOLID_DYN_TORQUE = 4

    JOINT_TYPE = 0
    JOINT_S1 = 1
    JOINT_S2 = 2
    JOINT_SOLIDS = slice(1, 3)

    JOINT_P1 = slice(0, 2)
    JOINT_P2 = slice(2, 4)
    JOINT_A1, JOINT_D1, JOINT_A2, JOINT_D2 = range(4)

    COMPOSITE_TYPE = 0
    COMPOSITE_JOINTS = slice(1, 4)

    JOINT_DYN_FORCE = slice(0, 2)
    JOINT_DYN_TORQUE = 2

    RELATION_TYPE = 0
    RELATION_JOINTS = slice(1, 3)
    RELATION_J1 = 1
    RELATION_J2 = 2
    RELATION_TYPE_JOINTS = slice(0, 3)
    RELATION_G1 = 3
    RELATION_G2 = 4

    RELATION_V0 = 0
    RELATION_R = 1
    RELATION_PRESSURE_ANGLE = 2
    RELATION_R1 = 1
    RELATION_R2 = 2
    RELATION_T0 = 3

    ACTION_SOLID = 0
    ACTION_MODE = 1
    ACTION_INDIRECTION = 2

    ACTION_DYN_FORCE = slice(0, 2)
    ACTION_DYN_TORQUE = 2

    def __init__(self):
        # name
        self.solid_names = ['Ground']
        # 3dof_index
        self.solid_config = np.array(((-1,),), int)
        # mass, moment_of_inertia, g.x, g.y
        self.solid_physics = np.zeros((1, 4), float)

        self.joint_names = []
        # _type, s1, s2
        self.joint_config = np.zeros((0, 3), int)

        # p1.x, p1.y, p2.x, p2.y
        # angle1, distance1, angle2, distance2
        self.joint_physics = np.zeros((0, 4), float)

        # _type, ghost_j1, ghost_j2, ghost_j3, ghost_s1, ghost_s2
        self.composite_joint_config = np.zeros((0, 6), int)

        self.piloted_joints = np.zeros((0,), int)
        self.working_joints = np.zeros((0,), int)

        # _type, j1, j2, g1, g2
        self.relation_config = np.zeros((0, 5), int)
        # v0, r, _, _ (distant / effortless)
        # v0, r, pressure_angle, _ (gear pair / gear rack)
        # v0, r1, r2, t0 (belt)
        self.relation_physics = np.zeros((0, 4), float)

        # solid, indirection type, indirection index
        self.action_config = np.zeros((0, 3), int)
        # ap.x, ap.y
        self.action_physics = np.zeros((0, 2), float)

        self.joint_states = []
        self.final_joint_states = []

        self.frame_time = 0.0

        self.results = Result()

        self.state = ConfigState.NO_READ_ALLOWED

    def invalidate_config(self):
        self.state = ConfigState.NO_READ_ALLOWED

    def invalidate_dynamics(self):
        self.state = min(ConfigState.KINEMATICS_OK, self.state)

    def invalidate_kinematics(self):
        self.state = min(ConfigState.ALLOCATED_RESOURCES, self.state)

    def allocate_results(self, frame_count, frame_time=0.0):
        self.state = ConfigState.ALLOCATED_RESOURCES
        # x, y, cos(a), sin(a)
        self.results.solid_values = np.zeros((self.solid_physics.shape[0], frame_count, 4), float)
        self.results.solid_values[..., 2] = 1.

        self.results.joint_values = np.zeros((self.joint_config.shape[0], frame_count), float)

        self.allocated_results_dyn(frame_count, frame_time)

    def allocated_results_dyn(self, frame_count, frame_time):
        self.frame_time = frame_time
        # force x, force y, gx, gy, torque(g)
        self.results.solid_dynamics = np.zeros((self.solid_physics.shape[0], frame_count, 5))
        # force x, force y, torque
        self.results.joint_dynamics = np.zeros((self.joint_config.shape[0], frame_count, 3))

        # force.x, force.y, torque
        self.results.action_values = np.zeros((self.action_config.shape[0], frame_count, 3))

    def add_solids(self, names: list[str], physics: np.ndarray):
        self.invalidate_config()
        self.solid_names.extend(names)
        self.solid_config = np.r_[self.solid_config, ((-1,),) * physics.shape[0]]
        self.solid_physics = np.r_[self.solid_physics, physics]

    def add_joints(self, names: Iterable[str], config: np.ndarray, physics: np.ndarray):
        self.invalidate_config()
        self.joint_names.extend(names)
        self.joint_config = np.r_[self.joint_config, config]
        self.joint_physics = np.r_[self.joint_physics, physics]

    def add_composite(self, _type: int, ghost_j_indices, ghost_s_indices) -> int:
        index = self.composite_joint_config.shape[0]

        _gj = [-1] * 3
        _gj[:len(ghost_j_indices)] = ghost_j_indices

        _gs = [0] * 2
        _gs[:len(ghost_s_indices)] = ghost_s_indices

        self.composite_joint_config = np.r_[self.composite_joint_config, [[_type] + _gj + _gs]]
        return index

    def get_composite_solids(self, composite_index) -> tuple[int, int]:
        joints = self.composite_joint_config[composite_index, Config.COMPOSITE_JOINTS]
        while joints[-1] == -1:
            joints = joints[:-1]
        return int(self.joint_config[joints[0], Config.JOINT_S1]), int(self.joint_config[joints[-1], Config.JOINT_S2])

    def add_relations(self, config: np.ndarray, physics: np.ndarray):
        self.invalidate_config()
        self.relation_config = np.r_[self.relation_config, config]
        self.relation_physics = np.r_[self.relation_physics, physics]

    def add_actions(self, config: np.ndarray, physics: np.ndarray):
        self.invalidate_config()
        self.action_config = np.r_[self.action_config, config]
        self.action_physics = np.r_[self.action_physics, physics]


class ConfigView:
    __slots__ = '_config', '_index'

    def __init__(self, config: Config, index: int):
        self._config: Config = config
        self._index: int = index

    def check_against(self, config: Config) -> bool:
        if self._config is not config:
            return False
        if self._index >= self._config_arr().shape[0]:
            return False
        return True

    def _config_arr(self) -> np.ndarray[int]:
        pass

    def _names(self) -> list[str]:
        pass

    def _physics(self) -> np.ndarray[float]:
        pass

    @classmethod
    def _physics_view(cls, index: int | slice, phy) -> property:
        def getter(self: cls) -> phy:
            return self._physics()[self._index, index]

        def setter(self: cls, value: phy):
            self._physics()[self._index, index] = value

        return property(getter, setter)

    @classmethod
    def _config_view(cls, index: int | slice) -> property:
        def getter(self: cls) -> int:
            return int(self._config_arr()[self._index, index])

        def setter(self: cls, value):
            self._config_arr()[self._index, index] = value

        return property(getter, setter)

    @classmethod
    def _name(cls) -> property:
        def getter(self: cls) -> str:
            return self._names()[self._index]

        return property(getter)


class ReadOnlyArray(np.ndarray):
    def __array_finalize__(self, obj, /):
        self.flags.writeable = False


def disable_set(prop: property) -> property:
    def getter(self):
        return prop.__get__(self).view(ReadOnlyArray)
    return property(getter)


def mirror_other(prop: property, other: property) -> property:
    def setter(self, value):
        prop.fset(self, value)
        other.fset(self, value)

    return property(prop.fget, setter)
