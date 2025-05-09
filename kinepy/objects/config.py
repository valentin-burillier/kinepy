from collections.abc import Iterable

import numpy as np
import enum


class Result:
    solid_values: np.ndarray
    solid_dynamics: np.ndarray

    joint_values: np.ndarray
    joint_dynamics: np.ndarray

    action_values: np.ndarray


class ActionMode(enum.Enum):
    NO_INDIRECTION, SOLID_G, JOINT_POINT = range(3)


class PropertyType(enum.Enum):
    CONFIG, PHYSICS, RESULT = range(3)

    @property
    def array_name(self):
        return f'{self.name.lower()}_array'
    
    @property
    def count_name(self):
        return f'_{self.name.lower()}_count'

class ArrayProperty(property):
    stop: int
    type_: PropertyType

    def __init__(self, type_: PropertyType, index: int | slice):
        arr = type_.array_name
        self.type_ = type_
        self.stop = index+1 if isinstance(index, int) else index.stop

        def getter(self) -> np.ndarray:
            return getattr(self, arr)[..., index]
        property.__init__(self, getter)
    
    @staticmethod
    def check(obj):
        for type_ in PropertyType:
            assert hasattr(obj, type_.array_name), f"[Internal] Wrong initialisation: {obj} has no attribute {type_.array_name}"

class MetaArray(type):
    def __new__(mcs, name: str, bases: tuple[type, ...], dict_: dict[str]):
        _array_counts = {type_: 0 for type_ in PropertyType}

        for obj in dict_.values():
            if not isinstance(obj, ArrayProperty):
                continue
            _array_counts[obj.type_] = max(_array_counts[obj.type_], obj.stop)

        for type_, size in _array_counts.items():
            dict_[type_.count_name] = size
        
        return type.__new__(mcs, name, bases, dict_)


class ConfigArray(metaclass=MetaArray):
    _config_count: int
    _physics_count: int
    _result_count: int

    def __init__(self):
        self.names = []

        self.config_array = np.array((0, self._config_count), int)
        self.physics_array = np.array((0, self._physics_count), float)
        self.result_array = np.array((0, 0, self._result_count), float)
        ArrayProperty.check(self)

    @property
    def count(self) -> int:
        return len(self.names)

    def allocate_results(self, frame_count):
        self.result_array.resize((self.count, frame_count, self._result_count))

    def add(self, names: list[str], config: np.ndarray, phy: np.ndarray):
        assert config.shape[1] == self._config_count, "Wrong config attributes shape"
        assert phy.shape[1] == self._physics_count, "Wrong physical attributes shape"
        assert len(names) == phy.shape[0] == phy.shape[1]

        self.names.extend(names)
        self.config_array = np.r_[self.config_array, config]
        self.physics_array = np.r_[self.physics_array, phy]
    
    def reserve(self, size) -> slice:
        result = slice(self.count, self.count + size)
        self.names.extend(('',) * size)

        for arr in self.config_array, self.physics_array:
            arr.resize((self.count, *arr.shape[1:]))

        return result

class SolidArray(ConfigArray):
    # Config attributes
    j3dof = ArrayProperty(PropertyType.CONFIG, 0)

    # Physics attributes
    mass = ArrayProperty(PropertyType.PHYSICS, 0)
    moment_of_inertia = ArrayProperty(PropertyType.PHYSICS, 1)
    g = ArrayProperty(PropertyType.PHYSICS, slice(2, 4))

    # Result attributes
    position = ArrayProperty(PropertyType.RESULT, slice(0, 2))
    orienation = ArrayProperty(PropertyType.RESULT, slice(2, 4))
    g_value = ArrayProperty(PropertyType.RESULT, slice(4, 6))
    newtons_2nd_law_force = ArrayProperty(PropertyType.RESULT, slice(6, 8))
    newtons_2nd_law_torque = ArrayProperty(PropertyType.RESULT, 8)


class JointArray(ConfigArray):
    # Config attributes
    type_ = ArrayProperty(PropertyType.CONFIG, 0)
    s1 = ArrayProperty(PropertyType.CONFIG, 1)
    s2 = ArrayProperty(PropertyType.CONFIG, 2)
    solids = ArrayProperty(PropertyType.CONFIG, slice(1, 3))

    # Physics attributes
    revolute_p1 = ArrayProperty(PropertyType.PHYSICS, slice(0, 2))
    revolute_p2 = ArrayProperty(PropertyType.PHYSICS, slice(2, 4))
    
    prismatic_angle1 = ArrayProperty(PropertyType.PHYSICS, 0)
    prismatic_distance1 = ArrayProperty(PropertyType.PHYSICS, 1)
    prismatic_angle2 = ArrayProperty(PropertyType.PHYSICS, 2)
    prismatic_distance2 = ArrayProperty(PropertyType.PHYSICS, 3)

    # Result attributes
    value = ArrayProperty(PropertyType.RESULT, 0)
    force = ArrayProperty(PropertyType.RESULT, slice(1, 3))
    torque = ArrayProperty(PropertyType.RESULT, 3)


class CompositeJointArray(ConfigArray):
    # Config attributes
    type_ = ArrayProperty(PropertyType.CONFIG, 0)
    ghost_solids = ArrayProperty(PropertyType.CONFIG, slice(1, 3))
    ghost_joints = ArrayProperty(PropertyType.CONFIG, slice(3, 6))


class RelationArray(ConfigArray):
    # Config attributes
    type_ = ArrayProperty(PropertyType.CONFIG, 0)
    j1 = ArrayProperty(PropertyType.CONFIG, 1)
    j2 = ArrayProperty(PropertyType.CONFIG, 2)
    joints = ArrayProperty(PropertyType.CONFIG, slice(1, 3))
    g1 = ArrayProperty(PropertyType.CONFIG, 3)
    g2 = ArrayProperty(PropertyType.CONFIG, 4)
    first_action = ArrayProperty(PropertyType.CONFIG, 5)

    # Physics attributes
    v0 = ArrayProperty(PropertyType.PHYSICS, 0)
    r = ArrayProperty(PropertyType.PHYSICS, 1)
    gear_pressure_angle = ArrayProperty(PropertyType.PHYSICS, 2)

    belt_r1 = ArrayProperty(PropertyType.PHYSICS, 1)
    belt_r2 = ArrayProperty(PropertyType.PHYSICS, 2)
    belt_t0 = ArrayProperty(PropertyType.PHYSICS, 3)


class ActionArray(ConfigArray):
    # Config attributes
    type_ = ArrayProperty(PropertyType.CONFIG, 0)
    object_reference = ArrayProperty(PropertyType.CONFIG, 1) # object might have to move because of solids added after
    custom_solid = ArrayProperty(PropertyType.CONFIG, 2)

    # Physics attributes
    custom_point = ArrayProperty(PropertyType.PHYSICS, slice(0, 2))

    # Result attributes
    force = ArrayProperty(PropertyType.RESULT, slice(0, 2))
    torque = ArrayProperty(PropertyType.RESULT, 2)
    application_point = ArrayProperty(PropertyType.RESULT, slice(3, 5))

class InteractionArray(ConfigArray):
    # Config attributes
    type_ = ArrayProperty(PropertyType.CONFIG, 0)
    first_action = ArrayProperty(PropertyType.CONFIG, 1)
    twisting_spring_revolute = ArrayProperty(PropertyType.CONFIG, 2)
    linear_spring_s1 = ArrayProperty(PropertyType.CONFIG, 2)
    linear_spring_s2 = ArrayProperty(PropertyType.CONFIG, 3)

    # Physics attributes
    g_fields = ArrayProperty(PropertyType.PHYSICS, slice(0, 2))
    spring_stiffness = ArrayProperty(PropertyType.PHYSICS, 1)
    spring_equilibrium_position = ArrayProperty(PropertyType.PHYSICS, 2)
    linear_spring_p1 = ArrayProperty(PropertyType.PHYSICS, slice(3, 5))
    linear_spring_p2 = ArrayProperty(PropertyType.PHYSICS, slice(5, 7))

    

class ConfigState(enum.Enum):
    NO_READ_ALLOWED, STRATEGY_OK, ALLOCATED_RESOURCES, KINEMATICS_OK, DYNAMICS_OK = range(5)

    def __ge__(self, other):
        return self.value >= other.value

    def __le__(self, other):
        return self.value <= other.value

    def __gt__(self, other):
        return self.value > other.value


class NewConfig:
    def __init__(self):
        self.state = ConfigState.NO_READ_ALLOWED
        self.frame_time = 0.0
        self.frame_count = 0

        # Data
        self.solids = SolidArray()
        self.joints = JointArray()
        self.composite_joints = CompositeJointArray()
        self.relations = RelationArray()
        self.actions = ActionArray()
        self.interactions = InteractionArray()

        # External configuration
        self.piloted_joints = np.zeros((0,), int)
        self.working_joints = np.zeros((0,), int)

        # Strategy states
        self.joint_states = []
        self.final_joint_states = []
        self.kinematics_strategy = []
        self.dynamics_strategy = []

    def invalidate_config(self):
        self.state = ConfigState.NO_READ_ALLOWED

    def invalidate_kinematics(self):
        self.state = min(ConfigState.ALLOCATED_RESOURCES, self.state)

    def invalidate_dynamics(self):
        self.state = min(ConfigState.KINEMATICS_OK, self.state)

    def allocate_resources(self, frame_count):
        self.state = ConfigState.ALLOCATED_RESOURCES
        for arr in self.solids, self.joints, self.composite_joints, self.relations, self.actions:
            arr.allocate_results(frame_count)

class Config:

    # TODO: this constants garbage has to go, i have been mistaken more than once

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


class KpArray(np.ndarray):
    """
    Specialized ndarray that offers time derivatives
    """
    _config: Config
    _time_axis: int

    def __array_finalize__(self, obj, /):
        self._config = getattr(obj, '_config', None)
        self._time_axis = getattr(obj, '_time_axis', 0)

    def _configure(self, _config: Config, _time_axis: int):
        self._config = _config
        self._time_axis = _time_axis
        return self

    def _inherit(self, arr: np.ndarray):
        return arr.view(self.__class__)._configure(self._config, self._time_axis)

    def derivative(self):
        return self._inherit(0.5 * (np.diff(self, axis=self._time_axis, prepend=float('NaN')) + np.diff(self, axis=self._time_axis, append=float('NaN'))) / self._config.frame_time)
    
    def second_derivative(self):
        return self._inherit(np.diff(self, 2, axis=self._time_axis, prepend=float('NaN'), append=float('NaN')) / self._config.frame_time / self._config.frame_time)


def disable_set(prop: property) -> property:
    def getter(self):
        return prop.__get__(self).copy()
    return property(getter)


def mirror_other(prop: property, other: property) -> property:
    def setter(self, value):
        prop.fset(self, value)
        other.fset(self, value)

    return property(prop.fget, setter)
