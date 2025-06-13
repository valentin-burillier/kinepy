import typing
import numpy as np
import enum
import functools
import kinepy.math.calculus as cal
import kinepy.exceptions as ex


class IntEnum(enum.Enum):
    """
    Intermediate Enum that allows implicit int casts to use in numpy.ndarray
    """

    def __int__(self) -> int:
        return self.value

    def __gt__(self, other):
        return int(self) > other


class KpProperty(property):
    """
    Represents attributes of most kinepy objects as slices of bigger numpy arrays

    Attributes:
        stop(int):  upper bound of slice it occupies on the reference array
        type_(KpProperty.Type):
    """

    stop: int

    class Type(IntEnum):
        CONFIG, PHYSICS, RESULT_KINEMATICS, RESULT_DYNAMICS = range(4)

        @property
        def array_name(self):
            """
            Instance array attribute to read values from
            """
            return f'{self.name.lower().split('_')[0]}_array'

        @property
        def count_name(self):
            """
            Class attribute to update when counting attributes
            """
            return f'_{self.name.lower().split('_')[0]}_count'

        def __call__(self, index: int | slice):
            """
            Create a property of this type occupying index in the target Array
            """
            return KpProperty(self, index)

    def __init__(self, type_: Type, index: int | slice):
        arr = type_.array_name
        self.type_: KpProperty.Type = type_
        self.stop = index+1 if isinstance(index, int) else index.stop

        def getter(_self) -> np.ndarray:
            return getattr(_self, arr)[..., index]
        property.__init__(self, getter)

    @staticmethod
    def check(obj: "ConfigArray"):
        """
        Helper to verify that an object owning KpProperties is properly initialized

        :param obj: KpProperty owner
        :type obj: ConfigArray
        """ 
        for type_ in KpProperty.Type:
            assert not getattr(obj, type_.count_name) or hasattr(obj, type_.array_name), f"[Internal] Wrong initialisation: {obj} has no attribute {type_.array_name}"

    def __call__(self) -> property:
        """
        Create the corresponding property on ConfigView classes
        """
        return ConfigView.__forward__(self)


class MetaArray(type):
    """
    Count how many attributes are needed per sub-array (config, physics, result) through the use of `KpProperty`
    """

    def __new__(mcs, name: str, bases: tuple[type, ...], dict_: dict[str, typing.Any]):
        # setting counters for each attribute type
        _array_counts = {type_.count_name: 0 for type_ in KpProperty.Type}

        for obj in dict_.values():
            if not isinstance(obj, KpProperty):
                continue
            # updating counters
            _array_counts[obj.type_.count_name] = max(_array_counts[obj.type_.count_name], obj.stop)
        
        # sharing counters
        dict_  |= _array_counts

        # no result attributes -> no result allocation
        def _allocate_results(self, frame_count):
            self.result_array.resize((self.count, frame_count, self._result_count))
        dict_['allocate_results'] = _allocate_results if _array_counts[KpProperty.Type.RESULT_DYNAMICS.count_name] else lambda self, frame_count: None

        return type.__new__(mcs, name, bases, dict_)


class ConfigArray(metaclass=MetaArray):
    _config_count: int
    _physics_count: int
    _result_count: int

    def __init__(self):
        # names of each object
        self.names = []
        
        # arrays that are actually used
        self.__arrays = []

        if self._config_count:
            self.config_array = np.zeros((0, self._config_count), int)
            self.__arrays.append(self.config_array)
        if self._physics_count:
            self.physics_array = np.zeros((0, self._physics_count), float)
            self.__arrays.append(self.physics_array)
        if self._result_count:
            self.result_array = np.zeros((0, 0, self._result_count), float)
        KpProperty.check(self)

    @property
    def count(self) -> int:
        """
        Number of objects in the array
        """
        return len(self.names)

    def allocate_results(self, frame_count: int):
        """
        Allocate result_array, if neccessary, to support `frame_count` frames
        
        :param frame_count: number of frames in the simulation
        :type frame_count: int
        """

    def reserve(self, obj_cnt: int) -> slice:
        """
        Reserves `obj_cnt` places in the array gives back the slice occupied be the new objects

        :param obj_cnt: number of objects to allocate for
        :type obj_cnt: int
        :return: region occupied by the new objects
        :rtype: slice
        """

        result = slice(self.count, self.count + obj_cnt)
        self.names.extend(('',) * obj_cnt)
        for array in self.__arrays:
            array.resize((self.count, *array.shape[1:]), refcheck=False)
            array[result] = 0
        return result


class Solids(ConfigArray):
    # Config attributes
    j3dof = KpProperty.Type.CONFIG(0)
    is_ghost = KpProperty.Type.CONFIG(1)

    # Physics attributes
    mass = KpProperty.Type.PHYSICS(0)
    moment_of_inertia = KpProperty.Type.PHYSICS(1)
    g = KpProperty.Type.PHYSICS(slice(2, 4))

    # Result attributes
    position = KpProperty.Type.RESULT_KINEMATICS(slice(0, 2))
    orientation = KpProperty.Type.RESULT_KINEMATICS(slice(2, 4))
    g_value = KpProperty.Type.RESULT_DYNAMICS(slice(4, 6))
    newtons_2nd_law_force = KpProperty.Type.RESULT_DYNAMICS(slice(6, 8))
    newtons_2nd_law_torque = KpProperty.Type.RESULT_DYNAMICS(8)


class Joints(ConfigArray):
    class Type(IntEnum):
        EMPTY, REVOLUTE, PRISMATIC = range(3)

        PRIMITIVE_SEPARATOR = 4

        GHOST_ANGLE = 5
        X, Y, J_AXLE = 6, 10, 14

        def primitive(self):
            """
            Removes extra information giving only the primitive type

            :return: `REVOLUTE` or `PRISMATIC`
            :rtype: Type
            """
            return self.__class__(self.value & 3)

    # Config attributes
    type_ = KpProperty.Type.CONFIG(0)
    s1 = KpProperty.Type.CONFIG(1)
    s2 = KpProperty.Type.CONFIG(2)
    solids = KpProperty.Type.CONFIG(slice(1, 3))
    state = KpProperty.Type.CONFIG(3)

    # Physics attributes 
    revolute_p1 = KpProperty.Type.PHYSICS(slice(0, 2))
    revolute_p2 = KpProperty.Type.PHYSICS(slice(2, 4))

    prismatic_angle1 = KpProperty.Type.PHYSICS(0)
    prismatic_distance1 = KpProperty.Type.PHYSICS(1)
    prismatic_angle2 = KpProperty.Type.PHYSICS(2)
    prismatic_distance2 = KpProperty.Type.PHYSICS(3)

    # Result attributes
    value = KpProperty.Type.RESULT_KINEMATICS(0)
    force = KpProperty.Type.RESULT_DYNAMICS(slice(1, 3))
    torque = KpProperty.Type.RESULT_DYNAMICS(3)


class Composite(ConfigArray):
    class Type(IntEnum):
        PIN_SLOT, TRANSLATION, J3DOF = range(3)

        @property
        def ghost_count(self) -> int:
            """
            Number of ghost solids used by this `Type` of `CompositeJoint`
            """ 
            return {
                Composite.Type.PIN_SLOT: 1,
                Composite.Type.TRANSLATION: 1,
                Composite.Type.J3DOF: 2
            }[self]

    # Config attributes
    type_ = KpProperty.Type.CONFIG(0)
    first_ghost_solid = KpProperty.Type.CONFIG(1)
    first_ghost_joint = KpProperty.Type.CONFIG(2)


class Relations(ConfigArray):
    class Type(IntEnum):
        GEAR_PAIR, GEAR_RACK, BELT, DISTANT, EFFORTLESS = range(5)

        @property
        def is_gear(self) -> bool:
            return self in (Relations.Type.GEAR_PAIR, Relations.Type.GEAR_RACK, Relations.Type.BELT)

    # Config attributes
    type_ = KpProperty.Type.CONFIG(0)
    j1 = KpProperty.Type.CONFIG(1)
    j2 = KpProperty.Type.CONFIG(2)
    joints = KpProperty.Type.CONFIG(slice(1, 3))
    g1 = KpProperty.Type.CONFIG(3)
    g2 = KpProperty.Type.CONFIG(4)
    first_action = KpProperty.Type.CONFIG(5)

    # Physics attributes
    v0 = KpProperty.Type.PHYSICS(0)
    r = KpProperty.Type.PHYSICS(1)
    gear_pressure_angle = KpProperty.Type.PHYSICS(2)

    belt_r1 = KpProperty.Type.PHYSICS(1)
    belt_r2 = KpProperty.Type.PHYSICS(2)
    belt_t0 = KpProperty.Type.PHYSICS(3)


class Actions(ConfigArray):
    class Type(IntEnum):
        USER, INTERNAL_INTERACTION, INTERNAL_OUTPUT = range(3)

    # Config attributes
    type_ = KpProperty.Type.CONFIG(0)
    solid = KpProperty.Type.CONFIG(1)

    # Physics attributes
    user_point = KpProperty.Type.PHYSICS(slice(0, 2))

    # Result attributes
    force = KpProperty.Type.RESULT_DYNAMICS(slice(0, 2))
    torque = KpProperty.Type.RESULT_DYNAMICS(2)
    application_point = KpProperty.Type.RESULT_DYNAMICS(slice(3, 5))


class Interactions(ConfigArray):
    class Type(IntEnum):
        GRAVITY, INERTIA, LINEAR_SPRING, TWISTING_SPRING = range(4)

    # Config attributes
    type_ = KpProperty.Type.CONFIG(0)
    first_action = KpProperty.Type.CONFIG(1)
    twisting_spring_revolute = KpProperty.Type.CONFIG(2)
    linear_spring_s1 = KpProperty.Type.CONFIG(2)
    linear_spring_s2 = KpProperty.Type.CONFIG(3)

    # Physics attributes
    g_fields = KpProperty.Type.PHYSICS(slice(0, 2))
    spring_stiffness = KpProperty.Type.PHYSICS(1)
    spring_equilibrium_position = KpProperty.Type.PHYSICS(2)
    linear_spring_p1 = KpProperty.Type.PHYSICS(slice(3, 5))
    linear_spring_p2 = KpProperty.Type.PHYSICS(slice(5, 7))


class ResolutionStep(ConfigArray):
    class Type(IntEnum):
        JOINT_INPUT, JOINT_COMPUTE, RELATION = range(3)
        __GRAPHS__ = gRRR, gRRP, gPPR, g3RR, g2RR_PP, g3PR, g2PR_RR, gPP_RR_PR, g2RR_PR, gPP_PR_RP, g2RP_PP, gRR_PR_RP, g2RP_PR = range(3, 16)

        @classmethod
        def graphs(cls):
            yield from (cls(g) for g in cls.__GRAPHS__)

    class ComputeFlags:
        SOLVED_BIT = 1 << 0

        # joint value is certified to be computed for joints that are solved: -by inputs; -by relations.
        # relations may have to compute the joint values for those that are not available yet, otherwise these computations are not necessary and will depend on user queries
        COMPUTED_BIT = 1 << 1

        # joint value is certified to be continuous for all prismatic joints, and for revolute joints that are solved: -before inputs; -by inputs; -by relations.
        # when driven by a revolute joint, relations may have to compute the continuous version of its angle if not already available
        CONTINUOUS_BIT = 1 << 2

        RELATION_READY = CONTINUOUS_BIT | COMPUTED_BIT
        READY_FOR_USER = SOLVED_BIT | COMPUTED_BIT | CONTINUOUS_BIT

        @classmethod
        def relation_ready(cls, value: int) -> bool:
            return bool(value & cls.COMPUTED_BIT and value & cls.CONTINUOUS_BIT)

        @classmethod
        def solved_joint(cls, type_: int, value_is_computed: bool, certain_continuity: bool):
            return cls.SOLVED_BIT | (certain_continuity or Joints.Type(type_).primitive() == Joints.Type.PRISMATIC) * cls.CONTINUOUS_BIT | value_is_computed * cls.COMPUTED_BIT

    # Soft Config attributes
    type_ = KpProperty.Type.CONFIG(0)
    eqs = KpProperty.Type.CONFIG(1)
    
    graph_edges = KpProperty.Type.CONFIG(2)
    graph_solution_index = KpProperty.Type.CONFIG(3)
    
    joint = KpProperty.Type.CONFIG(2)
    joint_state = KpProperty.Type.CONFIG(3)

    relation = KpProperty.Type.CONFIG(2)
    relation_direction = KpProperty.Type.CONFIG(3)

class Eqs:
    """
    Serialised arrays of Equivalence classes decribing used in resolution steps
    
    Format:
    ```
    | GROUP_X_OFFSET               GroupX                              | GROUP_Y_OFFSET               GroupY
    | |                                                                | |
    |  offset0, offset1, ..., offsetn, ...eq1, ...eq2, ..., ...eqn     |  ...
    | |                              |       |       |            |    | |
    | 0                           offset0 offset1 offset2      offsetn | 0
    ```
    """

    def __init__(self):
        self.eqs = np.zeros((0,), int)

    def add(self, eqs) -> int:
        index = self.eqs.size
        lengths = 0, *map(len, eqs)
        offsets = np.cumsum(lengths) + len(lengths)
        self.eqs.resize((self.eqs.size + offsets[-1]))
        self.eqs[index:][:offsets[0]] = offsets
        self.eqs[index+offsets[0]:] = sum(eqs, ())
        return index

    def _get(self, offset):
        begin = offset+self.eqs[offset]
        for i in range(offset+1, offset+self.eqs[offset]):
            end = offset + self.eqs[i]
            yield self.eqs[begin:end]
            begin = end

    def get(self, offset):
        return tuple(self._get(offset))


class ConfigState(enum.Enum):
    NO_READ_ALLOWED, STRATEGY_OK, ALLOCATED_RESOURCES, KINEMATICS_OK, DYNAMICS_OK = range(5)

    def __ge__(self, other):
        return self.value >= other.value

    def __le__(self, other):
        return self.value <= other.value

    def __gt__(self, other):
        return self.value > other.value


class Config:
    def __init__(self):
        self.state = ConfigState.NO_READ_ALLOWED
        self.frame_time = 0.0
        self.frame_count = 0
        self.has_universal_interaction = False

        # Data
        self.solids = Solids()
        self.joints = Joints()
        self.composite_joints = Composite()
        self.relations = Relations()
        self.actions = Actions()
        self.interactions = Interactions()
        self.arrays = self.solids, self.joints, self.composite_joints, self.relations, self.actions, self.interactions

        # External configuration
        self.declarations = []

        # Strategy states
        self.joint_states = []
        self.final_joint_states = []
        self.kinematics_strategy = []
        self.dynamics_strategy = []
        
        # Ground
        index = self.solids.reserve(1)
        self.solids.names[index] = 'Ground',
        self.solids.j3dof[index] = -1
        self.solids.is_ghost[index] = 1
        self.solids.physics_array[index] = 0

    def invalidate_config(self):
        self.state = ConfigState.NO_READ_ALLOWED

    def invalidate_resources(self):
        self.state = min(ConfigState.STRATEGY_OK, self.state)

    def invalidate_kinematics(self):
        self.state = min(ConfigState.ALLOCATED_RESOURCES, self.state)

    def invalidate_dynamics(self):
        self.state = min(ConfigState.KINEMATICS_OK, self.state)

    def allocate_resources(self, frame_count):
        self.state = ConfigState.ALLOCATED_RESOURCES
        for arr in self.arrays:
            arr.allocate_results(frame_count)
        self.frame_count = frame_count

    def assert_no_universal(self):
        assert not self.has_universal_interaction, "Please make sure to declare all your solids and joints before adding a UniversalInteraction (Gravity, Inertia), To be safe you should add them right before calling System.set_sim_parameters"

    def kp_array(self, array, axis):
        return array.view(KpArray)._configure(self, axis)

    def __contains__(self, obj: "ConfigView"):
        return obj._config is self and any(obj._array() is arr for arr in self.arrays) and obj._index < obj._array().count


class ReadOnlyArray(np.ndarray):
    def __array_finalize__(self, obj):
        self.flags.writeable = False


class ConfigView:
    __slots__ = '_index', '_config'

    def __init__(self, config: Config, index: int):
        assert not isinstance(index, slice)
        self._index = index
        self._config = config

    def _array(self) -> ConfigArray:
        pass

    @property
    def name(self) -> str:
        return self._array().names[self._index]

    @classmethod
    def __forward__(cls, prop: KpProperty) -> property:
        """
        Create a property from KpProperty to acces object data from its index
        """
        
        invalidation_method = {
            KpProperty.Type.CONFIG: Config.invalidate_config,
            KpProperty.Type.PHYSICS: Config.invalidate_kinematics,
            KpProperty.Type.RESULT_KINEMATICS: Config.invalidate_kinematics,
            KpProperty.Type.RESULT_DYNAMICS: Config.invalidate_dynamics
        }[prop.type_]

        def getter(self: cls) -> np.ndarray:
            # user can't modify through getter().__setitem__(...), they have to go through setter(...) in order to invalidate config state properly
            return prop.__get__(self._array())[self._index].view(ReadOnlyArray).view(np.ndarray)
        
        def setter(self: cls, value: np.ndarray):
            invalidation_method(self._config)
            prop.__get__(self._array())[self._index] = value

        return property(getter, setter)

    @classmethod
    def assert_resources(cls, method):
        @functools.wraps(method)
        def n_method(self: cls, *args, **kwargs):
            assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, f'Call `System.set_sim_parameters` before messing with `{method.__qualname__}`'
            return method(self, *args, **kwargs)
        return n_method

    @classmethod
    def _create_subclass(cls, other: type) -> typing.Self:
        assert issubclass(other, cls) or issubclass(other, cls), f"Creating from: {cls}; Result class: {other}; None is derived from the other"
        return ConfigView.__new__(cls if issubclass(cls, other) else other)

    def _kp_array(self, array, axis=-1):
        return self._config.kp_array(array, axis)

    def __eq__(self, other: typing.Self):
        if other is None:
            return False
        if self._config is not other._config:
            raise ex.UnrelatedObjectsError()
        return self._array() is other._array() and self._index == other._index


class KpArray(np.ndarray):
    """
    Specialized ndarray that offers time derivatives
    """
    _config: Config
    _time_axis: int

    def __array_finalize__(self, obj, /):
        self._config = getattr(obj, '_config', None)
        self._time_axis = getattr(obj, '_time_axis', -1)

    def _configure(self, _config: Config, _time_axis: int):
        self._config = _config
        self._time_axis = _time_axis
        return self

    def _inherit(self, arr: np.ndarray):
        return arr.view(self.__class__)._configure(self._config, self._time_axis)

    def derivative(self):
        return self._inherit(cal.Derivation.derivative(self, self._time_axis, self._config.frame_time))
    
    def second_derivative(self):
        return self._inherit(cal.Derivation.second_derivative(self, self._time_axis, self._config.frame_time))


def disable_set(prop: property) -> property:
    def getter(self):
        return prop.__get__(self).copy()
    return property(getter)


def mirror_other(prop: property, other: property) -> property:
    def setter(self, value):
        prop.fset(self, value)
        other.fset(self, value)

    return property(prop.fget, setter)
