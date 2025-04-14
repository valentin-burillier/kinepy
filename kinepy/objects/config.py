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


class Config:
    SOLID = 'solid_physics'
    JOINT = 'joint_physics'
    RELATION = 'relation_physics'

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

    JOINT_DYN_FORCE = slice(0, 2)
    JOINT_DYN_TORQUE = 2

    RELATION_TYPE = 0
    RELATION_JOINTS = slice(1, 3)
    RELATION_TYPE_JOINTS = slice(0, 3)
    RELATION_G1 = 3
    RELATION_G2 = 4

    RELATION_V0 = 0
    RELATION_R = 1
    RELATION_PRESSURE_ANGLE = 2
    RELATION_R1 = 1
    RELATION_R2 = 2
    RELATION_T0 = 3

    def __init__(self):
        # name
        self.solid_config = ['Ground']
        # mass, moment_of_inertia, g.x, g.y
        self.solid_physics = np.zeros((1, 4), float)

        # _type, s1, s2
        self.joint_config = np.zeros((0, 3), int)
        # p1.x, p1.y, p2.x, p2.y
        # angle1, distance1, angle2, distance2
        self.joint_physics = np.zeros((0, 4), float)

        self.piloted_joints = np.zeros((0,), int)
        self.working_joints = np.zeros((0,), int)

        # _type, j1, j2, g1, g2
        self.relation_config = np.zeros((0, 5), int)
        # v0, r, _, _
        # v0, r, pressure_angle, _
        # v0, r1, r2, t0
        self.relation_physics = np.zeros((0, 4), float)

        # solid
        self.action_config = np.zeros((0,), int)
        # ap.x, ap.y
        self.action_physics = np.zeros((0, 2), float)

        self.joint_states = []
        self.final_joint_states = []

        self.frame_time = 0.0

        self.results = Result()

        self.state = ConfigState.NO_READ_ALLOWED

    def invalidate_config(self):
        self.state = ConfigState.NO_READ_ALLOWED

    def invalidate_physics(self):
        self.state = min(ConfigState.ALLOCATED_RESOURCES, self.state)

    def allocate_results(self, frame_count, frame_time=0.0):
        self.state = ConfigState.ALLOCATED_RESOURCES
        # x, y, cos(a), sin(a)
        self.results.solid_values = np.zeros((self.solid_physics.shape[0], 4, frame_count), float)
        self.results.solid_values[:, 2, :] = 1.

        self.results.joint_values = np.zeros((self.joint_config.shape[0], frame_count), float)

        self.allocated_results_dyn(frame_count, frame_time)

    def allocated_results_dyn(self, frame_count, frame_time):
        self.frame_time = frame_time
        # force x, force y, gx, gy, torque(g)
        self.results.solid_dynamics = np.zeros((self.solid_physics.shape[0], 5, frame_count))
        # force x, force y, torque
        self.results.joint_dynamics = np.zeros((self.joint_config.shape[0], 3, frame_count))

        # force.x, force.y, torque
        self.results.action_values = np.zeros((self.action_config.shape[0], 3, frame_count))

    def add_solids(self, names: list[str], physics: np.ndarray):
        self.invalidate_config()
        self.solid_config.extend(names)
        self.solid_physics = np.r_[self.solid_physics, physics]

    def add_joints(self, config: np.ndarray, physics: np.ndarray):
        self.invalidate_config()
        self.joint_config = np.r_[self.joint_config, config]
        self.joint_physics = np.r_[self.joint_physics, physics]

    def add_relations(self, config: np.ndarray, physics: np.ndarray):
        self.invalidate_config()
        self.relation_config = np.r_[self.relation_config, config]
        self.relation_physics = np.r_[self.relation_physics, physics]

    def add_actions(self, config: np.ndarray, physics: np.ndarray):
        self.invalidate_config()
        self.action_config = np.r_[self.action_config, config]
        self.action_physics = np.r_[self.action_physics, physics]


class Immutable:
    __slots__ = '_initialized'

    def __init__(self):
        self._initialized: None = None

    def __setattr__(self, key, value):
        if not hasattr(self, '_initialized'):
            return object.__setattr__(self, key, value)
        if hasattr(self.__class__, key) and isinstance(getattr(self.__class__, key), property):
            prop: property = getattr(self.__class__, key)
            return prop.__set__(self, value)
        raise ValueError(f'You should not be internally modifying {self.__class__.__name__} objects')


class ConfigView(Immutable):
    __slots__ = '_config', '_index'

    def __init__(self, config: Config, index: int):
        self._config: Config = config
        self._index: int = index
        Immutable.__init__(self)

    @classmethod
    def physics_view(cls, array_name: str, sub_index, phy, scalar=True) -> property:
        get = sub_index[0] if isinstance(sub_index, (tuple, list)) and scalar else sub_index

        def getter(self: cls) -> phy:
            return getattr(self._config, array_name)[self._index, get]

        def setter(self: cls, value: phy) -> None:
            self._config.invalidate_physics()
            getattr(self._config, array_name)[self._index, sub_index] = value

        return property(getter, setter)

    def check_against(self, config: Config, array: np.ndarray) -> bool:
        if self._config is not config:
            return False
        if self._index >= array.shape[0]:
            return False
        return True
