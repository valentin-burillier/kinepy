import numpy as np


class Result:
    solid_values: np.ndarray
    joint_values: np.ndarray


class Config:
    SOLID = 'solid_physics'
    JOINT = 'joint_physics'

    def __init__(self):
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
        # r, v0, pressure_angle
        self.relation_physics = np.zeros((0, 3), float)

        self.joint_states = []
        self.final_joint_states = []

        self.results = Result()

    def allocate_results(self, frame_count):
        self.results.solid_values = np.zeros((self.solid_physics.shape[0], 4, frame_count), float)
        self.results.solid_values[:, 2, :] = 1.

        self.results.joint_values = np.zeros((self.joint_config.shape[0], frame_count), float)

    def add_solids(self, physics: np.ndarray):
        self.solid_physics = np.r_[self.solid_physics, physics]

    def add_joints(self, config: np.ndarray, physics: np.ndarray):
        self.joint_config = np.r_[self.joint_config, config]
        self.joint_physics = np.r_[self.joint_physics, physics]

    def add_relations(self, config: np.ndarray, physics: np.ndarray):
        self.relation_config = np.r_[self.relation_config, config]
        self.relation_physics = np.r_[self.relation_physics, physics]


class ConfigView:
    __slots__ = '_config', '_index', '_initialized'

    def __init__(self, config: Config, index: int):
        self._config: Config = config
        self._index: int = index
        self._initialized: None = None

    def __setattr__(self, key, value):
        if not hasattr(self, '_initialized'):
            return object.__setattr__(self, key, value)
        if hasattr(self.__class__, key) and isinstance(self.__class__.__dict__[key], property):
            prop: property = self.__class__.__dict__[key]
            return prop.__set__(self, value)
        raise ValueError(f'You should not be internally modifying {self.__class__.__name__} objects')

    @classmethod
    def physics_view(cls, array_name: str, sub_index, phy, scalar=True) -> property:
        get = sub_index[0] if isinstance(sub_index, (tuple, list)) and scalar else sub_index

        def getter(self: cls) -> phy:
            return getattr(self._config, array_name)[self._index, get]

        def setter(self: cls, value: phy) -> None:
            getattr(self._config, array_name)[self._index, sub_index] = value

        return property(getter, setter)

    def check_against(self, config: Config, array: np.ndarray) -> bool:
        if self._config is not config:
            return False
        if self._index >= array.shape[0]:
            return False
        return True
