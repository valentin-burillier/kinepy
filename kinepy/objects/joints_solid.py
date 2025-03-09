from kinepy.objects.config import *
import kinepy.units as u
from kinepy.strategy.graph_data import JointType


@u.UnitSystem.class_
class Solid(ConfigView):
    __slots__ = 'name', '__3dof'

    def __init__(self, config: Config, index: int, name: str):
        self.name = name
        self.__3dof = []
        ConfigView.__init__(self, config, index)

    mass: u.Mass.phy = ConfigView.physics_view(Config.SOLID, 0, u.Mass.phy)
    moment_of_inertia: u.MomentOfInertia.phy = ConfigView.physics_view(Config.SOLID, 1, u.MomentOfInertia.phy)
    g: u.Length.point = ConfigView.physics_view(Config.SOLID, slice(2, 4), u.Length.point)

    def _get_3dof(self):
        if not self.__3dof:
            s_ghost_index = self._config.solid_physics.shape[0]
            self._config.add_solids(np.zeros((2, 4)))
            ghost_solids = CompositeJoint.GSolid(self._config, s_ghost_index, f'GhostSolid {s_ghost_index}'), CompositeJoint.GSolid(self._config, s_ghost_index+1, f'GhostSolid {s_ghost_index+1}')

            j_ghost_index = self._config.joint_config.shape[0]
            self._config.add_joints(
                np.array([[JointType.PRISMATIC.value, 0, s_ghost_index], [JointType.PRISMATIC.value, s_ghost_index, s_ghost_index + 1], [JointType.REVOLUTE.value, s_ghost_index + 1, self._index]]),
                np.array([[0, 0, 0, 0], [np.pi * 0.5, 0, np.pi * 0.5, 0], [0, 0, 0, 0]])
            )
            ghost_joints = (
                J3DOF.Axle(self._config, j_ghost_index, CompositeJoint.GSolid(self._config, 0, 'Ground'), ghost_solids[0], f"<{self.name}.x>"),
                J3DOF.Axle(self._config, j_ghost_index+1, ghost_solids[0], ghost_solids[1], f"<{self.name}.y>"),
                J3DOF.Angle(self._config, j_ghost_index+1, ghost_solids[1], self, f"<{self.name}.angle>")
            )
            self.__3dof.append(J3DOF(ghost_joints, ghost_solids))
        return self.__3dof[0]

    @property
    def x(self) -> "J3DOF.Axle":
        return self._get_3dof().x

    @property
    def y(self) -> "J3DOF.Axle":
        return self._get_3dof().y

    @property
    def angle(self) -> "J3DOF.Angle":
        return self._get_3dof().angle


class PrimitiveJoint(ConfigView):
    _s1_slice = slice(0, 2)
    _s2_slice = slice(2, 4)

    __slots__ = 's1', 's2', 'name'

    def __init__(self, config: Config, index: int, s1: Solid, s2: Solid, name: str = ''):
        self.name = name or f'{self.__class__.__name__}: {s2.name}/{s1.name}'
        self.s1: Solid = s1
        self.s2: Solid = s2
        ConfigView.__init__(self, config, index)

    def pilot(self):
        self._config.piloted_joints = np.r_[self._config.piloted_joints, self._index]

    def work(self):
        self._config.working_joints = np.r_[self._config.working_joints, self._index]

    def set_input(self, value):
        self._config.results.joint_values[self._index, :] = value

    def get_value(self):
        return self._config.results.joint_values[self._index, :]


@u.UnitSystem.class_
class Revolute(PrimitiveJoint):
    __slots__ = ()

    p1: u.Length.point = ConfigView.physics_view(Config.JOINT, PrimitiveJoint._s1_slice, u.Length.point, scalar=False)
    p2: u.Length.point = ConfigView.physics_view(Config.JOINT, PrimitiveJoint._s2_slice, u.Length.point, scalar=False)

    def set_input(self, value: u.Angle.phy):
        return PrimitiveJoint.set_input(self, value)

    def get_value(self) -> u.Angle.phy:
        return PrimitiveJoint.get_value(self)


@u.UnitSystem.class_
class Prismatic(PrimitiveJoint):
    _a1, _d1, _a2, _d2 = range(4)
    __slots__ = ()

    distance1: u.Length.phy = ConfigView.physics_view(Config.JOINT, _d1, u.Length.phy)
    distance2: u.Length.phy = ConfigView.physics_view(Config.JOINT, _d2, u.Length.phy)

    angle1: u.Angle.phy = ConfigView.physics_view(Config.JOINT, _a1, u.Angle.phy)
    angle2: u.Angle.phy = ConfigView.physics_view(Config.JOINT, _a2, u.Angle.phy)

    def set_input(self, value: u.Length.phy):
        return PrimitiveJoint.set_input(self, value)

    def get_value(self) -> u.Length.phy:
        return PrimitiveJoint.get_value(self)


def _disable_set(prop: property) -> property:
    return property(prop.fget)


def _mirror_other(prop: property, other: property) -> property:
    def setter(self: PrimitiveJoint, value):
        prop.fset(self, value)
        other.fset(self, value)

    return property(prop.fget, setter)


class CompositeJoint:
    __slots__ = '_joints', '_solids', '_initialized'

    class GSolid(Solid):
        __slots__ = ()

        mass: u.Mass.phy = _disable_set(Solid.mass)
        moment_of_inertia: u.MomentOfInertia.phy = _disable_set(Solid.moment_of_inertia)
        g: u.Length.point = _disable_set(Solid.g)

    def __init__(self, joints: tuple[PrimitiveJoint, ...], solids: tuple[GSolid, ...]):
        self._joints: tuple[PrimitiveJoint, ...] = joints
        self._solids: tuple[CompositeJoint.GSolid, ...] = solids
        self._initialized = None

    def __setattr__(self, key, value):
        if not hasattr(self, '_initialized'):
            return object.__setattr__(self, key, value)
        if hasattr(self.__class__, key) and isinstance(self.__class__.__dict__[key], property):
            prop: property = self.__class__.__dict__[key]
            return prop.__set__(self, value)
        raise ValueError(f'{self.__class__.__name__} objects cannot be internally modified')

    @property
    def s1(self) -> Solid:
        return self._joints[0].s1

    @property
    def s2(self) -> Solid:
        return self._joints[-1].s2


class PinSlot(CompositeJoint):
    _sliding, _angle = range(2)
    __slots__ = ()

    class Sliding(Prismatic):
        __slots__ = ()

        angle1: u.Angle.phy = _mirror_other(Prismatic.angle1, Prismatic.angle2)
        angle2: u.Angle.phy = _mirror_other(Prismatic.angle2, Prismatic.angle1)

        distance1 = _disable_set(Prismatic.distance1)

    class Angle(Revolute):
        __slots__ = ()
        p2 = _disable_set(Revolute.p2)

    @property
    def sliding(self) -> Sliding:
        return self._joints[self._sliding]

    @property
    def angle(self) -> Angle:
        return self._joints[self._angle]


@u.UnitSystem.class_
class Translation(CompositeJoint):
    _x, _y = range(2)
    __slots__ = ()

    class AxleX(Prismatic):
        __slots__ = ()
        angle1: u.Angle.phy = _mirror_other(Prismatic.angle1, Prismatic.angle2)
        angle2: u.Angle.phy = _mirror_other(Prismatic.angle2, Prismatic.angle1)

        distance2 = _disable_set(Prismatic.distance2)

    class AxleY(Prismatic):
        __slots__ = ()
        distance1 = _disable_set(Prismatic.distance1)

    @property
    def x(self) -> AxleX:
        return self._joints[self._x]

    @property
    def y(self) -> AxleY:
        return self._joints[self._y]

    @property
    def angle_diff(self) -> u.Angle.phy:
        _y = self.y
        return np.diff(_y._config.joint_physics[_y._index, [Prismatic._a1, Prismatic._a2]])

    @angle_diff.setter
    def angle_diff(self, value: u.Angle.phy):
        _y = self.y
        _y._config.joint_physics[_y._index, Prismatic._a2] = _y._config.joint_physics[_y._index, Prismatic._a1] + value


class J3DOF(CompositeJoint):
    _x, _y, _angle = range(3)
    __slots__ = ()

    class Axle(Prismatic):
        __slots__ = ()
        angle1 = _disable_set(Prismatic.angle1)
        angle2 = _disable_set(Prismatic.angle2)
        distance1 = _disable_set(Prismatic.distance1)
        distance2 = _disable_set(Prismatic.distance2)

    class Angle(Revolute):
        __slots__ = ()
        p1 = _disable_set(Revolute.p1)

    @property
    def x(self) -> Axle:
        return self._joints[self._x]

    @property
    def y(self) -> Axle:
        return self._joints[self._y]

    @property
    def angle(self) -> Angle:
        return self._joints[self._angle]
