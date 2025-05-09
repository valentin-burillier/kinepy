from kinepy.objects.config import *
from kinepy.objects.action import Action
import kinepy.units as u
from kinepy.strategy.graph_data import JointType
import kinepy.math.geometry as geo
import kinepy.strategy.types as strategy

from typing import Self


@u.UnitSystem.class_
class Solid(ConfigView):
    def __new__(cls, config: Config, index: int):
        if not index:
            # Ground is a ghost solid no matter the config
            return ConfigView.__new__(GhostSolid)
        return ConfigView.__new__(cls)

    def _names(self) -> list[str]:
        return self._config.solid_names

    def _config_arr(self) -> np.ndarray[int]:
        return self._config.solid_config

    def _physics(self) -> np.ndarray[float]:
        return self._config.solid_physics

    name: str = ConfigView._name()
    mass: u.Mass.phy = ConfigView._physics_view(Config.SOLID_MASS, u.Mass.phy)
    moment_of_inertia: u.MomentOfInertia.phy = ConfigView._physics_view(Config.SOLID_MOMENT_OF_INERTIA, u.MomentOfInertia.phy)
    g: u.Length.point = ConfigView._physics_view(Config.SOLID_CFG_G, u.Length.point)

    _3dof: int = ConfigView._config_view(Config.SOLID_3DOF)

    def _get_3dof(self):
        if self._3dof < 0:
            s_ghost_index = self._config.solid_physics.shape[0]
            self._config.add_solids(
                [f'GhostSolid {s_ghost_index}', f'GhostSolid {s_ghost_index + 1}'],
                np.zeros((2, 4))
            )

            j_ghost_index = self._config.joint_config.shape[0]
            self._config.add_joints(
                [f"<{self.name}.x>", f"<{self.name}.y>", f"<{self.name}.angle>"],
                np.array([
                    [JointType.J_AXLE.value, 0, s_ghost_index],
                    [JointType.J_AXLE.value, s_ghost_index, s_ghost_index + 1],
                    [JointType.GHOST_ANGLE.value, s_ghost_index + 1, self._index]
                ]),
                np.array([
                    [0, 0, 0, 0],
                    [np.pi * 0.5, 0, np.pi * 0.5, 0],
                    [0, 0, 0, 0]
                ])
            )
            self._3dof = self._config.composite_joint_config.shape[0]
            self._config.add_composite(CompositeType.J3DOF.value, (j_ghost_index, j_ghost_index+1, j_ghost_index+2), (s_ghost_index, s_ghost_index+1))
        return J3DOF(self._config, self._3dof)

    def __eq__(self, other: Self):
        return isinstance(other, Solid) and self._config is other._config and self._index == other._index

    @property
    def x(self) -> "J3DOFAxle":
        return self._get_3dof().x

    @property
    def y(self) -> "J3DOFAxle":
        return self._get_3dof().y

    @property
    def angle(self) -> "J3DOFAngle":
        return self._get_3dof().angle

    def get_origin(self) -> u.Length.point:
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters`before reading Solid kinematics"
        return geo.Position.get(self._config, self._index).swapaxes(0, 1).view(KpArray)._configure(self._config, -1)

    def get_point(self, p: u.Length.point = (0.0, 0.0)) -> u.Length.point:
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters`before reading Solid kinematics"
        return geo.Position.point(self._config, self._index, np.array(p)).swapaxes(0, 1).view(KpArray)._configure(self._config, -1)

    def get_vector(self, v: u.point_type = (0.0, 0.0)) -> u.point_type:
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters`before reading Solid kinematics"
        return geo.Position.local_point(self._config, self._index, np.array(v)).swapaxes(0, 1).view(KpArray)._configure(self._config, -1)

    def get_angle(self):
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters`before reading Solid kinematics"
        ori = geo.Orientation.get(self._config, self._index)
        _angle = np.arctan2(ori[..., 1], ori[..., 0])
        geo.Orientation.make_angle_continuous(_angle)
        return _angle.view(KpArray)._configure(self._config, -1)

    def add_action(self, ap: u.Length.point = (0, 0)) -> Action:
        _action_index = self._config.action_config.shape[0]
        self._config.add_actions(
            np.array([[self._index, ActionMode.NO_INDIRECTION.value, 0]]),
            np.array([ap])
        )
        return Action(self._config, _action_index)


class PrimitiveJoint(ConfigView):
    def __new__(cls, config, index):
        _dict: dict[JointType, type[PrimitiveJoint]] = {
            JointType.REVOLUTE: Revolute,
            JointType.PRISMATIC: Prismatic,
            JointType.GHOST_ANGLE: GhostAngle,
            JointType.X: _X,
            JointType.Y: TranslationAxleY,
            JointType.J_AXLE: J3DOFAxle
        }
        return ConfigView.__new__(_dict.get(JointType(config.joint_config[index, Config.JOINT_TYPE]), cls))

    def _names(self) -> list[str]:
        return self._config.joint_names

    def _config_arr(self) -> np.ndarray[int]:
        return self._config.joint_config

    def _physics(self) -> np.ndarray[float]:
        return self._config.joint_physics

    name: str = ConfigView._name()
    _type = ConfigView._config_view(Config.JOINT_TYPE)
    _s1 = ConfigView._config_view(Config.JOINT_S1)
    _s2 = ConfigView._config_view(Config.JOINT_S2)

    @property
    def s1(self) -> Solid:
        return Solid(self._config, self._s1)

    @property
    def s2(self) -> Solid:
        return Solid(self._config, self._s2)

    def pilot(self):
        self._config.invalidate_config()
        self._config.piloted_joints = np.r_[self._config.piloted_joints, self._index]

    def work(self):
        self._config.invalidate_config()
        self._config.working_joints = np.r_[self._config.working_joints, self._index]

    def set_input(self, value):
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters` before setting Joint values"
        self._config.results.joint_values[self._index] = value

    def _get_value(self) -> np.ndarray:
        if self._config.joint_states[self._index] ^ strategy.JointFlags.READY_FOR_USER:
            strategy.JointValueComputationStep(self._index, JointType(self._type), self._config.joint_states[self._index], self.s1._index, self.s2._index).solve_kinematics(self._config)
            self._config.joint_states[self._index] = strategy.JointFlags.READY_FOR_USER
        return self._config.results.joint_values[self._index]

    def get_value(self):
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters` before reading Joint values"
        return self._get_value().view(KpArray)._configure(self._config, -1)

    def get_force(self):
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters` before reading Joint dynamics"
        return self._config.results.joint_dynamics[self._index, :, Config.JOINT_DYN_FORCE].view(KpArray)._configure(self._config, -2)

    def get_torque(self):
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters` before reading Joint dynamics"
        return self._config.results.joint_dynamics[self._index, :, Config.JOINT_DYN_TORQUE].view(KpArray)._configure(self._config, -1)

    def __eq__(self, other: Self):
        return isinstance(other, PrimitiveJoint) and self._config is other._config and self._index == other._index


@u.UnitSystem.class_
class Revolute(PrimitiveJoint):
    p1: u.Length.point = ConfigView._physics_view(Config.JOINT_P1, u.Length.point)
    p2: u.Length.point = ConfigView._physics_view(Config.JOINT_P2, u.Length.point)

    def set_input(self, value: u.Angle.phy):
        return PrimitiveJoint.set_input(self, value)

    def get_value(self) -> u.Angle.phy:
        return PrimitiveJoint.get_value(self)


@u.UnitSystem.class_
class Prismatic(PrimitiveJoint):
    distance1: u.Length.phy = ConfigView._physics_view(Config.JOINT_D1, u.Length.phy)
    distance2: u.Length.phy = ConfigView._physics_view(Config.JOINT_D2, u.Length.phy)

    angle1: u.Angle.phy = ConfigView._physics_view(Config.JOINT_A1, u.Angle.phy)
    angle2: u.Angle.phy = ConfigView._physics_view(Config.JOINT_A2, u.Angle.phy)

    def set_input(self, value: u.Length.phy):
        return PrimitiveJoint.set_input(self, value)

    def get_value(self) -> u.Length.phy:
        return PrimitiveJoint.get_value(self)


class GhostSolid(Solid):
    mass: u.Mass.phy = disable_set(Solid.mass)
    moment_of_inertia: u.MomentOfInertia.phy = disable_set(Solid.moment_of_inertia)
    g: u.Length.point = disable_set(Solid.g)

    def _get_3dof(self):
        raise ValueError("Ghost solids can't be controlled this way")


class CompositeType(enum.Enum):
    PIN_SLOT, TRANSLATION, J3DOF = range(3)


class CompositeJoint(ConfigView):
    def _config_arr(self) -> np.ndarray[int]:
        return self._config.composite_joint_config

    def _joint(self, index) -> int:
        return int(self._config_arr()[self._index, Config.COMPOSITE_JOINTS][index])

    _ghost_counts = {
        CompositeType.PIN_SLOT: 2,
        CompositeType.TRANSLATION: 2,
        CompositeType.J3DOF: 3
    }

    _type = ConfigView._config_view(Config.COMPOSITE_TYPE)

    @property
    def s1(self) -> Solid:
        return PrimitiveJoint(self._config, self._joint(0)).s1

    @property
    def s2(self) -> Solid:
        return PrimitiveJoint(self._config, self._joint(self._ghost_counts[CompositeType(self._type)] - 1)).s2

    @classmethod
    def _forward_property(cls, joint_prop: property, prop: property) -> property:
        def getter(self: cls):
            return prop.__get__(joint_prop.__get__(self))

        def setter(self: cls, value):
            return prop.__set__(joint_prop.__get__(self), value)

        return property(getter, setter)


class _X(Prismatic):
    angle1: u.Angle.phy = mirror_other(Prismatic.angle1, Prismatic.angle2)
    angle2: u.Angle.phy = mirror_other(Prismatic.angle2, Prismatic.angle1)

    distance2 = disable_set(Prismatic.distance2)

    @property
    def s2(self) -> Solid:
        return GhostSolid(self._config, self._s2)


PinSlotSliding = _X
TranslationAxleX = _X


class GhostAngle(Revolute):
    p1 = disable_set(Revolute.p1)

    @property
    def s1(self) -> Solid:
        return GhostSolid(self._config, self._s1)


PinSlotAngle = GhostAngle


class PinSlot(CompositeJoint):
    _sliding, _angle = range(2)

    @property
    def sliding(self) -> PinSlotSliding:
        return PinSlotSliding(self._config, self._joint(self._sliding))

    @property
    def angle(self) -> PinSlotAngle:
        return PinSlotAngle(self._config, self._joint(self._angle))

    angle1 = CompositeJoint._forward_property(sliding, _X.angle1)
    distance1 = CompositeJoint._forward_property(sliding, _X.distance1)

    p2 = CompositeJoint._forward_property(angle, PinSlotAngle.p2)


class TranslationAxleY(Prismatic):
    distance1 = disable_set(Prismatic.distance1)

    @property
    def s1(self) -> Solid:
        return GhostSolid(self._config, self._s1)


@u.UnitSystem.class_
class Translation(CompositeJoint):
    _x, _y = range(2)

    @property
    def x(self) -> TranslationAxleX:
        return TranslationAxleX(self._config, self._joint(self._x))

    @property
    def y(self) -> TranslationAxleY:
        return TranslationAxleY(self._config, self._joint(self._y))

    @property
    def angle_diff(self) -> u.Angle.phy:
        _y = self.y
        return np.diff(_y._config.joint_physics[_y._index, [Config.JOINT_A1, Config.JOINT_A2]])

    @angle_diff.setter
    def angle_diff(self, value: u.Angle.phy):
        _y = self.y
        _y._config.joint_physics[_y._index, Config.JOINT_A2] = _y._config.joint_physics[_y._index, Config.JOINT_A1] + value

    angle1 = CompositeJoint._forward_property(x, _X.angle1)
    angle2 = CompositeJoint._forward_property(y, TranslationAxleY.angle1)


class J3DOFAxle(Prismatic):
    angle1 = disable_set(Prismatic.angle1)
    angle2 = disable_set(Prismatic.angle2)
    distance1 = disable_set(Prismatic.distance1)
    distance2 = disable_set(Prismatic.distance2)

    @property
    def s1(self) -> Solid:
        return GhostSolid(self._config, self._s1)

    @property
    def s2(self) -> Solid:
        return GhostSolid(self._config, self._s2)


J3DOFAngle = GhostAngle


class J3DOF(CompositeJoint):
    _x, _y, _angle = range(3)

    @property
    def x(self) -> J3DOFAxle:
        return J3DOFAxle(self._config, self._joint(self._x))

    @property
    def y(self) -> J3DOFAxle:
        return J3DOFAxle(self._config, self._joint(self._y))

    @property
    def angle(self) -> J3DOFAngle:
        return J3DOFAngle(self._config, self._joint(self._angle))

    p2 = CompositeJoint._forward_property(angle, J3DOFAngle.p2)
