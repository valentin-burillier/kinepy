import kinepy.objects.config as cfg
import kinepy.math.geometry as geo
import kinepy.strategy.types as strategy
import numpy as np


class SolidBase(cfg.ConfigView):
    def _array(self) -> cfg.Solids:
        return self._config.solids
    
    def __new__(cls, config: cfg.Config, index: int):
        return cls._create_subclass(GhostSolid if config.solids.is_ghost[index] else Solid)

    _position = cfg.Solids.position()
    _orientation = cfg.Solids.orientation()

    @cfg.ConfigView.assert_resources
    def get_origin(self):
        return self._kp_array(self._position)

    @cfg.ConfigView.assert_resources
    def get_point(self, p=(0.0, 0.0)):
        return self._kp_array(geo.Position.point(self._config, self._index, np.array(p)))

    @cfg.ConfigView.assert_resources
    def get_vector(self, v=(0.0, 0.0)):
        return self._kp_array(geo.Position.vector(self._config, self._index, np.array(v)))

    @cfg.ConfigView.assert_resources
    def get_angle(self):
        ori = self._orientation
        _angle = np.arctan2(ori[..., 1], ori[..., 0])
        geo.Orientation.make_angle_continuous(_angle)
        return self._kp_array(_angle)


class GhostSolid(SolidBase):
    """
    A ghost: no mass, no inertia, you can't see it, you can't touch it... At least it has a name, and you may have the privilege to know its position
    """


class Solid(SolidBase):
    mass = cfg.Solids.mass()
    moment_of_inertia = cfg.Solids.moment_of_inertia()
    g = cfg.Solids.g()
    _3dof = cfg.Solids.j3dof()

    def _get_3dof(self):
        if self._3dof < 0:
            self._config.assert_no_universal()

            # solids
            ghost_s_indices = self._config.solids.reserve(2)
            self._config.solids.names[ghost_s_indices] = f'GhostSolid {ghost_s_indices.start}', f'GhostSolid {ghost_s_indices.start + 1}'
            self._config.solids.j3dof[ghost_s_indices] = -1
            self._config.solids.is_ghost[ghost_s_indices] = 1
            self._config.solids.physics_array[ghost_s_indices] = 0

            # joints
            ghost_j_indices = self._config.joints.reserve(3)
            # config part
            self._config.joints.names[ghost_j_indices] = f"<{self.name}.x>", f"<{self.name}.y>", f"<{self.name}.angle>"
            self._config.joints.type_[ghost_j_indices] = cfg.Joints.Type.J_AXLE, cfg.Joints.Type.J_AXLE, cfg.Joints.Type.GHOST_ANGLE
            self._config.joints.solids[ghost_j_indices] = (0, ghost_s_indices.start), (ghost_s_indices.start, ghost_s_indices.start+1), (ghost_s_indices.start+1, self._index)
            # physics part
            self._config.joints.revolute_p1[ghost_j_indices] = (0, 0), (np.pi * 0.5, 0), (0, 0)
            self._config.joints.revolute_p2[ghost_j_indices] = (0, 0), (np.pi * 0.5, 0), (0, 0)

            # j3dof
            self._3dof = self._config.composite_joints.reserve(1).start
            self._config.composite_joints.names[self._3dof] = f'<{self.name}.3dof>'
            self._config.composite_joints.type_[self._3dof] = cfg.Composite.Type.J3DOF
            self._config.composite_joints.first_ghost_solid[self._3dof] = ghost_s_indices.start
            self._config.composite_joints.first_ghost_joint[self._3dof] = ghost_j_indices.start
        return J3DOF(self._config, self._3dof)

    @property
    def x(self) -> "J3DOFAxle":
        return self._get_3dof().x

    @property
    def y(self) -> "J3DOFAxle":
        return self._get_3dof().y

    @property
    def angle(self) -> "J3DOFAngle":
        return self._get_3dof().angle


class Joint(cfg.ConfigView):
    def _array(self) -> cfg.Joints:
        return self._config.joints

    def __new__(cls, config: cfg.Config, index: int):
        _dict: dict[cfg.Joints.Type, type[Joint]] = {
            cfg.Joints.Type.REVOLUTE: Revolute,
            cfg.Joints.Type.PRISMATIC: Prismatic,
            cfg.Joints.Type.GHOST_ANGLE: GhostAngle,
            cfg.Joints.Type.X: _X,
            cfg.Joints.Type.Y: TranslationAxleY,
            cfg.Joints.Type.J_AXLE: J3DOFAxle
        }
        final_type: type[Joint] = _dict[cfg.Joints.Type(config.joints.type_[index])]
        return cls._create_subclass(final_type)

    _type = cfg.Joints.type_()
    _s1 = cfg.Joints.s1()
    _s2 = cfg.Joints.s2()
    _state = cfg.Joints.state()

    _force = cfg.Joints.force()
    _torque = cfg.Joints.torque()
    _value = cfg.Joints.value()

    def _set_state_bit(self, bit, value):
        self._state = (self._state ^ (self._state & (1 << bit))) | (value << bit)

    def pilot(self, value=True):
        self._set_state_bit(0, value)

    def work(self, value=True):
        self._set_state_bit(1, value)

    @cfg.ConfigView.assert_resources
    def set_input(self, value):
        self._value = value

    def _get_value(self) -> np.ndarray:
        if self._config.joint_states[self._index] ^ strategy.JointFlags.READY_FOR_USER:
            strategy.JointValueComputationStep(self._index, cfg.Joints.Type(self._type), self._config.joint_states[self._index], self._s1, self._s2).solve_kinematics(self._config)
            self._config.joint_states[self._index] = strategy.JointFlags.READY_FOR_USER
        return self._value

    @cfg.ConfigView.assert_resources
    def get_value(self):
        return self._kp_array(self._get_value())

    @cfg.ConfigView.assert_resources
    def get_force(self):
        return self._kp_array(self._force)

    @cfg.ConfigView.assert_resources
    def get_torque(self):
        return self._kp_array(self._torque)

    @property
    def s1(self) -> SolidBase:
        return SolidBase(self._config, self._s1)

    @property
    def s2(self) -> SolidBase:
        return SolidBase(self._config, self._s2)


class Revolute(Joint):
    p1 = cfg.Joints.revolute_p1()
    p2 = cfg.Joints.revolute_p2()


class Prismatic(Joint):
    angle1 = cfg.Joints.prismatic_angle1()
    angle2 = cfg.Joints.prismatic_angle2()
    distance1 = cfg.Joints.prismatic_distance1()
    distance2 = cfg.Joints.prismatic_distance2()


class CompositeJoint(cfg.ConfigView):
    def _array(self) -> cfg.Composite:
        return self._config.composite_joints

    _type = cfg.Composite.type_()
    _first_solid = cfg.Composite.first_ghost_solid()
    _first_joint = cfg.Composite.first_ghost_joint()

    def __new__(cls, config: cfg.Config, index: int):
        _dict: dict[cfg.Composite.Type, type[CompositeJoint]] = {
            cfg.Composite.Type.PIN_SLOT: PinSlot,
            cfg.Composite.Type.TRANSLATION: Translation,
            cfg.Composite.Type.J3DOF: J3DOF
        }
        final_type: type[CompositeJoint] = _dict[cfg.Composite.Type(config.composite_joints.type_[index])]
        return cls._create_subclass(final_type)

    @property
    def s1(self) -> SolidBase:
        return Joint(self._config, self._first_joint).s1

    @property
    def s2(self) -> SolidBase:
        return Joint(self._config, self._first_joint + cfg.Composite.Type(self._type).ghost_count).s2

    @classmethod
    def _forward_property(cls, joint_prop: property, prop: property) -> property:
        def getter(self: cls):
            return prop.__get__(joint_prop.__get__(self))

        def setter(self: cls, value):
            return prop.__set__(joint_prop.__get__(self), value)

        return property(getter, setter)


class _X(Joint):
    angle1 = cfg.mirror_other(Prismatic.angle1, Prismatic.angle2)
    distance1 = Prismatic.distance1


PinSlotSliding = _X
TranslationAxleX = _X


class GhostAngle(Joint):
    p2 = Revolute.p2


PinSlotAngle = GhostAngle


class PinSlot(CompositeJoint):
    _sliding, _angle = range(2)

    @property
    def sliding(self) -> PinSlotSliding:
        return PinSlotSliding(self._config, self._first_joint + self._sliding)

    @property
    def angle(self) -> PinSlotAngle:
        return PinSlotAngle(self._config, self._first_joint + self._angle)

    angle1 = CompositeJoint._forward_property(sliding, _X.angle1)
    distance1 = CompositeJoint._forward_property(sliding, _X.distance1)

    p2 = CompositeJoint._forward_property(angle, PinSlotAngle.p2)


class TranslationAxleY(Joint):
    angle1 = Prismatic.angle1
    angle2 = Prismatic.angle2
    distance2 = Prismatic.distance2


class Translation(CompositeJoint):
    _x, _y = range(2)

    @property
    def x(self) -> TranslationAxleX:
        return TranslationAxleX(self._config, self._first_joint + self._x)

    @property
    def y(self) -> TranslationAxleY:
        return TranslationAxleY(self._config, self._first_joint + self._y)

    @property
    def angle_diff(self):
        _y = self.y
        return self.y.angle2 - self.y.angle1

    @angle_diff.setter
    def angle_diff(self, value):
        self.y.angle2 = self.y.angle1 + value

    angle1 = CompositeJoint._forward_property(x, _X.angle1)
    angle2 = CompositeJoint._forward_property(y, TranslationAxleY.angle1)


class J3DOFAxle(Joint):
    pass


J3DOFAngle = GhostAngle


class J3DOF(CompositeJoint):
    _x, _y, _angle = range(3)

    @property
    def x(self) -> J3DOFAxle:
        return J3DOFAxle(self._config, self._first_joint + self._x)

    @property
    def y(self) -> J3DOFAxle:
        return J3DOFAxle(self._config, self._first_joint + self._y)

    @property
    def angle(self) -> J3DOFAngle:
        return J3DOFAngle(self._config, self._first_joint + self._angle)

    p2 = CompositeJoint._forward_property(angle, J3DOFAngle.p2)
