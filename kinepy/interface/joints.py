import kinepy.objects as obj
from kinepy.interface.metaclass import *
from kinepy.interface.decorators import to_function
from kinepy.math.geometry import rvec, ZERO_F
from kinepy.interface.interactions import RevoluteTorque, PrismaticTangent, PinSlotTangentTorque


class RevoluteJoint(obj.RevoluteJoint, metaclass=MetaUnit):
    _object: obj.RevoluteJoint
    read_only = ANGLE_, FORCE_, TORQUE_
    read_write = P1, P2

    def __init__(self): # noqa
        self.interaction = RevoluteTorque(self, ZERO_F)

    @to_function
    def set_torque(self, t):
        self.interaction.torque = t

    point = property(lambda self: self._object.s2.origin / units.SYSTEM[LENGTH] + rvec(self._object.s2.angle, self.p2))


class PrismaticJoint(obj.PrismaticJoint, metaclass=MetaUnit):
    _object: obj.PrismaticJoint
    read_only = SLIDING, NORMAL, TANGENT, TORQUE_
    read_write = A1, A2, D1, D2

    def __init__(self): # noqa
        self.interaction = PrismaticTangent(self, ZERO_F)

    @to_function
    def set_tangent(self, t):
        self.interaction.tangent = t


class PinSlotJoint(obj.PinSlotJoint, metaclass=MetaUnit):
    _object: obj.PinSlotJoint
    read_only = SLIDING, NORMAL, TANGENT, TORQUE_
    read_write = A1, D1, P2

    def __init__(self): # noqa
        self.interaction = PinSlotTangentTorque(self, ZERO_F, ZERO_F)

    @to_function
    def set_torque(self, t):
        self.interaction.torque = t

    @to_function
    def set_tangent(self, t):
        self.interaction.tangent = t

    point = property(lambda self: self._object.s2.origin / units.SYSTEM[LENGTH] + rvec(self._object.s2.angle, self.p2))


class RectangularJoint(obj.RectangularJoint, metaclass=MetaUnit):
    _object: obj.RectangularJoint
    read_only = SLIDING, TORQUE_, ('force_x', FORCE), ('force_y', FORCE)
    read_write = P1, P2, ANGLE_, A1, A2


class ThreeDegreesOfFreedomJoint(obj.ThreeDegreesOfFreedomJoint, metaclass=MetaUnit):
    _object: obj.ThreeDegreesOfFreedomJoint
    read_only = SLIDING, ANGLE_, TORQUE_, FORCE_
    read_write = P1, P2


J3DOF = ThreeDegreesOfFreedomJoint
