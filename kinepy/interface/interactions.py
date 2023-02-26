import kinepy.objects as obj
from kinepy.interface.metaclass import *
from kinepy.math.geometry import ZERO_21, unit, get_angle, get_zero, rvec
from kinepy.interface.decorators import get_object


class Gravity(obj.Gravity, metaclass=MetaUnit):
    read_write = ('g', ACCELERATION),


class Spring(obj.Spring, metaclass=MetaUnit):
    read_only = FORCE_,
    read_write = P1, P2, ('k', SPRING_CONSTANT), ('l0', LENGTH)


class SolidExternal:
    def __init__(self, unit_system, solid):
        self._unit_system, self.solid = unit_system, solid
        self.externals = []

    def append(self, force, torque, point):
        self.externals.append((
            (lambda: force * self._unit_system[FORCE]),
            (lambda: torque * self._unit_system[TORQUE]),
            point
        ))

    def set_ma(self, _):
        sol: obj.Solid = get_object(self.solid)
        for f, t, p in self.externals:
            sol.add_mech_action(f(), sol.origin + rvec(sol.angle, p), t())


class RevoluteTorque:

    def __init__(self, unit_system, rev, t):
        self._unit_system = unit_system
        self.rev, self._torque = rev, t

    torque = property(
        (lambda self: (lambda: self._torque() / self._unit_system[FORCE])),
        (lambda self, value: setattr(self, '_torque', (lambda: self._torque() * self._unit_system[FORCE])))
    )

    def set_ma(self, _):
        t = self._torque()
        self.rev.s1.add_mech_action(ZERO_21, 0., t)
        self.rev.s2.add_mech_action(ZERO_21, 0., -t)


class PrismaticTangent:

    def __init__(self, unit_system, pri, f):
        self._unit_system = unit_system
        self.pri, self._tangent = pri, f

    tangent = property(
        (lambda self: (lambda: self._tangent() / self._unit_system[FORCE])),
        (lambda self, value: setattr(self, '_tangent', (lambda: self._tangent() * self._unit_system[FORCE])))
    )

    def set_ma(self, _):
        u = unit(get_angle(self.pri, 0))
        f = self._tangent() * u
        p = get_zero(self.pri, 0, u)
        self.pri.s1.add_mech_action(f, p, 0.)
        self.pri.s2.add_mech_action(-f, p, 0.)


class PinSlotTangentTorque:

    def __init__(self, unit_system, pin, f, t):
        self._unit_system = unit_system
        self.pin, self._tangent, self._torque = pin, f, t

    def set_ma(self, _):
        u = unit(self.pin.s1.angle_ + self.pin.a1_)
        f, t = self._tangent() * u, self._torque()
        p = self.pin.s2.origin_ + rvec(self.pin.s2.angle_, self.pin.p2_)
        self.pin.s1.add_mech_action(f, p, t)
        self.pin.s2.add_mech_action(-f, p, -t)

    tangent = property(
        (lambda self: (lambda: self._tangent() / self._unit_system[FORCE])),
        (lambda self, value: setattr(self, '_tangent', (lambda: self._tangent() * self._unit_system[FORCE])))
    )

    torque = property(
        (lambda self: (lambda: self._torque() / self._unit_system[FORCE])),
        (lambda self, value: setattr(self, '_torque', (lambda: self._torque() * self._unit_system[FORCE])))
    )
