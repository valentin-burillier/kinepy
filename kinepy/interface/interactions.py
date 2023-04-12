import kinepy.objects as obj
from kinepy.interface.metaclass import *
import kinepy.units as units
from kinepy.math.geometry import ZERO_21, unit, get_angle, get_zero, rvec
from kinepy.interface.decorators import get_object


class Gravity(obj.Gravity, metaclass=MetaUnit):
    read_write = ('g', ACCELERATION),


class Spring(obj.Spring, metaclass=MetaUnit):
    read_only = FORCE_,
    read_write = P1, P2, ('k', SPRING_CONSTANT), ('l0', LENGTH)


class SolidExternal:
    def __init__(self, solid):
        self.solid = solid
        self.externals = []

    def append(self, force, torque, point):
        self.externals.append((
            (lambda: force() * units.SYSTEM[FORCE]),
            (lambda: torque() * units.SYSTEM[TORQUE]),
            point
        ))

    def set_ma(self, _):
        sol: obj.Solid = get_object(self.solid)
        for f, t, p in self.externals:
            sol.add_mech_action(f(), sol.origin + rvec(sol.angle, p), t())


class RevoluteTorque:

    def __init__(self, rev, t):
        self.rev, self._torque = rev, t

    torque = property(
        (lambda self: (lambda: self._torque() / units.SYSTEM[FORCE])),
        (lambda self, value: setattr(self, '_torque', (lambda: value() * units.SYSTEM[FORCE])))
    )

    def set_ma(self, _):
        t = self._torque()
        get_object(self.rev).s1.add_mech_action(ZERO_21, 0., t)
        get_object(self.rev).s2.add_mech_action(ZERO_21, 0., -t)


class PrismaticTangent:

    def __init__(self, pri, f):
        self.pri, self._tangent = pri, f

    tangent = property(
        (lambda self: (lambda: self._tangent() / units.SYSTEM[FORCE])),
        (lambda self, value: setattr(self, '_tangent', (lambda: value() * units.SYSTEM[FORCE])))
    )

    def set_ma(self, _):
        u = unit(get_angle(get_object(self.pri), 0))
        f = self._tangent() * u
        p = get_zero(get_object(self.pri), 0, u)
        get_object(self.pri).s1.add_mech_action(f, p, 0.)
        get_object(self.pri).s2.add_mech_action(-f, p, 0.)


class PinSlotTangentTorque:

    def __init__(self, pin, f, t):
        self.pin, self._tangent, self._torque = get_object(pin), f, t

    def set_ma(self, _):
        u = unit(self.pin.s1.angle + self.pin.a1)
        f, t = self._tangent() * u, self._torque()
        p = self.pin.s2.origin + rvec(self.pin.s2.angle, self.pin.p2)
        self.pin.s1.add_mech_action(f, p, t)
        self.pin.s2.add_mech_action(-f, p, -t)

    tangent = property(
        (lambda self: (lambda: self._tangent() / units.SYSTEM[FORCE])),
        (lambda self, value: setattr(self, '_tangent', (lambda: value() * units.SYSTEM[FORCE])))
    )

    torque = property(
        (lambda self: (lambda: self._torque() / units.SYSTEM[FORCE])),
        (lambda self, value: setattr(self, '_torque', (lambda: value() * units.SYSTEM[FORCE])))
    )
