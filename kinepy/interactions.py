from kinepy.base_units import *
from kinepy.solid import ZERO, ZERO_F
from kinepy.geometry import det, get_g, rvec, sq_mag, unit, get_angle, get_zero


class MechanicalAction:
    def __init__(self, f, p, m):
        self.f, self.p, self.m = f, p, m

    def babar(self, point):
        return self.m + det(self.p - point, self.f)


class Interaction(metaclass=MetaUnit):
    read_only = read_write = ()

    def __init__(self, unit_system):
        self._unit_system = unit_system

    def set_ma(self):
        pass


class Gravity(Interaction):
    read_write = ('g', ACCELERATION, (0., -G)),

    def __init__(self, unit_system, system, g):
        Interaction.__init__(self, unit_system)
        self.g, self.system = g, system

    def set_ma(self):
        g = np.reshape(np.array(self.g), (2, 1))
        for s in self.system.sols:
            s.mech_actions.append(MechanicalAction(g * s.m_, get_g(s), 0.))


class Spring(Interaction):
    read_only = FORCE_,
    read_write = P1, P2

    def __init__(self, unit_system, k, l0, s1, s2, p1, p2):
        Interaction.__init__(self, unit_system)
        self.s1, self.s2, self.p1, self.p2 = s1, s2, p1, p2
        self.l0, self.k = l0, k

    def set_ma(self):
        a, b = self.s1.origin_ + rvec(self.s2.angle_, self.p2_), self.s2.origin_ + rvec(self.s2.angle_, self.p2_)
        ab = b - a
        l_ = sq_mag(ab) ** .5
        f = ab * (f_ := self.k * (1 - self.l0 / l_))
        self.s1.mech_actions.append(MechanicalAction(f, a, 0.))
        self.s2.mech_actions.append(MechanicalAction(-f, b, 0.))
        self.force_ = f_ * l_


class RevoluteTorque(Interaction):
    read_write = ('torque', TORQUE, ZERO_F),

    def __init__(self, unit_system, rev, t):
        Interaction.__init__(self, unit_system)
        self.rev, self.torque = rev, t

    def set_ma(self):
        t = self.t()
        self.rev.s1.mech_actions.append(MechanicalAction(ZERO, 0., t))
        self.rev.s2.mech_actions.append(MechanicalAction(ZERO, 0., -t))


class PrismaticTangent(Interaction):
    read_write = ('tangent', FORCE, ZERO_F),

    def __init__(self, unit_system, pri, f):
        Interaction.__init__(self, unit_system)
        self.pri, self.f = pri, f

    def set_ma(self):
        u = unit(get_angle(self.pri, 0))
        f = self.f() * u
        p = get_zero(self.pri, 0, u)
        self.pri.s1.mech_actions.append(MechanicalAction(f, p, 0.))
        self.pri.s2.mech_actions.append(MechanicalAction(-f, p, 0.))


class PinSlotTangentTorque(Interaction):
    read_write = ('tangent', FORCE, ZERO_F), ('torque', TORQUE, ZERO_F)

    def __init__(self, unit_system, pin, f, t):
        Interaction.__init__(self, unit_system)
        self.pin, self.tangent, self.torque = pin, f, t

    def set_ma(self):
        u = unit(self.pin.s1.angle_ + self.pin.a1_)
        f, t = self.f() * u, self.t()
        p = self.pin.s2.origin_ + rvec(self.pin.s2.angle_, self.pin.p2_)
        self.pin.s1.mech_actions.append(MechanicalAction(f, p, t))
        self.pin.s2.mech_actions.append(MechanicalAction(-f, p, -t))
