from kinepy.geometry import *

ZERO = np.array([[0.], [0.]])


class MechanicalAction:
    def __init__(self, f, p, m):
        self.f, self.p, self.m = f, p, m

    def babar(self, point):
        return self.m + det(self.p - point, self.f)


class Interaction:
    name: str

    def set_ma(self, system):
        pass


class Gravity(Interaction):
    def __init__(self, g):
        self.g = g

    def set_ma(self, system):
        g = np.reshape(np.array(self.g), (2, 1))
        for s in system.sols:
            s.mech_actions.append(MechanicalAction(g * s.m_, system.get_point(s.rep, s.g_), 0.))


class Spring(Interaction):
    def __init__(self, k, l0, s1, s2, p1, p2):
        self.s1, self.s2, self.p1, self.p2 = s1, s2, p1, p2
        self.l0, self.k = l0, k
        self.f = None

    def set_ma(self, system):
        a, b = system.get_point(self.s1, self.p1), system.get_point(self.s2, self.p2)
        ab = b - a
        l_ = mag(ab)
        f = ab * (f_ := self.k * (1 - self.l0 / l_))
        system.sols[self.s1].mech_actions.append(MechanicalAction(f, a, 0.))
        system.sols[self.s2].mech_actions.append(MechanicalAction(-f, b, 0.))
        self.f = f_ * l_


class RevoluteTorque(Interaction):
    def __init__(self, rev, t):
        self.rev, self.t = rev, t

    def set_ma(self, system):
        t = self.t()
        system.sols[self.rev.s1].mech_actions.append(MechanicalAction(ZERO, 0., t))
        system.sols[self.rev.s2].mech_actions.append(MechanicalAction(ZERO, 0., -t))


class PrismaticTangent(Interaction):
    def __init__(self, pri, f):
        self.pri, self.f = pri, f

    def set_ma(self, system):
        u = unit(system.get_ref(self.pri.s1) + self.pri.a1)
        f = self.f() * u
        p = system.get_origin(self.pri.s1) + (self.pri.d1 - self.pri.d2) * z_cross(u)
        system.sols[self.pri.s1].mech_actions.append(MechanicalAction(f, p, 0.))
        system.sols[self.pri.s2].mech_actions.append(MechanicalAction(-f, p, 0.))


class PinSlotTangentTorque(Interaction):
    def __init__(self, pin, f, t):
        self.pin, self.f, self.t = pin, f, t

    def set_ma(self, system):
        u = unit(system.get_ref(self.pin.s1) + self.pin.a1)
        f, t = self.f() * u, self.t()
        p = self.pin.point
        system.sols[self.pin.s1].mech_actions.append(MechanicalAction(f, p, t))
        system.sols[self.pin.s2].mech_actions.append(MechanicalAction(-f, p, -t))
