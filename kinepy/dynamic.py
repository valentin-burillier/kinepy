import numpy as np

from kinepy.geometry import *


class MechanicalAction:
    def __init__(self, f: np.ndarray, p: np.ndarray, m):
        self.f, self.p, self.m = f, p, m

    def babar(self, point):
        return self.m + det(self.p - point, self.f)


class Interaction:
    name: str

    def set_am(self, system):
        pass


class AccelerationField(Interaction):
    def __init__(self, g, name):
        self.g, self.name = g, name

    def set_am(self, system):
        for s in system.sols:
            s.mech_actions[self.name] = MechanicalAction(self.g * s.m, s.get_point(s.g), 0.)


class Spring(Interaction):
    def __init__(self, s1, s2, p1, p2, l0, k):
        self.s1, self.s2, self.p1, self.p2 = s1, s2, p1, p2
        self.l0, self.k = l0, k
        self.name = f'Spring({s2}/{s1})'

    def set_am(self, system):
        a, b = get_point(system, self.s1, self.p1), get_point(system, self.s2, self.p2)
        ab = b - a
        l_ = mag(ab)
        system.sols[self.s1].mech_actions[self.name] = MechanicalAction(ab * self.k * (1 - self.l0 / l_), a, 0.)
        system.sols[self.s2].mech_actions[self.name] = MechanicalAction(ab * self.k * (self.l0 / l_ - 1), b, 0.)


def tmd(system, point, eq) -> np.ndarray:
    return sum((sum(am.babar(point) for am in system.sols[s].mech_actions.values()) for s in eq), np.array((0., 0.)))


def trd(system, eq) -> np.ndarray:
    return sum((sum(am.f for am in system.sols[s].mech_actions.values()) for s in eq), np.array((0., 0.)))


def p_p_p(system, cycle, rev1, rev2, rev3, eq1, eq2, eq3):
    (s13, _), (s11, p11) = rev1
    (s21, _), (s22, p22) = rev2
    (s32, _), (s33, p33) = rev3
    p1, p2, p3 = get_point(system, s11, p11), get_point(system, s22, p22), get_point(system, s33, p33)
    p1p2, p3p2 = p2 - p1, p2 - p3

    inv_12, inv_32 = inv_mag(p1p2), inv_mag(p3p2)
    u1, u2 = p1p2 * inv_12, p3p2 * inv_32

    x1, x2 = -tmd(system, p1, eq2 + eq3) * inv_12, -tmd(system, p2, eq2) * inv_32
    y1 = (x2 - dot(u1, u2) * x1) / det(u1, u2)

    f2 = x1 * z_cross(u1) - y1 * u1
    f3 = -trd(system, eq2) - f2
    f1 = -trd(system, eq3) + f3

    n1 = system.joints[cycle[0]].name
    system.sols[s11].mech_actions[n1] = MechanicalAction(-f1, p1, 0.)
    system.sols[s13].mech_actions[n1] = MechanicalAction(f1, p1, 0.)

    n2 = system.joints[cycle[1]].name
    system.sols[s21].mech_actions[n2] = MechanicalAction(-f2, p1, 0.)
    system.sols[s22].mech_actions[n2] = MechanicalAction(f2, p1, 0.)

    n3 = system.joints[cycle[2]].name
    system.sols[s33].mech_actions[n3] = MechanicalAction(-f3, p1, 0.)
    system.sols[s32].mech_actions[n3] = MechanicalAction(f3, p1, 0.)
