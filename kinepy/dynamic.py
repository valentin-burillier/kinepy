from kinepy.geometry import *


class MechanicalAction:
    def __init__(self, f: np.ndarray, p: np.ndarray, m):
        self.f, self.p, self.m = f, p, m


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
