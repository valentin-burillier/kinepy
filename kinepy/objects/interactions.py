from kinepy.math.geometry import np, rvec, sq_mag


class Gravity:
    name = "Gravity"

    def __init__(self, g):
        self.g = g

    def set_ma(self, system):
        g = np.reshape(np.array(self.g), (2, 1))
        for s in system.sols:
            s.add_mech_action(g * s.m, s.og, 0.)


class Spring:
    force = None

    def __init__(self, k, l0, s1, s2, p1, p2):
        self.s1, self.s2, self.p1, self.p2 = s1, s2, p1, p2
        self.l0, self.k = l0, k

    def set_ma(self, _):
        a, b = self.s1.origin + rvec(self.s2.angle, self.p2), self.s2.origin + rvec(self.s2.angle, self.p2)
        ab = b - a
        l_ = sq_mag(ab) ** .5
        f = ab * (f_ := self.k * (1 - self.l0 / l_))
        self.s1.add_mech_action(f, a, 0.)
        self.s2.add_mech_action(-f, b, 0.)
        self.force = f_ * l_
