from kinepy.math.geometry import np, det, rvec


class Solid:
    rep = angle = origin = trd = g_tmd = og = None

    def __init__(self,  m, j, g, name):
        self.name = name
        self.j, self.m, self.g = j, m, g

    def reset(self, n):
        self.trd = np.zeros((2, n), float)
        self.g_tmd = np.zeros((n,), float)
        self.angle = np.zeros((n,), float)
        self.origin = np.zeros((2, n), float)

    def dyn_reset(self):
        self.trd *= 0
        self.g_tmd *= 0
        self.og = self.origin + rvec(self.angle, self.g)

    def add_mech_action(self, force, point, torque):
        self.trd += force
        self.g_tmd += torque + det(point - self.og, force)

    def babar(self, point):
        return self.g_tmd + det(self.og - point, self.trd)

    def __repr__(self):
        return self.name
