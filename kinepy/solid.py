from kinepy.geometry import np, rot
from kinepy.interactions import ZERO


FUNCTION_TYPE = type(lambda: 0)


class Solid:
    def __init__(self, system, j=0., m=0., g=(0., 0.), name='', rep=0):
        self.system = system
        self.rep = rep
        self.name = name if name else f'Solid {self.rep}'
        self.angle_ = self.origin_ = None
        self.j_, self.m_, self.g_ = j * system.units['Inertia'], m * system.units['Mass'], np.array(g) * system.units['Length']

        self.mech_actions = []
        self.external_actions = []


    def get_point(self, point):
        point = np.array(point)
        if point.shape[0] != 2:
            raise ValueError('Shape must start with 2')
        (a00, a01), (a10, a11) = rot(self.angle_)
        return self.origin_ / self.system.units['Length'] + np.array((point[0] * a00 + point[1] * a01, point[0] * a10 + point[1] * a11))

    @property
    def origin(self):
        return self.origin_ / self.system.units['Length']

    @property
    def angle(self):
        return self.angle_ / self.system.units['Angle']

    @property
    def g(self):
        return self.g_ / self.system.units['Length']

    @g.setter
    def g(self, value):
        self.g_ = np.array(value) * self.system['Length']

    @property
    def m(self):
        return self.m_ / self.system.units['Mass']

    @m.setter
    def m(self, value):
        self.m_ = value * self.system['Mass']

    @property
    def j(self):
        return self.j_ / self.system.units['Interia']

    @j.setter
    def j(self, value):
        self.j_ = value * self.system['Inertia']

    def reset(self, n):
        self.mech_actions = []
        self.origin_ = np.zeros((2, n), float)
        self.angle_ = np.zeros((n,), float)
    
    def __repr__(self):
        return str(self.rep) + ' | ' + self.name

    def add_force(self, f, p):
        p = np.array(p) * self.system.units['Length']
        if isinstance(f, (tuple, list, np.ndarray)) and len(f) == 2:
            f = np.array(f) * self.system.units['Force']
            if f.shape == (2,):
                f = f.reshape((2, 1))
            self.external_actions.append(((lambda: f), (lambda: 0), p))
        elif isinstance(f, FUNCTION_TYPE):
            self.external_actions.append(((lambda: f() * self.system.units['Force']), (lambda: 0), p))

    def add_torque(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.external_actions.append(((lambda: ZERO), (lambda: t * self.system.units['Torque']), self.g_))
        elif isinstance(t, FUNCTION_TYPE):
            self.external_actions.append(((lambda: ZERO), (lambda: t() * self.system.units['Torque']), self.g_))
