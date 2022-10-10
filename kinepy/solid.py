from kinepy.base_units import *
from kinepy.geometry import rvec

ZERO = np.zeros((2, 1))
ZERO_F = (lambda: 0)
ZERO_ARRAY_F = (lambda : ZERO)

class Solid(metaclass=MetaUnit):
    read_only = ('origin', LENGTH, None), ('angle', ANGLE, None)
    read_write = ('m', MASS, 0.), ('g', LENGTH, (0., 0.)), ('j', INERTIA, 0.)

    def __init__(self, unit_system, rep, m, j, g, name):
        self._unit_system, self.rep, self.name = unit_system, rep, name if name else f'Solid {rep}'
        self.j, self.m, self.g = j, m, g
        self.mech_actions, self.external_actions = [], []

    def __repr__(self):
        return f'{self.rep} | {self.name}'

    def reset(self, n):
        self.mech_actions = []
        self.angle_ = np.zeros((n,), float)
        self.origin_ = np.zeros((2, n), float)

    @physics_output(LENGTH)
    @physics_input(LENGTH)
    def get_point(self, p):
        return self.origin_ + rvec(self.angle_, p)

    @physics_input(FORCE, LENGTH)
    def add_force(self, f, p):
        if isinstance(f, np.ndarray) and len(f) == 2:
            if f.shape == (2,):
                f = f.reshape((2, 1))
            self.external_actions.append(((lambda: f), ZERO_F, p))
        elif isinstance(f, FUNCTION_TYPE):
            self.external_actions.append((f, ZERO_F, p))

    @physics_input(TORQUE)
    def add_torque(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.external_actions.append((ZERO_ARRAY_F, (lambda: t), self.g_))
        elif isinstance(t, FUNCTION_TYPE):
            self.external_actions.append((ZERO_ARRAY_F, t, self.g_))


class GhostSolid:
    m_, j_, g_ = 0., 0., (0., 0.)
    angle_, origin_ = None, None

    def __init__(self):
        self.mech_actions = []
        self.rep = None

    reset = Solid.reset
