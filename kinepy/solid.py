from kinepy.geometry import *


FUNCTION_TYPE = type(lambda: 0)


class Solid:
    def __init__(self, j=0., m=0., g=(0., 0.), name='', rep=0):
        self.rep = rep
        self.name = name if name else f'Solid {self.rep}'
        self.angle = self._points = self.origin = None
        self.j, self.m, self.g = j, m, g

        self.mech_actions = []
        self.external_actions = []

    def get_point(self, point):
        point = np.array(point)
        if point.shape[0] != 2:
            raise ValueError('Shape must start with 2')
        (a00, a01), (a10, a11) = rot(self.angle)
        return self.origin + np.array((point[0] * a00 + point[1] * a01, point[0] * a10 + point[1] * a11))

    def reset(self, n):
        self.mech_actions = []
        self.origin = np.zeros((2, n), float)
        self.angle = np.zeros((n,), float)

    def save(self):
        return {
            'name': self.name,
            'j': self.j,
            'm': self.m,
            'g': self.g
        }
    
    @classmethod
    def load(cls, data):
        return cls(**data)
    
    def __repr__(self):
        return str(self.rep) + ' | ' + self.name

    def add_force(self, f, p):
        if isinstance(f, (tuple, list, np.ndarray)) and len(f) == 2:
            self.external_actions.append(((lambda: f), (lambda: 0), p))
        elif isinstance(f, FUNCTION_TYPE):
            self.external_actions.append((f, (lambda: 0), p))

    def add_torque(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.external_actions.append(((lambda: 0), (lambda: t), self.g))
        elif isinstance(t, FUNCTION_TYPE):
            self.external_actions.append(((lambda: 0), t, self.g))
