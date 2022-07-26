from kinepy.geometry import *
from kinepy.linkage import RevoluteJoint, PinSlotJoint


class Solid:
    def __init__(self, j=0., m=0., g=(0., 0.), name='', rep=0):
        self.rep = rep
        self.name = name if name else f'Solid {self.rep}'
        self.angle = self._points = self.origin = None
        self.j, self.m, self.g = j, m, g

        self.mech_actions = {}

    def get_point(self, point):
        if isinstance(point, (RevoluteJoint, PinSlotJoint)):
            return point.point
        point = np.array(point)
        if point.shape[0] != 2:
            raise ValueError('Shape must start with 2')
        (a00, a01), (a10, a11) = rot(self.angle)
        return self.origin + np.array((point[0] * a00 + point[1] * a01, point[0] * a10 + point[1] * a11))

    def reset(self, n):
        self.mech_actions = {}
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
