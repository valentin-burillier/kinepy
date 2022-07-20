import numpy as np
from kinepy.geometry import *


class Solid:
    nb = 0
    
    def __init__(self, points=(), named_points=None, j=0., m=0., g=(0., 0.), name=''):
        self.__class__.nb += 1
        self.points = list(points)
        self.name = name if name else f'Solid{Solid.nb}'
        self.named_points = named_points if named_points else dict()
        self.angle = self._points = self.origin = None
        self.j, self.m, self.g = j, m, g
        self._ma = self._ja = self._og = None

    def __setitem__(self, item, value):
        if isinstance(item, int):
            self.points[item] = value
        elif isinstance(item, str):
            if item not in self.named_points:
                if item == 'G':
                    if not isinstance(value, tuple):
                        raise TypeError('G is point that is not related to others, expected tuple')
                    self.g = value
                else:
                    self.named_points[item] = len(self.points)
                    self.points.append(value)
            else:
                self.points[self.named_points[item]] = value
        else:
            raise TypeError('invalid argument')
    
    def __getitem__(self, item):
        if isinstance(item, int):
            return self.points[item]
        elif isinstance(item, str):
            if item == 'G':
                return self.g
            return self.points[self.named_points[item]]
    
    def reset(self, n):
        self.origin = np.zeros((2, n), float)
        self.angle = np.zeros((n,), float)
        self._ma = np.zeros((2, n - 2), float)
        self._ja = np.zeros((n - 2,), float)

    def _save(self):
        return {
            'name': self.name,
            'points': self.points,
            'named_points': self.named_points,
            'j': self.j,
            'm': self.m,
            'g': self.g
        }
    
    @classmethod
    def _load(cls, data):
        return cls(**data)

    def compute_ma(self, dt):
        self._og = self.origin + mat_mul_n(rot(self.angle), self.g)
        self._ma = np.array((derivative2(self._og[0], dt), derivative2(self._og[1], dt))) * self.m

    def compute_ja(self, dt):
        self._ja = self.j * derivative2(self.angle, dt)

    def get_ja(self, point):
        #  BABAR
        return self._ja + det(self._og - point, self._ma)

    def get_ma(self):
        return self._ma
