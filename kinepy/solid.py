import numpy as np

from kinepy.geometry import *


class Solid:
    nb = 0
    
    def __init__(self, points=(), named_points=None, j=0., m=0., g=(0., 0.), name=''):
        self.points = list(points)
        self.name = name if name else f'Solid{Solid.nb}'
        self.__class__.nb += 1
        self.named_points = named_points if named_points else dict()
        self.angle = self._points = self.origin = None
        self.j, self.m, self.g = j, m, g
        self._ma = self._ja = self._og = self.__mat = None

    def __setitem__(self, item, value):
        if isinstance(item, int):
            # Référence par l'indice
            self.points[item] = value
        elif isinstance(item, str):
            # Référence par le nom
            if item not in self.named_points:
                # Nouveau point
                if item == 'G':
                    # 'G' est un nom dédié
                    if not isinstance(value, tuple):
                        raise TypeError('G is a point that is not related to others, expected tuple')
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
            # Référence par l'indice
            return self.points[item]
        elif isinstance(item, str):
            # Référence par le nom
            if item == 'G':
                return self.g
            return self.points[self.named_points[item]]

    def get_point(self, point):
        if isinstance(point, (str, int)):
            return self.origin + np.einsum('ikl,k->il', rot(self.angle), self[point])
        point = np.array(point)
        if point.shape[0] != 2:
            raise ValueError('Shape must start with 2')
        (a00, a01), (a10, a11) = rot(self.angle)
        return self.origin + np.array((point[0] * a00 + point[1] * a01, point[0] * a10, point[1] * a11))

    def reset(self, n):
        self.origin = np.zeros((2, n), float)
        self.angle = np.zeros((n,), float)

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

