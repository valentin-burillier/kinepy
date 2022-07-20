import numpy as np


class Solid:
    nb = 0
    
    def __init__(self, points=(), named_points=None, j=0., m=0., g=(0., 0.), name=''):
        self.__class__.nb += 1
        self.points = list(points)
        self.name = name if name else f'Solid{Solid.nb}'
        self.named_points = named_points if named_points else dict()
        self.angle = self._points = self.origin = None
        self.j, self.m, self.g = j, m, g
    
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

    def _save(self):
        return {'name': self.name, 'points': self.points, 'named_points': self.named_points, 'j': self.j, 'm': self.m, 'g': self.g}
    
    @classmethod
    def _load(cls, data):
        return cls(**data)
