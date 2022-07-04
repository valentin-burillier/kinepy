import numpy as np


class Solid:
    nb = 0
    
    def __init__(self, points=(), named_points=None, name=''):
        self.__class__.nb += 1
        self.points = list(points)
        self.name = name if name else f'Solid{Solid.nb}'
        self.named_points = named_points if named_points else dict()
        self.angle = self._points = self.origin = None
    
    def __setitem__(self, item, value):
        if isinstance(item, int):
            self.points[item] = value
        elif isinstance(item, str):
            if item not in self.named_points:
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
            return self.points[self.named_points[item]]
    
    def reset(self, n):
        self.origin = np.zeros((2, n), float)
        self.angle = np.zeros((n,), float)

    def _save(self):
        return {'name': self.name, 'points': self.points, 'named_points': self.named_points}
    
    @classmethod
    def _load(cls, data):
        return cls(**data)
