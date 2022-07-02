import numpy as np


class Solid:
    nb = 0
    def __new__(cls, *args, **kwargs):
        cls.nb += 1
        return object.__new__(cls, *args, **args)
    
    def __init__(self, points=(), named_points=None, name=''):
        self.points = points
        self._points = None
        self.name = name if name else f'Solid{Solid.nb}'
        self.named_points = named_points if named_points else dict()
        self.angle = None
    
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
        points = np.swapaxes(np.array([(0., 0.)] + self.points), 0, 1)
        m = points.shape(1)
        self._points = np.reshape(points, (2, m, n), 'F')
        self.angle = np.zeros((n,), float)

    def _save(self):
        return {'name': self.name, 'points': self.points, 'named_points': self.named_points}
    
    @classmethod
    def _load(cls, data):
        return cls(**data)

            