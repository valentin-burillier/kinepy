from kinepy.geometry import *
from kinepy.linkage import *

class Solid:
    def __init__(self, points=(), named_points=None, j=0., m=0., g=(0., 0.), name='', rep=0):
        self.points = list(points)
        self.rep = rep
        self.name = name if name else f'Solid {self.rep}'
        self.named_points = named_points if named_points else dict()
        self.angle = self._points = self.origin = None
        self.j, self.m, self.g = j, m, g

        self.mech_actions = {}

    def __setitem__(self, item, value):
        if isinstance(item, int):
            # Référence par l'indice
            self.points[item] = value
        elif isinstance(item, str):
            # Référence par le nom
            if item not in self.named_points:
                # Nouveau point
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
        if isinstance(point, (RevoluteJoint, PinSlotJoint)):
            point = point.__repr__()
        if isinstance(point, (str, int)):
            return self.origin + np.einsum('ikl,k->il', rot(self.angle), self[point])
        point = np.array(point)
        if point.shape[0] != 2:
            raise ValueError('Shape must start with 2')
        (a00, a01), (a10, a11) = rot(self.angle)
        return self.origin + np.array((point[0] * a00 + point[1] * a01, point[0] * a10, point[1] * a11))

    def add_point(self, point):
        self.points.append(point)

    def reset(self, n):
        self.mech_actions = {}
        self.origin = np.zeros((2, n), float)
        self.angle = np.zeros((n,), float)

    def save(self):
        return {
            'name': self.name,
            'points': self.points,
            'named_points': self.named_points,
            'j': self.j,
            'm': self.m,
            'g': self.g
        }
    
    @classmethod
    def load(cls, data):
        return cls(**data)
    
    def __repr__(self):
        return str(self.rep) + ' | ' + self.name
