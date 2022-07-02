import numpy as np


class Linkage:
    nb = 0

    def __new__(cls, *args, **kwargs):
        cls.nb += 1
        return object.__new__(cls, *args, **kwargs)
    
    def __repr__(self):
        return f'{self.__class__.__name__}{self.sol2}/{self.sol1}, name: {self.name}'
    
    @classmethod
    def _load(cls, data):
        return cls(**data)

    def _save(self):
        return {self.__class__.__name__: self.getData()}


class Pivot(Linkage):
    
    def __init__(self, sol1, sol2, p1, p2, name=''):
        self.sol1, self.sol2, self.p1, self.p2 = sol1, sol2, p1, p2
        self.name = name if name else f'P{Pivot.nb}'
        self.angle = None
    
    def inputMode(self):
        return f'{self.name}, Angle',
    
    @property
    def identifier(self):
        return (self.sol1, self.p1), (self.sol2, self.p2)

    def getData(self):
        return {'sol1': self.sol1, 'sol2': self.sol2, 'p1': self.p1, 'p2': self.p2, 'name': self.name}
    
    def reset(self, n):
        self.angle = np.zeros((n,), float)


class Glissiere(Linkage):
    def __init__(self, sol1, sol2, alpha1, d1, alpha2, d2, name=''):
        self.sol1, self.sol2, self.alpha1, self.alpha2, self.d1, self.d2 = sol1, sol2, alpha1, alpha2, d1, d2
        self.name = name if name else f'G{Glissiere.nb}'
        self.delta = None
        
    def inputMode(self):
        return f'{self.name}, Length',
    
    @property
    def identifier(self):
        return (self.sol1, self.alpha1, self.d1), (self.sol2, self.alpha2, self.d2)

    def getData(self):
        return {'sol1': self.sol1, 'sol2': self.sol2, 'alpha1': self.alpha1, 'alpha2': self.alpha2, 'd1': self.d1, 'd2': self.d2, 'name': self.name}
    
    def reset(self, n):
        self.delta = np.zeros((n,), float)


class SpherePlan(Linkage):
    def __init__(self, sol1, sol2, p, alpha, d, name=''):
        self.sol1, self.sol2, self.p, self.alpha, self.d = sol1, sol2, p, alpha, d
        self.name = name if name else f'SP{SpherePlan.nb}'
        self. delta = self.angle = None
    
    def inputMode(self):
        return f'{self.name}, Length', f'{self.name}, Angle'
    
    @property
    def identifier(self):
        return (self.sol1, self.p), (self.sol2, self.alpha, self.d)

    def getData(self):
        return {'sol1': self.sol1, 'p': self.p, 'sol2': self.sol2, 'alpha': self.alpha, 'd': self.d, 'name': self.name}
        
    
class Translation(Linkage):
    def __init__(self, sol1, sol2, angle, base, name=''):
        self.sol1, self.sol2, self.angle, self.base = sol1, sol2, angle, base
        self.name = name if name else f'T{Translation.nb}'
        self.delta = None

    def inputMode(self):
        return f'{self.name}, X', f'{self.name}, Y'
        
    @property
    def identifier(self):
        return (self.sol1, self.angle), (self.sol2, -self.angle)

    def getData(self):
        return {'sol1': self.sol1, 'sol2': self.sol2, 'base': self.base, 'angle': self.angle, 'name': self.name}
    
    def reset(self, n):
        self.delta = np.zeros((2, n), float)
