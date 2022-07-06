import numpy as np


class Joint:
    nb = 0

    def __init__(self, sol1, sol2, name):
        self.__class__.nb += 1
        self.sol1, self.sol2, self.name = sol1, sol2, name

    def __repr__(self):
        return self.name
    
    @classmethod
    def _load(cls, data):
        return cls(**data)

    def get_data(self):
        pass

    def _save(self):
        return {self.__class__.__name__: self.get_data()}


class RevoluteJoint(Joint):
    
    def __init__(self, sol1, sol2, p1, p2):
        Joint.__init__(self, sol1, sol2, f'Rev({sol2}/{sol1})')
        self.p1, self.p2 = p1, p2
        self.angle = None
    
    def input_mode(self):
        return f'{self.name}, Angle',
    
    @property
    def identifier(self):
        return (self.sol1, self.p1), (self.sol2, self.p2)

    def get_data(self):
        return {
            'sol1': self.sol1,
            'sol2': self.sol2,
            'p1': self.p1,
            'p2': self.p2
        }
    
    def reset(self, n):
        self.angle = np.zeros((n,), float)


class PrismaticJoint(Joint):
    def __init__(self, sol1, sol2, alpha1, d1, alpha2, d2):
        Joint.__init__(self, sol1, sol2, f'Pri({sol2}/{sol1}')
        self.alpha1, self.alpha2, self.d1, self.d2 = alpha1, alpha2, d1, d2
        self.delta = None
        
    def input_mode(self):
        return f'{self.name}, Length',
    
    @property
    def identifier(self):
        return (self.sol1, self.alpha1, self.d1), (self.sol2, self.alpha2, self.d2)

    def get_data(self):
        return {
            'sol1': self.sol1,
            'sol2': self.sol2,
            'alpha1': self.alpha1,
            'alpha2': self.alpha2,
            'd1': self.d1,
            'd2': self.d2,
            'name': self.name
        }
    
    def reset(self, n):
        self.delta = np.zeros((n,), float)


class SlideCurveJoint(Joint):
    def __init__(self, sol1, sol2, p, alpha, d):
        Joint.__init__(self, sol1, sol2, f'Sli({sol2}/{sol1})')
        self.p, self.alpha, self.d = p, alpha, d
        self. delta = self.angle = None
    
    def input_mode(self):
        return f'{self.name}, Length', f'{self.name}, Angle'
    
    @property
    def identifier(self):
        return (self.sol1, self.p), (self.sol2, self.alpha, self.d)

    def get_data(self):
        return {
            'sol1': self.sol1,
            'p': self.p,
            'sol2': self.sol2,
            'alpha': self.alpha,
            'd': self.d
        }

    def reset(self, n):
        self.delta = np.zeros((n,))
        self.angle = np.zeros((n,))
        
    
class DoublePrismaticJoint(Joint):
    def __init__(self, sol1, sol2, angle, base):
        Joint.__init__(self, sol1, sol2, f'2-Pri({sol2}/{sol1})')
        self.angle, self.base = angle, base
        self.delta = None

    def input_mode(self):
        return f'{self.name}, X', f'{self.name}, Y'
        
    @property
    def identifier(self):
        return (self.sol1, self.angle), (self.sol2, -self.angle)

    def get_data(self):
        return {
            'sol1': self.sol1,
            'sol2': self.sol2,
            'base': self.base,
            'angle': self.angle
        }
    
    def reset(self, n):
        self.delta = np.zeros((2, n), float)
