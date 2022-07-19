import numpy as np


class Joint:
    nb = 0

    def __init__(self, s1, s2, name):
        self.__class__.nb += 1
        self.s1, self.s2, self.name = s1, s2, name

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
    
    def __init__(self, s1, s2, p1, p2):
        Joint.__init__(self, s1, s2, f'Rev({s2}/{s1})')
        self.p1, self.p2 = p1, p2
        self.angle = None
    
    def input_mode(self):
        return f'{self.name}, Angle',
    
    @property
    def identifier(self):
        return (self.s1, self.p1), (self.s2, self.p2)

    def get_data(self):
        return {
            's1': self.s1,
            's2': self.s2,
            'p1': self.p1,
            'p2': self.p2
        }
    
    def reset(self, n):
        self.angle = np.zeros((n,), float)


class PrismaticJoint(Joint):
    def __init__(self, s1, s2, alpha1, d1, alpha2, d2):
        Joint.__init__(self, s1, s2, f'Pri({s2}/{s1}')
        self.alpha1, self.alpha2, self.d1, self.d2 = alpha1, alpha2, d1, d2
        self.delta = None
        
    def input_mode(self):
        return f'{self.name}, Length',
    
    @property
    def identifier(self):
        return (self.s1, self.alpha1, self.d1), (self.s2, self.alpha2, self.d2)

    def get_data(self):
        return {
            's1': self.s1,
            's2': self.s2,
            'alpha1': self.alpha1,
            'alpha2': self.alpha2,
            'd1': self.d1,
            'd2': self.d2,
            'name': self.name
        }
    
    def reset(self, n):
        self.delta = np.zeros((n,), float)


class SlideCurveJoint(Joint):
    def __init__(self, s1, s2, alpha1, d1, p2):
        Joint.__init__(self, s1, s2, f'Sli({s2}/{s1})')
        self.p2, self.alpha1, self.d1 = p2, alpha1, d1
        self. delta = self.angle = None
    
    def input_mode(self):
        return f'{self.name}, Length', f'{self.name}, Angle'
    
    @property
    def identifier(self):
        return (self.s1, self.p2), (self.s2, self.alpha1, self.d1)

    def get_data(self):
        return {
            's1': self.s1,
            'p2': self.p2,
            's2': self.s2,
            'alpha1': self.alpha1,
            'd1': self.d1
        }

    def reset(self, n):
        self.delta = np.zeros((n,))
        self.angle = np.zeros((n,))
        
    
class DoublePrismaticJoint(Joint):
    def __init__(self, s1, s2, angle, base):
        Joint.__init__(self, s1, s2, f'2-Pri({s2}/{s1})')
        self.angle, self.base = angle, base
        self.delta = None

    def input_mode(self):
        return f'{self.name}, X', f'{self.name}, Y'
        
    @property
    def identifier(self):
        return (self.s1, self.angle), (self.s2, -self.angle)

    def get_data(self):
        return {
            's1': self.s1,
            's2': self.s2,
            'base': self.base,
            'angle': self.angle
        }
    
    def reset(self, n):
        self.delta = np.zeros((2, n), float)
