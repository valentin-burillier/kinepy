from geometry import *


class Joint:
    id_ = 0

    def __init__(self, s1, s2, name):
        self.s1, self.s2, self.name = s1, s2, name
        self.system = None

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
    id_ = 1
    tag = 'P'
    
    def __init__(self, s1, s2, p1, p2):
        Joint.__init__(self, s1, s2, f'Rev({s2}/{s1})')
        self._p1, self._p2 = p1, p2
        self.angle = None
    
    def input_mode(self):
        return f'{self.name}, Angle',
    
    @property
    def identifier(self):
        return (self.s1, self._p1), (self.s2, self._p2)

    def get_data(self):
        return {
            's1': self.s1,
            's2': self.s2,
            'p1': self._p1,
            'p2': self._p2
        }
    
    def reset(self, n):
        self.angle = np.zeros((n,), float)

    @property
    def p1(self):
        if self.system is None:
            raise Exception(f"{self} doesn't belong in any system")
        print(f"{self.system.sols[self.s1].points[self._p1]} (index: {self._p1}) "
              f"in Solid {self.system.sols[self.s1].name} (index: {self.s1})")

    @p1.setter
    def p1(self, value):
        if self.system is None:
            raise Exception(f"{self} doesn't belong in any system")
        if isinstance(value, int) and 0 <= value < len(self.system.sols[self.s1].points):
            self._p1 = value
            self.system.sols[self.s1].named_points[self.name] = value
        elif isinstance(value, (tuple, list, np.ndarray)) and len(value) == 2:
            self.system.sols[self.s1].points[self._p1] = tuple(value)
        elif isinstance(value, str):
            self.system.sols[self.s1].named_points[self.name] = self._p1 = self.system.sols[self.s1].named_points[value]

    @property
    def p2(self):
        if self.system is None:
            raise Exception(f"{self} doesn't belong in any system")
        print(f"{self.system.sols[self.s2].points[self._p2]} (index: {self._p2}) "
              f"in Solid {self.system.sols[self.s2].name} (index: {self.s2})")

    @p2.setter
    def p2(self, value):
        if self.system is None:
            raise Exception(f"{self} doesn't belong in any system")
        if isinstance(value, int) and 0 <= value < len(self.system.sols[self.s2].points):
            self._p2 = value
            self.system.sols[self.s2].named_points[self.name] = value
        elif isinstance(value, (tuple, list, np.ndarray)) and len(value) == 2:
            self.system.sols[self.s2].points[self._p2] = tuple(value)
        elif isinstance(value, str):
            self.system.sols[self.s2].named_points[self.name] = self._p2 = self.system.sols[self.s2].named_points[value]

    def pilot(self, system, index):
        self.angle = system.input[index[0]]
        theta = self.angle + system.get_ref(self.s1) - system.get_ref(self.s2)
        change_ref(system, self.s2, theta, rot(theta),
                   get_point(system, self.s2, self._p2), get_point(system, self.s1, self._p1))
        return system.eqs[self.s1] + system.eqs[self.s2]


class PrismaticJoint(Joint):
    id_ = 4
    tag = 'G'

    def __init__(self, s1, s2, a1, d1, a2, d2):
        Joint.__init__(self, s1, s2, f'Pri({s2}/{s1}')
        self.a1, self.a2, self.d1, self.d2 = a1, a2, d1, d2
        self.delta = None
        
    def input_mode(self):
        return f'{self.name}, Length',
    
    @property
    def identifier(self):
        return (self.s1, self.a1, self.d1), (self.s2, self.a2, self.d2)

    def get_data(self):
        return {
            's1': self.s1,
            's2': self.s2,
            'a1': self.a1,
            'a2': self.a2,
            'd1': self.d1,
            'd2': self.d2,
            'name': self.name
        }
    
    def reset(self, n):
        self.delta = np.zeros((n,), float)

    def pilot(self, system, index):
        self.delta = system.input[index[0]]
        theta = (alpha := system.get_ref(self.s1) + self.a1) - system.get_ref(self.s2) - self.a2
        ux = unit(alpha)
        change_ref(system, self.s2, theta, rot(alpha), system.get_origin(self.s2),
                   system.get_origin(self.s1) + self.delta * ux + (self.d1 - self.d2) * z_cross(ux))
        return system.eqs[self.s1] + system.eqs[self.s2]


class PinSlotJoint(Joint):
    id_ = 16
    tag = 'SP'

    def __init__(self, s1, s2, a1, d1, p2):
        Joint.__init__(self, s1, s2, f'Sli({s2}/{s1})')
        self._p2, self.a1, self.d1 = p2, a1, d1
        self. delta = self.angle = None
    
    def input_mode(self):
        return f'{self.name}, Length', f'{self.name}, Angle'
    
    @property
    def identifier(self):
        return (self.s2, self._p2), (self.s1, self.a1, self.d1)

    def get_data(self):
        return {
            's1': self.s1,
            'p2': self._p2,
            's2': self.s2,
            'a1': self.a1,
            'd1': self.d1
        }

    def reset(self, n):
        self.delta = np.zeros((n,))
        self.angle = np.zeros((n,))

    @property
    def p2(self):
        if self.system is None:
            raise Exception(f"{self} doesn't belong in any system")
        print(f"{self.system.sols[self.s2].points[self._p2]} (index: {self._p2}) "
              f"in Solid {self.system.sols[self.s2].name} (index: {self.s2})")

    @p2.setter
    def p2(self, value):
        if self.system is None:
            raise Exception(f"{self} doesn't belong in any system")
        if isinstance(value, int) and 0 <= value < len(self.system.sols[self.s2].points):
            self._p2 = value
            self.system.sols[self.s2].named_points[self.name] = value
        elif isinstance(value, (tuple, list, np.ndarray)) and len(value) == 2:
            self.system.sols[self.s2].points[self._p2] = tuple(value)
        elif isinstance(value, str):
            self.system.sols[self.s2].named_points[self.name] = self._p2 = self.system.sols[self.s2].named_points[value]

    def pilot(self, system, index):
        self.delta, self.angle = system.input[index[0]], system.input[index[1]]
        theta = system.get_ref(self.s1) + self.angle - system.get_ref(self.s2)
        ux = unit(system.get_ref(self.s1) + self.a1)
        change_ref(system, self.s2, theta, rot(theta), get_point(system, self.s2, self._p2),
                   system.get_origin(self.s1) + self.delta * ux + self.d1 * z_cross(ux))
        return system.eqs[self.s1] + system.eqs[self.s2]


class RectangularJoint(Joint):
    id_ = 64
    tag = 'T'

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

    def pilot(self, system, index):
        self.delta = np.array((system.input[index[0]], system.input[index[1]]))
        theta = system.get_ref(self.s1) + self.angle - system.get_ref(self.s2)
        ux, uy = unit(system.get_ref(self.s1) + self.base[0]), unit(system.get_ref(self.s1) + self.base[1])
        change_ref(system, self.s1, theta, rot(theta), system.get_origin(self.s2),
                   system.get_origin(self.s1) + mat_mul_n(((ux[0], uy[0]), (ux[1], uy[1])), self.delta))
        return system.eqs[self.s1] + system.eqs[self.s2]
