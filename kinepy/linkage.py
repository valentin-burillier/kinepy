from kinepy.geometry import *
from kinepy.dynamic import trd, tmd, MechanicalAction, RevoluteTorque, PinSlotTangentTorque, PrismaticTangent


FUNCTION_TYPE = type(lambda: 0)


class Joint:
    id_ = -1

    def __init__(self, s1, s2, name):
        self.s1, self.s2, self.name = s1, s2, name
        self.system = None

    def __repr__(self):
        return self.name
    
    @classmethod
    def load(cls, data):
        return cls(**data)

    def get_data(self):
        pass

    def save(self):
        return self.__class__.__name__, self.get_data()

    def input_mode(self):
        return ()


class RevoluteJoint(Joint):
    id_ = 0
    tag = 'P'
    
    def __init__(self, s1, s2, p1, p2):
        Joint.__init__(self, s1, s2, f'Rev({s2}/{s1})')
        self.p1, self.p2 = p1, p2
        self.angle = None
        self.interaction = RevoluteTorque(self, lambda: 0.)

    def input_mode(self):
        return f'{self.name}: Angle',
    
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

    def pilot(self, system, index):
        self.angle = system.input[index[0]]
        theta = self.angle + system.get_ref(self.s1) - system.get_ref(self.s2)
        change_ref(system, self.s2, theta, rot(theta),
                   get_point(system, self.s2, self.p2), get_point(system, self.s1, self.p1))
        return system.eqs[self.s1] + system.eqs[self.s2]

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.point
        f = trd(system, eq2)
        m = tmd(system, p, eq2)
        system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))

    @property
    def point(self):
        return get_point(self.system, self.s1, self.p1)

    def set_torque(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.t = lambda: t
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.t = t
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')


class PrismaticJoint(Joint):
    id_ = 1
    tag = 'G'

    def __init__(self, s1, s2, a1, d1, a2, d2):
        Joint.__init__(self, s1, s2, f'Pri({s2}/{s1})')
        self.a1, self.a2, self.d1, self.d2 = a1, a2, d1, d2
        self.delta = None
        self.interaction = RevoluteTorque(self, lambda: 0.)
        
    def input_mode(self):
        return f'{self.name}: Length',
    
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

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = system.get_origin(self.s1) + self.d1 * unit(system.get_ref(self.s1) + self.a1 + np.pi * .5)
        f = trd(system, eq2)
        m = tmd(system, p, eq2)

        system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))

    def set_tangent(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.f = lambda: t
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.f = t
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')


class PinSlotJoint(Joint):
    id_ = 2
    tag = 'SP'

    def __init__(self, s1, s2, a1, d1, p2):
        Joint.__init__(self, s1, s2, f'Pin({s2}/{s1})')
        self.p2, self.a1, self.d1 = p2, a1, d1
        self. delta = self.angle = None
        self.interaction = PinSlotTangentTorque(self, (lambda: 0), lambda: 0)
    
    def input_mode(self):
        return f'{self.name}: Length', f'{self.name}: Angle'
    
    @property
    def identifier(self):
        return (self.s2, self.p2), (self.s1, self.a1, self.d1)

    def get_data(self):
        return {
            's1': self.s1,
            'p2': self.p2,
            's2': self.s2,
            'a1': self.a1,
            'd1': self.d1
        }

    def reset(self, n):
        self.delta = np.zeros((n,))
        self.angle = np.zeros((n,))

    def pilot(self, system, index):
        self.delta, self.angle = system.input[index[0]], system.input[index[1]]
        theta = system.get_ref(self.s1) + self.angle - system.get_ref(self.s2)
        ux = unit(system.get_ref(self.s1) + self.a1)
        change_ref(system, self.s2, theta, rot(theta), get_point(system, self.s2, self.p2),
                   system.get_origin(self.s1) + self.delta * ux + self.d1 * z_cross(ux))
        return system.eqs[self.s1] + system.eqs[self.s2]

    @property
    def point(self):
        return get_point(self.system, self.s2, self.p2)

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.point
        f = trd(system, eq2)
        m = tmd(system, p, eq2)

        system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))

    def set_torque(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.t = lambda: t
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.t = t
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')

    def set_tangent(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.f = lambda: t
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.f = t
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')


class RectangularJoint(Joint):
    id_ = 3
    tag = 'T'

    def __init__(self, s1, s2, angle, base):
        Joint.__init__(self, s1, s2, f'Rect({s2}/{s1})')
        self.angle, self.base = angle, base
        self.delta = None

    def input_mode(self):
        return f'{self.name}: X', f'{self.name}: Y'
        
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

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = system.get_origin(self.s1)
        f = trd(system, eq2)
        m = tmd(system, p, eq2)

        system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))


class ThreeDegreesOfFreedomJoint(Joint):
    id_ = 4
    tag = '3DoF'

    def __init__(self, s1, s2):
        Joint.__init__(self, s1, s2, f'3DoF({s2}/{s1})')
        self.delta, self.angle = None, None

    def get_data(self):
        return {'s1': self.s1, 's2': self.s2, 'name': self.name}

    def input_mode(self):
        return f'{self.name}: X', f'{self.name}: Y', f'{self.name}: Angle'

    @property
    def identifier(self):  # Ne devrait pas servir...
        return (self.s1,), (self.s2,)

    def reset(self, n):
        self.delta = np.zeros((2, n), float)
        self.angle = np.zeros((n,), float)

    def pilot(self, system, index):
        self.delta[0], self.delta[1] = system.input[index[0]], system.input[index[1]]
        self.angle = system.input[index[2]]
        theta = system.get_ref(self.s1) + self.angle - system.get_ref(self.s2)
        change_ref(system, self.s2, theta, rot(theta), system.get_origin(self.s2), system.get_origin(self.s1))
        return system.eqs[self.s1] + system.eqs[self.s2]

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = system.get_origin(self.s1)
        f = trd(system, eq2)
        m = tmd(system, p, eq2)

        system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))


class_dict = {cls.__name__: cls for cls in Joint.__subclasses__()}
