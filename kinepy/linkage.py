from kinepy.geometry import np, rot, change_ref, mat_mul_n2, z_cross, unit, mat_mul_n
from kinepy.interactions import MechanicalAction, RevoluteTorque, PinSlotTangentTorque, PrismaticTangent
from kinepy.dynamic import tmd, trd


FUNCTION_TYPE = type(lambda: 0)


class Joint:
    id_ = -1
    dof = None

    def __init__(self, s1, s2, name):
        self.s1, self.s2, self.name = s1, s2, name
        self.system, self.interaction = None, None

    def __repr__(self):
        return self.name

    def input_mode(self):
        return ()

    def __get_angle__(self, index):
        pass

    def __get_unit__(self, index):
        pass

    def __get_point__(self, index):
        pass

    def __get_dist__(self, index):
        pass


class RevoluteJoint(Joint):
    id_ = 0
    tag = 'P'
    dof = 1
    
    def __init__(self, s1, s2, p1, p2):
        Joint.__init__(self, s1, s2, f'Rev({s2}/{s1})')
        self.p1, self.p2 = p1, p2
        self.angle = None
        self.interaction = RevoluteTorque(self, lambda: 0.)
        self.force, self.torque = None, None
        self.input_torque = None

    def input_mode(self):
        return f'{self.name}: Angle',
    
    @property
    def identifier(self):
        return (self.s1, 0), (self.s2, 1)

    def reset(self, n):
        self.force, self.torque = None, None
        self.input_torque = None
        self.angle = np.zeros((n,), float)

    def pilot(self, index, s_index):
        return self.set_value(self.system.input[index[0]], s_index)

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.point
        f_12 = trd(system, eq2)
        m_12 = tmd(system, p, eq2)
        system.sols[s1].mech_actions.append(MechanicalAction(-f_12, p, -m_12))
        system.sols[s2].mech_actions.append(MechanicalAction(f_12, p, m_12))
        self.force, self.torque = (f_12, m_12) if self.s1 == s2 else (-f_12, -m_12)

    @property
    def point(self):
        return self.__get_point0()

    def set_torque(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.t = lambda: t
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.t = t
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')

    def __get_point0(self):
        return self.system.get_origin(self.s1) + mat_mul_n2(rot(self.system.get_ref(self.s1)), self.p1)

    def __get_point1(self):
        return self.system.get_origin(self.s2) + mat_mul_n2(rot(self.system.get_ref(self.s2)), self.p2)

    __point_getters = __get_point0, __get_point1

    def __get_point__(self, index):
        return self.__point_getters[index](self)

    def get_value(self):
        return self.angle

    def set_value(self, value, s_index):
        self.angle = value
        theta = (self.angle + self.system.get_ref(self.s1) - self.system.get_ref(self.s2)) * (-2 * s_index + 1)
        change_ref(self.system, (self.s1, self.s2)[1 ^ s_index], theta, rot(theta),
                   self.__get_point__(1 ^ s_index), self.__get_point__(s_index))
        return self.system.eqs[self.s1] + self.system.eqs[self.s2]


class PrismaticJoint(Joint):
    id_ = 1
    tag = 'G'
    dof = 1

    def __init__(self, s1, s2, a1, d1, a2, d2):
        Joint.__init__(self, s1, s2, f'Pri({s2}/{s1})')
        self.a1, self.a2, self.d1, self.d2 = a1, a2, d1, d2
        self.delta = None
        self.interaction = PrismaticTangent(self, lambda: 0.)
        self.normal, self.tangent, self.torque = None, None, None
        self.input_tangent = None
        
    def input_mode(self):
        return f'{self.name}: Length',
    
    @property
    def identifier(self):
        return (self.s1, 0), (self.s2, 1)
    
    def reset(self, n):
        self.normal, self.tangent, self.torque = None, None, None
        self.input_tangent = None
        self.delta = np.zeros((n,), float)

    def pilot(self, index, s_index):
        return self.set_value(self.system.input[index[0]], s_index)

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.__get_point__(0)
        f_12 = trd(system, eq2)
        m_12 = tmd(system, p, eq2)

        system.sols[s1].mech_actions.append(MechanicalAction(-f_12, p, -m_12))
        system.sols[s2].mech_actions.append(MechanicalAction(f_12, p, m_12))
        t_12, n_12 = mat_mul_n(rot(-self.__get_angle0()), f_12)
        self.torque, self.tangent, self.normal = (m_12, t_12, n_12) if s2 == self.s1 else (-m_12, -t_12, -n_12)

    def set_tangent(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.f = lambda: t
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.f = t
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')

    def __get_angle0(self):
        return self.system.get_ref(self.s1) + self.a1

    def __get_angle1(self):
        return self.system.get_ref(self.s2) + self.a2

    __angle_getters = __get_angle0, __get_angle1

    def __get_angle__(self, index):
        return self.__angle_getters[index](self)

    def __get_dist__(self, index):
        return (2 * index - 1) * (self.d1 - self.d2)

    def __get_unit__(self, index):
        return unit(self.__get_angle__(index))

    def __get_point__(self, index):
        return self.system.get_origin((self.s1, self.s2)[index]) + \
            z_cross(self.__get_dist__(index) * self.__get_unit__(index))

    def get_value(self):
        return self.delta

    def set_value(self, value, s_index):
        self.delta = value
        theta = (self.__get_angle0() - self.__get_angle1()) * (-2 * s_index + 1)
        change_ref(self.system, (self.s1, self.s2)[1 ^ s_index], theta, rot(theta), self.system.get_origin((self.s1, self.s2)[1 ^ s_index]),
                   self.__get_point__(s_index) + self.delta * self.__get_unit__(s_index) * (-2 * s_index + 1))
        return self.system.eqs[self.s1] + self.system.eqs[self.s2]


class PinSlotJoint(Joint):
    id_ = 2
    tag = 'SP'
    dof = 2

    def __init__(self, s1, s2, a1, d1, p2):
        Joint.__init__(self, s1, s2, f'Pin({s2}/{s1})')
        self.p2, self.a1, self.d1 = p2, a1, d1
        self. delta = self.angle = None
        self.interaction = PinSlotTangentTorque(self, (lambda: 0), lambda: 0)
        self.normal, self.tangent, self.torque = None, None, None
        self.input_torque, self.input_tangent = None, None

    def input_mode(self):
        return f'{self.name}: Length', f'{self.name}: Angle'
    
    @property
    def identifier(self):
        return (self.s2, 1), (self.s1, 0)

    def reset(self, n):
        self.normal, self.tangent, self.torque = None, None, None
        self.input_torque, self.input_tangent = None, None
        self.delta = np.zeros((n,))
        self.angle = np.zeros((n,))

    def pilot(self, index, s_index):
        self.delta, self.angle = self.system.input[index[0]], self.system.input[index[1]]
        theta = (self.angle + self.system.get_ref(self.s1) - self.system.get_ref(self.s2)) * (-2 * s_index + 1)
        ux = unit(self.system.get_ref(self.s1) + self.a1 - theta * s_index)
        change_ref(self.system, (self.s1, self.s2)[1 ^ s_index], theta, rot(theta), self.__get_point__(1 ^ s_index), self.__get_point__(s_index) + self.delta * ux * (-2 * s_index + 1))
        return self.system.eqs[self.s1] + self.system.eqs[self.s2]

    @property
    def point(self):
        return self.__get_point1()

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.point
        f_12 = trd(system, eq2)
        m_12 = tmd(system, p, eq2)

        system.sols[s1].mech_actions.append(MechanicalAction(-f_12, p, -m_12))
        system.sols[s2].mech_actions.append(MechanicalAction(f_12, p, m_12))
        t_12, n_12 = mat_mul_n(rot(-system.get_ref(self.s1) - self.a1), f_12)
        self.normal, self.tangent, self.torque = (n_12, t_12, m_12) if s2 == self.s1 else (-n_12, -t_12, -m_12)

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

    def __get_point0(self):
        return self.system.get_origin(self.s1) + self.d1 * unit(self.system.get_ref(self.s1) + self.a1)

    def __get_point1(self):
        return self.system.get_origin(self.s2) + mat_mul_n2(rot(self.system.get_ref(self.s2)), self.p2)

    __point_getters = __get_point0, __get_point1

    def __get_point__(self, index):
        return self.__point_getters[index](self)

    def __get_angle__(self, index):
        return self.system.get_ref(self.s1) + self.a1

    def __get_unit__(self, index):
        return unit(self.__get_angle__(index))


class RectangularJoint(Joint):
    id_ = 3
    tag = 'T'
    dof = 2

    def __init__(self, s1, s2, angle, base):
        Joint.__init__(self, s1, s2, f'Rect({s2}/{s1})')
        self.angle, self.base = angle, base
        self.delta = None
        self.interaction = None
        self.torque = None

    def input_mode(self):
        return f'{self.name}: X', f'{self.name}: Y'
        
    @property
    def identifier(self):
        return (self.s1, 0), (self.s2, 1)
    
    def reset(self, n):
        self.torque = None
        self.delta = np.zeros((2, n), float)

    def pilot(self, index, s_index):
        self.delta = np.array((self.system.input[index[0]], self.system.input[index[1]]))
        f = -2 * s_index + 1
        theta = self.angle * f
        s1, s2 = (self.s1, self.s2)[::(-2 * s_index + 1)]
        ux, uy = unit(self.system.get_ref(s1) + self.base[0] - s_index * self.angle), unit(self.system.get_ref(s1) + self.base[1] - s_index * self.angle)
        change_ref(self.system, s2, theta, rot(theta), self.system.get_origin(s2),
                   self.system.get_origin(s1) + mat_mul_n(((ux[0], uy[0]), (ux[1], uy[1])), f * self.delta))
        return self.system.eqs[self.s1] + self.system.eqs[self.s2]

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = system.get_origin(self.s1)
        f = trd(system, eq2)
        m = tmd(system, p, eq2)

        system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))

    def __get_angle__(self, index):
        return self.system.get_ref((self.s1, self.s2)[index]) - self.angle * (2 * index - 1)


class ThreeDegreesOfFreedomJoint(Joint):
    id_ = 4
    tag = '3DoF'
    dof = 3

    def __init__(self, s1, s2):
        Joint.__init__(self, s1, s2, f'3DoF({s2}/{s1})')
        self.delta, self.angle = None, None
        self.interaction = None

    def input_mode(self):
        return f'{self.name}: X', f'{self.name}: Y', f'{self.name}: Angle'

    @property
    def identifier(self):  # Ne devrait pas servir...
        return (self.s1,), (self.s2,)

    def reset(self, n):
        self.delta = np.zeros((2, n), float)
        self.angle = np.zeros((n,), float)

    def pilot(self, index, s_index):
        self.delta[0], self.delta[1] = self.system.input[index[0]], self.system.input[index[1]]
        f = -2 * s_index + 1
        self.angle = self.system.input[index[2]]
        s1, s2 = (self.s1, self.s2)[::(-2 * s_index + 1)]
        theta = (self.system.get_ref(self.s1) + self.angle - self.system.get_ref(self.s2)) * f
        change_ref(self.system, s2, theta, rot(theta),
                   self.system.get_origin(s2), self.system.get_origin(s1))
        return self.system.eqs[s1] + self.system.eqs[s2]

    def block(self, system, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = system.get_origin(self.s1)
        f = trd(system, eq2)
        m = tmd(system, p, eq2)

        system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))
