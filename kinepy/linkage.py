from kinepy.geometry import np, rot, change_ref, mat_mul_n2, z_cross, unit, mat_mul_n
from kinepy.interactions import MechanicalAction, RevoluteTorque, PinSlotTangentTorque, PrismaticTangent, ZERO
from kinepy.dynamic import tmd, trd


FUNCTION_TYPE = type(lambda: 0)


class Joint:
    id_ = -1
    dof = None

    def __init__(self, system, s1, s2, name):
        self.s1, self.s2, self.name = s1, s2, name
        self.system, self.interaction = system, None

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
    
    def __init__(self, system, s1, s2, p1, p2):
        Joint.__init__(self, system, s1, s2, f'Rev({s2}/{s1})')
        self.p1_, self.p2_ = np.array(p1) * system.units['Length'], np.array(p2) * system.units['Length']
        self.angle_ = None
        self.interaction = RevoluteTorque(self, lambda: 0.)
        self.force_, self.torque_ = None, None

    @property
    def p1(self):
        return self.p1_ / self.system.units['Length']

    @p1.setter
    def p1(self, value):
        self.p1_ = np.array(value) * self.system.units['Length']

    @property
    def p2(self):
        return self.p2_ / self.system.units['Length']

    @p2.setter
    def p2(self, value):
        self.p2_ = np.array(value) * self.system.units['Length']

    @property
    def point(self):
        return self.__get_point0() / self.system.units['Length']

    @property
    def angle(self):
        return self.angle_ / self.system.units['Angle']

    @property
    def torque(self):
        return self.torque_ / self.system.units['Torque']

    @property
    def force(self):
        return self.force_ / self.system.units['Force']

    def input_mode(self):
        return f'{self.name}: Angle',
    
    @property
    def identifier(self):
        return (self.s1, 0), (self.s2, 1)

    def reset(self, n):
        self.force_, self.torque_ = None, None
        self.angle_ = np.zeros((n,), float)

    def pilot(self, index, s_index):
        return self.set_value(self.system.input[index[0]] * self.system.units['Angle'], s_index)

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.__get_point0()
        f_12 = trd(self.system, eq2)
        m_12 = tmd(self.system, p, eq2)
        self.system.sols[s1].mech_actions.append(MechanicalAction(-f_12, p, -m_12))
        self.system.sols[s2].mech_actions.append(MechanicalAction(f_12, p, m_12))
        self.force_, self.torque_ = (f_12, m_12) if self.s1 == s2 else (-f_12, -m_12)

    def set_torque(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.t = lambda: t * self.system.units['Torque']
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.t = lambda: t() * self.system.units['Torque']
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')

    def __get_point0(self):
        return self.system.get_origin(self.s1) + mat_mul_n2(rot(self.system.get_ref(self.s1)), self.p1_)

    def __get_point1(self):
        return self.system.get_origin(self.s2) + mat_mul_n2(rot(self.system.get_ref(self.s2)), self.p2_)

    __point_getters = __get_point0, __get_point1

    def __get_point__(self, index):
        return self.__point_getters[index](self)

    def get_value(self):
        return self.angle_

    def set_value(self, value, s_index):
        self.angle_ = value
        theta = (self.angle_ + self.system.get_ref(self.s1) - self.system.get_ref(self.s2)) * (-2 * s_index + 1)
        change_ref(self.system, (self.s1, self.s2)[1 ^ s_index], theta, rot(theta),
                   self.__get_point__(1 ^ s_index), self.__get_point__(s_index))
        return self.system.eqs[self.s1] + self.system.eqs[self.s2]

    def get_effort(self):
        return self.torque_

    def __add_effort__(self, value):
        p = self.__get_point0()
        self.system.sols[self.s1].mech_actions.append(MechanicalAction(ZERO, p, value))
        self.system.sols[self.s2].mech_actions.append(MechanicalAction(ZERO, p, -value))


class PrismaticJoint(Joint):
    id_ = 1
    tag = 'G'
    dof = 1

    def __init__(self, system, s1, s2, a1, d1, a2, d2):
        Joint.__init__(self, system, s1, s2, f'Pri({s2}/{s1})')
        self.a1_, self.a2_, self.d1_, self.d2_ = a1 * system.units['Angle'], a2 * system.units['Angle'], d1 * system.units['Length'], d2 * system.units['Length']
        self.sliding_ = None
        self.interaction = PrismaticTangent(self, lambda: 0.)
        self.normal_, self.tangent_, self.torque_ = None, None, None
        self.input_tangent = None

    @property
    def a1(self):
        return self.a1_ / self.system.units['Angle']

    @a1.setter
    def a1(self, value):
        self.a1_ = value * self.system.units['Angle']

    @property
    def a2(self):
        return self.a2_ / self.system.units['Angle']

    @a2.setter
    def a2(self, value):
        self.a2_ = value * self.system.units['Angle']

    @property
    def d1(self):
        return self.d1_ / self.system.units['Length']

    @d1.setter
    def d1(self, value):
        self.d1_ = value * self.system.units['Length']

    @property
    def d2(self):
        return self.d2_ / self.system.units['Length']

    @d2.setter
    def d2(self, value):
        self.d2_ = value * self.system.units['Length']

    @property
    def sliding(self):
        return self.sliding_ / self.system.units['Length']

    @property
    def normal(self):
        return self.normal_ / self.system.units['Force']

    @property
    def tangent(self):
        return self.tangent_ / self.system.units['Force']

    @property
    def torque(self):
        return self.torque_ / self.system.units['Torque']

    def input_mode(self):
        return f'{self.name}: Length',
    
    @property
    def identifier(self):
        return (self.s1, 0), (self.s2, 1)
    
    def reset(self, n):
        self.normal_, self.tangent_, self.torque_ = None, None, None
        self.sliding_ = np.zeros((n,), float)

    def pilot(self, index, s_index):
        return self.set_value(self.system.input[index[0]] * self.system.units['Length'], s_index)

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.__get_point__(0)
        f_12 = trd(self.system, eq2)
        m_12 = tmd(self.system, p, eq2)

        self.system.sols[s1].mech_actions.append(MechanicalAction(-f_12, p, -m_12))
        self.system.sols[s2].mech_actions.append(MechanicalAction(f_12, p, m_12))
        t_12, n_12 = mat_mul_n(rot(-self.__get_angle0()), f_12)
        self.torque_, self.tangent_, self.normal_ = (m_12, t_12, n_12) if s2 == self.s1 else (-m_12, -t_12, -n_12)

    def set_tangent(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.f = lambda: t * self.system.units['Force']
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.f = lambda: t() * self.system.units['Force']
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')

    def __get_angle0(self):
        return self.system.get_ref(self.s1) + self.a1_

    def __get_angle1(self):
        return self.system.get_ref(self.s2) + self.a2_

    __angle_getters = __get_angle0, __get_angle1

    def __get_angle__(self, index):
        return self.__angle_getters[index](self)

    def __get_dist__(self, index):
        return (2 * index - 1) * (self.d2_ - self.d1_)

    def __get_unit__(self, index):
        return unit(self.__get_angle__(index))

    def __get_point__(self, index):
        return self.system.get_origin((self.s1, self.s2)[index]) + \
            z_cross(self.__get_dist__(index) * self.__get_unit__(index))

    def get_value(self):
        return self.sliding_

    def set_value(self, value, s_index):
        self.sliding_ = value
        theta = (self.__get_angle0() - self.__get_angle1()) * (-2 * s_index + 1)
        change_ref(self.system, (self.s1, self.s2)[1 ^ s_index], theta, rot(theta), self.system.get_origin((self.s1, self.s2)[1 ^ s_index]),
                   self.__get_point__(s_index) + self.sliding_ * self.__get_unit__(s_index) * (-2 * s_index + 1))
        return self.system.eqs[self.s1] + self.system.eqs[self.s2]

    def get_effort(self):
        return self.tangent_

    def __add_effort__(self, value):
        f_21 = value * self.__get_unit__(0)
        p = self.system.get_origin(self.s1) + self.d1_ * z_cross(self.__get_unit__(0))
        self.system.sols[self.s1].mech_actions.append(MechanicalAction(f_21, p, 0.))
        self.system.sols[self.s2].mech_actions.append(MechanicalAction(-f_21, p, 0.))


class PinSlotJoint(Joint):
    id_ = 2
    tag = 'SP'
    dof = 2

    def __init__(self, system, s1, s2, a1, d1, p2):
        Joint.__init__(self, system, s1, s2, f'Pin({s2}/{s1})')
        self.p2_, self.a1_, self.d1_ = np.array(p2) * system.units['Length'], a1 * system.units['Angle'], d1 * system.units['Length']
        self.sliding_ = self.angle_ = None
        self.interaction = PinSlotTangentTorque(self, (lambda: 0), lambda: 0)
        self.normal_, self.tangent_, self.torque_ = None, None, None

    @property
    def a1(self):
        return self.a1_ / self.system.units['Angle']

    @a1.setter
    def a1(self, value):
        self.a1_ = value * self.system.units['Angle']

    @property
    def d1(self):
        return self.d1_ / self.system.units['Length']

    @d1.setter
    def d1(self, value):
        self.d1_ = value * self.system.units['Length']

    @property
    def p2(self):
        return self.p2_ / self.system.units['Length']

    @p2.setter
    def p2(self, value):
        self.p2_ = np.array(value) * self.system.units['Length']

    @property
    def point(self):
        return self.__get_point1() / self.system.units['Length']

    @property
    def sliding(self):
        return self.sliding_ / self.system.units['Length']

    @property
    def normal(self):
        return self.normal_ / self.system.units['Force']

    @property
    def tangent(self):
        return self.tangent_ / self.system.units['Force']

    @property
    def torque(self):
        return self.torque_ / self.system.units['Torque']

    def input_mode(self):
        return f'{self.name}: Length', f'{self.name}: Angle'
    
    @property
    def identifier(self):
        return (self.s2, 1), (self.s1, 0)

    def reset(self, n):
        self.normal_, self.tangent_, self.torque_ = None, None, None
        self.sliding_ = np.zeros((n,))
        self.angle_ = np.zeros((n,))

    def pilot(self, index, s_index):
        self.sliding_, self.angle_ = self.system.input[index[0]] * self.system.units['Length'], self.system.input[index[1]] * self.system.units['Angle']
        theta = (self.angle_ + self.system.get_ref(self.s1) - self.system.get_ref(self.s2)) * (-2 * s_index + 1)
        ux = unit(self.system.get_ref(self.s1) + self.a1_ - theta * s_index)
        change_ref(self.system, (self.s1, self.s2)[1 ^ s_index], theta, rot(theta), self.__get_point__(1 ^ s_index), self.__get_point__(s_index) + self.sliding_ * ux * (-2 * s_index + 1))
        return self.system.eqs[self.s1] + self.system.eqs[self.s2]

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.point
        f_12 = trd(self.system, eq2)
        m_12 = tmd(self.system, p, eq2)

        self.system.sols[s1].mech_actions.append(MechanicalAction(-f_12, p, -m_12))
        self.system.sols[s2].mech_actions.append(MechanicalAction(f_12, p, m_12))
        t_12, n_12 = mat_mul_n(rot(-self.system.get_ref(self.s1) - self.a1), f_12)
        self.normal_, self.tangent_, self.torque_ = (n_12, t_12, m_12) if s2 == self.s1 else (-n_12, -t_12, -m_12)

    def set_torque(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.t = lambda: t * self.system.units['Torque']
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.t = lambda: t() * self.system.units['Torque']
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')

    def set_tangent(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.interaction.f = lambda: t * self.system.units['Force']
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.f = lambda: t() * self.system.units['Force']
        else:
            raise ValueError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')

    def __get_point0(self):
        return self.system.get_origin(self.s1) + self.d1_ * unit(self.system.get_ref(self.s1) + self.a1_)

    def __get_point1(self):
        return self.system.get_origin(self.s2) + mat_mul_n2(rot(self.system.get_ref(self.s2)), self.p2_)

    __point_getters = __get_point0, __get_point1

    def __get_point__(self, index):
        return self.__point_getters[index](self)

    def __get_angle__(self, index):
        return self.system.get_ref(self.s1) + self.a1_

    def __get_unit__(self, index):
        return unit(self.__get_angle__(index))


class RectangularJoint(Joint):
    id_ = 3
    tag = 'T'
    dof = 2

    def __init__(self, system, s1, s2, angle, base):
        Joint.__init__(self, system, s1, s2, f'Rect({s2}/{s1})')
        self.angle_, self.base_ = angle * system.units['Angle'], np.array(base) * system.units['Angle']
        self.sliding_ = None
        self.interaction = None
        self.torque_ = None

    @property
    def angle(self):
        return self.angle_ / self.system.units['Angle']

    @angle.setter
    def angle(self, value):
        self.angle_ = value * self.system.units['Angle']

    @property
    def base(self):
        return self.base_ / self.system.units['Angle']

    @base.setter
    def base(self, value):
        self.base_ = value * self.system.units['Angle']

    @property
    def sliding(self):
        return self.sliding_ / self.system.units['Length']

    @property
    def torque(self):
        return self.torque_ / self.system.units['Torque']

    def input_mode(self):
        return f'{self.name}: X', f'{self.name}: Y'
        
    @property
    def identifier(self):
        return (self.s1, 0), (self.s2, 1)
    
    def reset(self, n):
        self.torque_ = None
        self.sliding_ = np.zeros((2, n), float)

    def pilot(self, index, s_index):
        self.sliding_ = np.array((self.system.input[index[0]] * self.system.units['Length'], self.system.input[index[1]] * self.system.units['Length']))
        f = -2 * s_index + 1
        theta = self.angle_ * f
        s1, s2 = (self.s1, self.s2)[::(-2 * s_index + 1)]
        ux, uy = unit(self.system.get_ref(s1) + self.base_[0] - s_index * self.angle_), unit(self.system.get_ref(s1) + self.base_[1] - s_index * self.angle_)
        change_ref(self.system, s2, theta, rot(theta), self.system.get_origin(s2),
                   self.system.get_origin(s1) + mat_mul_n(((ux[0], uy[0]), (ux[1], uy[1])), f * self.sliding_))
        return self.system.eqs[self.s1] + self.system.eqs[self.s2]

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.system.get_origin(self.s1)
        f = trd(self.system, eq2)
        m = tmd(self.system, p, eq2)

        self.system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        self.system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))

    def __get_angle__(self, index):
        return self.system.get_ref((self.s1, self.s2)[index]) - self.angle_ * (2 * index - 1)


class ThreeDegreesOfFreedomJoint(Joint):
    id_ = 4
    tag = '3DoF'
    dof = 3

    def __init__(self, system, s1, s2):
        Joint.__init__(self, system, s1, s2, f'3DoF({s2}/{s1})')
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
        self.delta[0], self.delta[1] = self.system.input[index[0]] * self.system.units['Length'], self.system.input[index[1]] * self.system.units['Length']
        f = -2 * s_index + 1
        self.angle = self.system.input[index[2]] * self.system.units['Angle']
        s1, s2 = (self.s1, self.s2)[::(-2 * s_index + 1)]
        theta = (self.system.get_ref(self.s1) + self.angle - self.system.get_ref(self.s2)) * f
        change_ref(self.system, s2, theta, rot(theta),
                   self.system.get_origin(s2), self.system.get_origin(s1))
        return self.system.eqs[s1] + self.system.eqs[s2]

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.system.get_origin(self.s1)
        f = trd(self.system, eq2)
        m = tmd(self.system, p, eq2)

        self.system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        self.system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))
