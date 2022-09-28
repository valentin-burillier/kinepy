from kinepy.base_units import *
from kinepy.solid import GhostSolid
from kinepy.interactions import RevoluteTorque, PinSlotTangentTorque, PrismaticTangent, ZERO, ZERO_F


class Joint(metaclass=MetaUnit):
    read_only = read_write = ()

    id_ = -1
    dof = None
    interaction = None

    def __init__(self, unit_system, rep, s1, s2, name):
        self.s1, self.s2, self.name = s1, s2, name
        self._unit_system, self.rep = unit_system, self.rep
        self.identifier = (s1.rep, 0), (s2.rep, 1)

    def __repr__(self):
        return self.name

    def set_value(self, value, index):
        pass

    def get_value(self):
        pass

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

    read_only = ANGLE2_, FORCE_, TORQUE_
    read_write = P1, P2

    def __init__(self, unit_system, rep, s1, s2, p1, p2):
        Joint.__init__(self, unit_system, rep, s1, s2, f'Rev({s2.rep}/{s1.rep})')
        self.p1, self.p2 = p1, p2
        self.interaction = RevoluteTorque(self, ZERO_F)

    def input_mode(self):
        return f'{self.name}: {ANGLE}',

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
            self.interaction.torque = lambda: t
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.torque = t
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

    read_only = SLIDING, NORMAL, TANGENT, TORQUE_
    read_write = A1, A2, D1, D1

    def __init__(self, unit_system, rep, s1, s2, a1, d1, a2, d2):
        Joint.__init__(self, unit_system, rep, s1, s2, f'Pri({s2}/{s1})')
        self.a1, self.a2, self.d1, self.d2 = a1, a2, d1, d2
        self.interaction = PrismaticTangent(self, ZERO_F)

    def input_mode(self):
        return f'{self.name}: {LENGTH}',
    
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
            self.interaction.f = lambda: t
        elif isinstance(t, FUNCTION_TYPE):
            self.interaction.f = t
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

    read_only = SLIDING, NORMAL, TANGENT, TORQUE_
    read_write = A1, D1, P2

    def __init__(self, unit_system, rep, s1, s2, a1, d1, p2):
        Joint.__init__(self, unit_system, rep, s1, s2, f'Pin({s2}/{s1})')
        self.p2, self.a1, self.d1 = p2, a1, d1
        self.interaction = PinSlotTangentTorque(self, ZERO_F, ZERO_F)
        self.ghost_sol = GhostSolid()


    def input_mode(self):
        return f'{self.name}: {LENGTH}', f'{self.name}: {ANGLE}'

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

    read_only = SLIDING, TORQUE_
    read_write = P1, P2, ANGLE1_, ('base', ANGLE, (0., np.pi/2))

    def __init__(self, unit_system, rep, s1, s2, angle, base, p1, p2):
        Joint.__init__(self, unit_system, rep, s1, s2, f'Rect({s2}/{s1})')
        self.angle, self.base, self.p1, self.p2 = angle, base, p1, p2
        self.ghost_sol = GhostSolid()
        self.ghost_j1, self.ghost_j2 = GhostPrismatic(self, 0), GhostPrismatic(self, 1)

    def input_mode(self):
        return f'{self.name}: X {LENGTH}', f'{self.name}: Y {LENGTH}'
    
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

    read_only = SLIDING, ANGLE2_
    read_write = P1, P2

    def __init__(self, unit_system, rep, s1, s2, p1, p2):
        Joint.__init__(self, unit_system, rep, s1, s2, f'3DoF({s2}/{s1})')
        self.p1, self.p2 = p1, p2
        self.ghost_sol = GhostSolid()

    def input_mode(self):
        return f'{self.name}: X {LENGTH}', f'{self.name}: Y {LENGTH}', f'{self.name}: {FORCE}'

    def reset(self, n):
        self.sliding_ = np.zeros((2, n), float)
        self.angle_ = np.zeros((n,), float)

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

J3DOF = ThreeDegreesOfFreedomJoint


class GhostRevolute:
    id_ = 1
    rep = None

    def __init__(self, joint, index):
        self.joint, self.index = joint, index
        self.s1, self.s2 = ((joint.s1, joint.ghost_sol), (joint.ghost_sol, joint.s2))[index]
        self.get_p1 = ((lambda: joint.p1_), lambda: (0., 0.))[index]
        self.get_p2 = ((lambda: (0., 0.)), lambda: joint.p2_)[index]

    def __getattribute__(self, item):
        if item == 'p1_':
            return self.get_p1()
        elif item == 'p2_':
            return self.get_p2()
        return object.__getattribute__(item)


class GhostPrismatic:
    id_ = 2
    rep = None

    def __init__(self, joint, index):
        self.joint, self.index = joint, index
        self.s1, self.s2 = ((joint.s1, joint.ghost_sol), (joint.ghost_sol, joint.s2))[index]
        self.get_a = ((lambda: joint.a1_), lambda: joint.a2_)[index]
        self.d1_, self.d2_ = 0., 0.

    def __getattribute__(self, item):
        if item == 'a1_':
            return self.get_a()
        elif item == 'a2_':
            return self.get_a()
        return object.__getattribute__(item)
