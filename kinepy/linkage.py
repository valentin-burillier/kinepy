import numpy as np
from kinepy.base_units import *
from kinepy.solid import GhostSolid
from kinepy.interactions import RevoluteTorque, PinSlotTangentTorque, PrismaticTangent, ZERO, ZERO_F
from kinepy.geometry import rotate_eq, move_eq, get_point, get_angle, get_zero, unit, z_cross, rvec, det


def to_function(f):
    def g(self, arg):
        if isinstance(arg, (int, float, np.ndarray)):
            return f(self, lambda: arg)
        elif not isinstance(arg, FUNCTION_TYPE):
            raise TypeError(f'Invalid type: {type(t)}, expected int, float, np.ndarray or function')
        return f(self, arg)
    return g


class Joint(metaclass=MetaUnit):
    read_only = read_write = ()
    tag = 'Joint'
    id_ = 0
    dof = None
    interaction = None
    inputs = ()

    def __init__(self, unit_system, rep, s1, s2):
        self.s1, self.s2, self.name = s1, s2, f'{self.tag}({s2.rep}/{s1.rep})'
        self._unit_system, self.rep = unit_system, rep
        self.identifier = (s1.rep, 0), (s2.rep, 1)

    def __repr__(self):
        return self.name

    def get_value(self):
        pass

    def input_mode(self):
        return tuple(f'{self.name}: {i}' for i in self.inputs)


class RevoluteJoint(Joint):
    id_ = 1
    tag = 'Rev'
    dof = 1
    inputs = ANGLE,

    read_only = ANGLE2_, FORCE_, TORQUE_
    read_write = P1, P2

    def __init__(self, unit_system, rep, s1, s2, p1, p2):
        Joint.__init__(self, unit_system, rep, s1, s2)
        self.p1, self.p2 = p1, p2
        self.interaction = RevoluteTorque(self._unit_system, self, ZERO_F)

    @physics_output(LENGTH)
    def point(self):
        return self.s1.origin_ + rvec(self.s1.angle_, self.p1_)

    def reset(self, n):
        self.force_, self.torque_ = None, None
        self.angle_ = np.zeros((n,), float)

    @physics_input('', '', ANGLE)
    def pilot(self, eq1, eq2, value):
        return self.set_value(value, eq1, eq2)

    def set_value(self, value, eq1, eq2):
        self.angle_ = value
        theta = value + self.s1.angle_ - self.s2.angle_
        rotate_eq(eq2, theta)
        move_eq(eq2, get_point(self, 0) - get_point(self, 1))

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.__get_point0()
        f_12 = trd(self.system, eq2)
        m_12 = tmd(self.system, p, eq2)
        self.system.sols[s1].mech_actions.append(MechanicalAction(-f_12, p, -m_12))
        self.system.sols[s2].mech_actions.append(MechanicalAction(f_12, p, m_12))
        self.force_, self.torque_ = (f_12, m_12) if self.s1 == s2 else (-f_12, -m_12)

    @to_function
    def set_torque(self, t):
        self.interaction.torque = t

    def get_value(self):
        return self.angle_

    def get_effort(self):
        return self.torque_

    def __add_effort__(self, value):
        p = self.__get_point0()
        self.system.sols[self.s1].mech_actions.append(MechanicalAction(ZERO, p, value))
        self.system.sols[self.s2].mech_actions.append(MechanicalAction(ZERO, p, -value))


class PrismaticJoint(Joint):
    id_ = 2
    tag = 'Pri'
    dof = 1
    inputs = LENGTH,

    read_only = SLIDING, NORMAL, TANGENT, TORQUE_
    read_write = A1, A2, D1, D2

    def __init__(self, unit_system, rep, s1, s2, a1, d1, a2, d2):
        Joint.__init__(self, unit_system, rep, s1, s2)
        self.a1, self.a2, self.d1, self.d2 = a1, a2, d1, d2
        self.interaction = PrismaticTangent(self._unit_system, self, ZERO_F)
    
    def reset(self, n):
        self.normal_, self.tangent_, self.torque_ = None, None, None
        self.sliding_ = np.zeros((n,), float)

    @physics_input('', '', LENGTH)
    def pilot(self, eq1, eq2, value):
        return self.set_value(value, eq1, eq2)

    def set_value(self, value, eq1, eq2):
        self.sliding_ = value
        u = unit(alpha := get_angle(self, 0))
        rotate_eq(eq2, alpha - get_angle(self, 1))
        move_eq(eq2, get_zero(self, 0, u) - get_zero(self, 1, u) + value * u)

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.__get_point__(0)
        f_12 = trd(self.system, eq2)
        m_12 = tmd(self.system, p, eq2)

        self.system.sols[s1].mech_actions.append(MechanicalAction(-f_12, p, -m_12))
        self.system.sols[s2].mech_actions.append(MechanicalAction(f_12, p, m_12))
        t_12, n_12 = mat_mul_n(rot(-self.__get_angle0()), f_12)
        self.torque_, self.tangent_, self.normal_ = (m_12, t_12, n_12) if s2 == self.s1 else (-m_12, -t_12, -n_12)

    @to_function
    def set_tangent(self, t):
        self.interaction.f = t

    def get_value(self):
        return self.sliding_

    def get_effort(self):
        return self.tangent_

    def __add_effort__(self, value):
        f_21 = value * self.__get_unit__(0)
        p = self.system.get_origin(self.s1) + self.d1_ * z_cross(self.__get_unit__(0))
        self.system.sols[self.s1].mech_actions.append(MechanicalAction(f_21, p, 0.))
        self.system.sols[self.s2].mech_actions.append(MechanicalAction(-f_21, p, 0.))


class PinSlotJoint(Joint):
    id_ = 3
    tag = 'PinSlot'
    dof = 2
    inputs = LENGTH, ANGLE

    read_only = SLIDING, NORMAL, TANGENT, TORQUE_
    read_write = A1, D1, P2

    def __init__(self, unit_system, rep, s1, s2, a1, d1, p2):
        Joint.__init__(self, unit_system, rep, s1, s2)
        self.p2, self.a1, self.d1 = p2, a1, d1
        self.interaction = PinSlotTangentTorque(self._unit_system, self, ZERO_F, ZERO_F)
        self.ghost_sol = GhostSolid()
        self.ghost_j1, self.ghost_j2 = GhostPrismatic(self, 0), GhostRevolute(self, 1)

    def reset(self, n):
        self.normal_, self.tangent_, self.torque_ = None, None, None
        self.sliding_ = np.zeros((n,))
        self.angle_ = np.zeros((n,))
        self.ghost_sol.reset(n)
        self.ghost_j1.reset(n)
        self.ghost_j2.reset(n)

    @physics_input('', '', LENGTH, ANGLE)
    def pilot(self, eq1, eq2, sliding, angle):
        self.sliding_, self.angle_ = sliding, angle
        theta = angle + self.s1.angle_ - self.s2.angle_
        rotate_eq(eq2, theta)
        u = unit(alpha := self.s1.angle_ + self.a1)
        z, p = self.s1.origin_ + self.d1_ * z_cross(u), self.s2.origin_ +r_vec(self.s2.angle_, self.p2_)
        move_eq(eq2, u * sliding + z - p)

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.point
        f_12 = trd(self.system, eq2)
        m_12 = tmd(self.system, p, eq2)

        self.system.sols[s1].mech_actions.append(MechanicalAction(-f_12, p, -m_12))
        self.system.sols[s2].mech_actions.append(MechanicalAction(f_12, p, m_12))
        t_12, n_12 = mat_mul_n(rot(-self.system.get_ref(self.s1) - self.a1), f_12)
        self.normal_, self.tangent_, self.torque_ = (n_12, t_12, m_12) if s2 == self.s1 else (-n_12, -t_12, -m_12)

    @to_function
    def set_torque(self, t):
        self.interaction.t = t

    @to_function
    def set_tangent(self, t):
        self.interaction.f = t

    def kin_recover_ghosts(self):
        self.angle_ = self.ghost_j2.angle_
        self.sliding_ = self.ghost_j1.sliding_


class RectangularJoint(Joint):
    id_ = 4
    tag = 'Rec'
    dof = 2
    inputs = f'X ({LENGTH})', f'Y ({LENGTH})'

    read_only = SLIDING, TORQUE_
    read_write = P1, P2, ANGLE1_, A1, A2

    def __init__(self, unit_system, rep, s1, s2, angle, a1, a2, p1, p2):
        Joint.__init__(self, unit_system, rep, s1, s2)
        self.angle, self.a1, self.a2, self.p1, self.p2 = angle, a1, a2, p1, p2
        self.ghost_sol = GhostSolid()
        self.ghost_j1, self.ghost_j2 = GhostPrismatic(self, 0), GhostPrismatic(self, 1)
    
    def reset(self, n):
        self.torque_ = None
        self.sliding_ = np.zeros((2, n), float)
        self.ghost_sol.reset(n)
        self.ghost_j1.reset(n)
        self.ghost_j2.reset(n)

    @physics_input('', '', LENGTH, LENGTH)
    def pilot(self, eq1, eq2, x, y):
        self.sliding_ = x, y
        rotate_eq(eq2, self.s1.angle_ + self.angle_ - self.s2.angle_)
        ux, uy = unit(self.s1.angle_ + self.a1_), unit(self.s1.angle_ + self.a2_)
        p1 = self.s1.origin_ + rvec(self.s1.angle_, self.p1_)
        p2 = self.s2.origin_ + rvec(self.s2.angle_, self.p2_)
        move_eq(eq2, p1 + x * ux + y * uy - p2)

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.system.get_origin(self.s1)
        f = trd(self.system, eq2)
        m = tmd(self.system, p, eq2)

        self.system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        self.system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))

    def kin_recover_ghosts(self):
        ux, uy = unit(self.s1.angle_ + self.a1_), unit(self.s1.angle_ + self.a2_)
        p = self.s2.origin_ + rvec(self.s2.angle_, self.p2_) - self.s1.origin_ - rvec(self.s1.angle_, self.p1_)
        d = det(ux, uy)
        self.sliding_ = det(p, uy) / d, det(ux, p) / d


class ThreeDegreesOfFreedomJoint(Joint):
    id_ = 5
    tag = '3DoF'
    dof = 3

    inputs = f'X ({LENGTH})', f'Y ({LENGTH})', ANGLE

    read_only = SLIDING, ANGLE2_
    read_write = P1, P2

    def __init__(self, unit_system, rep, s1, s2, p1, p2):
        Joint.__init__(self, unit_system, rep, s1, s2)
        self.p1, self.p2 = p1, p2
        self.ghost_sol = GhostSolid()

    def reset(self, n):
        self.sliding_ = np.zeros((2, n), float)
        self.angle_ = np.zeros((n,), float)

    @physics_input('', '', LENGTH, LENGTH, ANGLE)
    def pilot(self, eq1, eq2, x, y, angle):
        self.sliding_, self.angle_ = (x, y), angle
        rotate_eq(eq2, angle + self.s1.origin_ - self.s2.origin_)
        p1 = self.s1.origin_ + r_vec(self.s1.angle_, self.p1_)
        p2 = self.s2.origin_ + r_vec(self.s2.angle_, self.p2_)
        move_eq(eq2, p1 + (x, y) - p2)

    def block(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        p = self.system.get_origin(self.s1)
        f = trd(self.system, eq2)
        m = tmd(self.system, p, eq2)

        self.system.sols[s1].mech_actions[self.name](MechanicalAction(f, p, m))
        self.system.sols[s2].mech_actions[self.name](MechanicalAction(-f, p, -m))


J3DOF = ThreeDegreesOfFreedomJoint


class Ghost:
    def __repr__(self):
        return f'Ghost{self.tag}{self.index}({repr(self.joint)})'


class GhostRevolute(Ghost):
    id_ = 1
    rep = None
    force_ = angle_ = torque_ = None
    tag = 'R'

    def __init__(self, joint, index):
        self.joint, self.index = joint, index
        self.s1, self.s2 = ((joint.s1, joint.ghost_sol), (joint.ghost_sol, joint.s2))[index]

    p1_ = property(lambda self: self.joint.p1_ if not self.index else (0., 0.))
    p2_ = property(lambda self: ((0., 0.), self.joint.p2_)[self.index])
    reset = RevoluteJoint.reset



class GhostPrismatic(Ghost):
    id_ = 2
    rep = None
    normal_ = tangent_ = torque_ = sliding_ = None
    tag = 'P'

    def __init__(self, joint, index):
        self.joint, self.index = joint, index
        self.s1, self.s2 = ((joint.s1, joint.ghost_sol), (joint.ghost_sol, joint.s2))[index]
        self.d2_ = 0.

    a1_ = property(lambda self: self.joint.a1_ if not self.index else self.joint.a2_)
    a2_ = property(lambda self: self.joint.a1_ if not self.index else self.joint.a2_ - self.joint.angle_)
    d1_ = property(lambda self: self.joint.d1_ if isinstance(self.joint, PinSlotJoint) else 0.)
    reset = PrismaticJoint.reset