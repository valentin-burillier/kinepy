from kinepy.units import ANGLE, LENGTH
from kinepy.objects.solid import Solid
from kinepy.math.geometry import np, rotate_eq, move_eq, get_point, rvec, unit, get_angle, get_zero, det, dot, z_cross,\
    ZERO_POINT, ZERO_21
from kinepy.math.dynamic import trd, tmd


class Joint:
    tag, id_, dof, interaction, inputs, rep = 'Joint', 0, None, None, (), None
    s1: Solid
    s2: Solid

    def __init__(self, s1, s2, name):
        self.s1, self.s2, self.name = s1, s2, f'{self.tag}({name})'

    def __repr__(self):
        return self.name

    def get_value(self):
        pass

    def input_mode(self):
        return tuple(f'{self.name}: {i}' for i in self.inputs)


class RevoluteJoint(Joint):
    id_, tag, dof, inputs = 1, 'Rev', 1, (ANGLE,)
    force = torque = angle = None

    def __init__(self, s1, s2, p1, p2, name):
        Joint.__init__(self, s1, s2, name)
        self.p1, self.p2 = p1, p2

    def reset(self, n):
        self.force, self.torque = np.zeros((2, n)), np.zeros((n,))
        self.angle = np.zeros((n,), float)

    def pilot(self, eq1, eq2, value):
        return self.set_value(value, eq1, eq2)

    def get_value(self):
        return self.angle

    def set_value(self, value, _, eq2):
        self.angle = value
        theta = value + self.s1.angle - self.s2.angle
        rotate_eq(eq2, theta)
        move_eq(eq2, get_point(self, 0) - get_point(self, 1))

    def block(self, eq1, eq2, ref):
        p = self.s1.origin + rvec(self.s1.angle, self.p1)
        f_21 = trd((eq2, eq1)[ref]) * (-1, 1)[ref]
        m_21 = tmd(p, (eq2, eq1)[ref]) * (-1, 1)[ref]
        self.s1.add_mech_action(f_21, p, m_21)
        self.s2.add_mech_action(-f_21, p, -m_21)
        self.force, self.torque = f_21, m_21

    def put_effort(self, value):
        p = get_point(self, 0)
        self.s1.add_mech_action(ZERO_21, p, value)
        self.s2.add_mech_action(ZERO_21, p, -value)

    def get_effort(self):
        return self.torque


class PrismaticJoint(Joint):
    id_, tag, dof, inputs = 2, 'Pri', 1, (LENGTH,)
    normal = tangent = torque = sliding = None

    def __init__(self, s1, s2, a1, d1, a2, d2, name):
        Joint.__init__(self, s1, s2, name)
        self.a1, self.a2, self.d1, self.d2 = a1, a2, d1, d2
    
    def reset(self, n):
        self.normal, self.tangent, self.torque = np.zeros((n,)), np.zeros((n,)), np.zeros((n,))
        self.sliding = np.zeros((n,), float)

    def pilot(self, eq1, eq2, value):
        return self.set_value(value, eq1, eq2)

    def get_value(self):
        return self.sliding

    def set_value(self, value, _, eq2):
        self.sliding = value
        u = unit(alpha := get_angle(self, 0))
        rotate_eq(eq2, alpha - get_angle(self, 1))
        move_eq(eq2, get_zero(self, 0, u) - get_zero(self, 1, u) + value * u)

    def block(self, eq1, eq2, ref):
        u = unit(get_angle(self, 0))
        p = get_zero(self, 0, u)

        f_21 = trd((eq2, eq1)[ref]) * (-1, 1)[ref]
        m_21 = tmd(p, (eq2, eq1)[ref]) * (-1, 1)[ref]

        self.s1.add_mech_action(f_21, p, m_21)
        self.s2.add_mech_action(-f_21, p, -m_21)
        self.torque, self.tangent, self.normal = m_21, dot(f_21, u), det(u, f_21)

    def put_effort(self, value):
        u = unit(get_angle(self, 0))
        n, p = z_cross(u) * value, get_zero(self, 0, u)
        self.s1.add_mech_action(n, p, 0.)
        self.s2.add_mech_action(-n, p, 0.)

    def get_effort(self):
        return self.normal


class GhostRevolute(Joint):
    g = (0, 0)
    id_, tag = RevoluteJoint.id_, 'GhostRev'
    force = torque = angle = None

    def __init__(self, master, index, name):
        self.master = master
        self.index = index
        s1, s2 = (master.s1, master.ghost_sol)[index], (master.ghost_sol, master.s2)[index]
        Joint.__init__(self, s1, s2, name)

    p1 = property(lambda self: ZERO_POINT if self.index else self.master.p1)
    p2 = property(lambda self: ZERO_POINT if not self.index else self.master.p2)

    reset = RevoluteJoint.reset


class GhostPrismatic(Joint):
    id_, tag = PrismaticJoint.id_, 'GhostPri'
    normal = tangent = torque = sliding = None
    d2 = 0.
    g = (0, 0)

    def __init__(self, master, index, name):
        self.master = master
        self.index = index
        s1, s2 = (master.s1, master.ghost_sol)[index], (master.ghost_sol, master.s2)[index]
        Joint.__init__(self, s1, s2, name)

    a1 = property(lambda self: self.master.a2 if self.index else self.master.a1)
    d1 = property(lambda self: 0. if isinstance(self.master, RectangularJoint) else self.master.d1)
    a2 = property(lambda self: self.master.a2 - self.master.angle if self.index else self.master.a1)

    reset = PrismaticJoint.reset


class PinSlotJoint(Joint):
    id_, tag, dof, inputs = 3, 'PinSlot', 2, (LENGTH, ANGLE)
    normal = tangent = torque = sliding = angle = None

    def __init__(self, s1, s2, a1, d1, p2, name):
        Joint.__init__(self, s1, s2, name)
        self.p2, self.a1, self.d1 = p2, a1, d1
        self.ghost_sol = Solid(0, 0, (0, 0), f'Ghost-{self.name}')
        self.ghost_j1 = GhostPrismatic(self, 0, f'{self.name}-0')
        self.ghost_j2 = GhostRevolute(self, 1, f'{self.name}-1')

    def reset(self, n):
        self.normal, self.tangent, self.torque = np.zeros((n,)), np.zeros((n,)), np.zeros((n,))
        self.sliding = np.zeros((n,))
        self.angle = np.zeros((n,))
        self.ghost_sol.reset(n)
        self.ghost_j1.reset(n)
        self.ghost_j2.reset(n)

    def pilot(self, _, eq2, sliding, angle):
        self.sliding, self.angle = sliding, angle
        theta = angle + self.s1.angle - self.s2.angle
        rotate_eq(eq2, theta)
        u = unit(self.s1.angle + self.a1)
        z, p = self.s1.origin + self.d1 * z_cross(u), self.s2.origin + rvec(self.s2.angle, self.p2)
        move_eq(eq2, u * sliding + z - p)

    def block(self, eq1, eq2, ref):
        p = self.s2.origin + rvec(self.s2.angle, self.p2)
        f_21 = trd((eq2, eq1)[ref]) * (-1, 1)[ref]
        m_21 = tmd(p, (eq2, eq1)[ref]) * (-1, 1)[ref]

        self.s1.add_mech_action(f_21, p, m_21)
        self.s2.add_mech_action(-f_21, p, -m_21)

        u = unit(self.s1.angle + self.a1)
        self.normal, self.tangent, self.torque = dot(f_21, u), det(u, f_21), m_21

    def kin_recover_ghosts(self):
        self.angle = self.ghost_j2.angle
        self.sliding = self.ghost_j1.sliding

    def dyn_recover_ghosts(self):
        self.normal = self.ghost_j1.normal


class RectangularJoint(Joint):
    id_, tag, dof, inputs = 4, 'Rec', 2, (f'X ({LENGTH})', f'Y ({LENGTH})')
    torque = sliding = force_x = force_y = None

    def __init__(self, s1, s2, angle, a1, a2, p1, p2, name):
        Joint.__init__(self, s1, s2, name)
        self.angle, self.a1, self.a2, self.p1, self.p2 = angle, a1, a2, p1, p2
        self.ghost_sol = Solid(0, 0, (0, 0), f'Ghost-{self.name}')
        self.ghost_j1 = GhostPrismatic(self, 0, f'{self.name}-0')
        self.ghost_j2 = GhostPrismatic(self, 1, f'{self.name}-1')

    def reset(self, n):
        self.torque = self.force_x = self.force_y = None
        self.sliding = np.zeros((2, n), float)
        self.ghost_sol.reset(n)
        self.ghost_j1.reset(n)
        self.ghost_j2.reset(n)

    def pilot(self, _, eq2, x, y):
        self.sliding = x, y
        rotate_eq(eq2, self.s1.angle + self.angle - self.s2.angle)
        ux, uy = unit(self.s1.angle + self.a1), unit(self.s1.angle + self.a2)
        p1 = self.s1.origin + rvec(self.s1.angle, self.p1)
        p2 = self.s2.origin + rvec(self.s2.angle, self.p2)
        move_eq(eq2, p1 + x * ux + y * uy - p2)

    def block(self, eq1, eq2, ref):
        p = self.s1.origin + rvec(self.s1.angle, self.p1)
        f_21 = trd((eq2, eq1)[ref]) * (-1, 1)[ref]
        m_21 = tmd(p, (eq2, eq1)[ref]) * (-1, 1)[ref]

        self.s1.add_mech_action(f_21, p, m_21)
        self.s2.add_mech_action(-f_21, p, -m_21)

        ux, uy = self.s1.angle + self.a1, self.s1.angle + self.a2
        self.force_x = det(f_21, uy) / det(ux, uy)
        self.force_y = det(f_21, ux) / det(uy, ux)
        self.torque = m_21

    def kin_recover_ghosts(self):
        ux, uy = unit(self.s1.angle + self.a1), unit(self.s1.angle + self.a2)
        p = self.s2.origin + rvec(self.s2.angle, self.p2) - self.s1.origin - rvec(self.s1.angle, self.p1)
        d = det(ux, uy)
        self.sliding = det(p, uy) / d, det(ux, p) / d

    def dyn_recover_ghosts(self):
        self.torque = self.ghost_j1.torque


class ThreeDegreesOfFreedomJoint(Joint):
    id_, tag, dof, inputs = 5, '3DoF', 3, (f'X ({LENGTH})', f'Y ({LENGTH})', ANGLE)
    sliding = angle = force = torque = None

    def __init__(self, s1, s2, p1, p2, name):
        Joint.__init__(self, s1, s2, name)
        self.p1, self.p2 = p1, p2

    def reset(self, n):
        self.sliding = np.zeros((2, n), float)
        self.angle = np.zeros((n,), float)

    def pilot(self, _, eq2, x, y, angle):
        self.sliding, self.angle = (x, y), angle
        rotate_eq(eq2, angle + self.s1.origin - self.s2.origin)
        p1 = self.s1.origin + rvec(self.s1.angle, self.p1)
        p2 = self.s2.origin + rvec(self.s2.angle, self.p2)
        move_eq(eq2, p1 + (x, y) - p2)

    def block(self, eq1, eq2, ref):
        p = self.s1.origin + rvec(self.s1.angle, self.p1)
        f_21 = trd((eq2, eq1)[ref]) * (-1, 1)[ref]
        m_21 = tmd(p, (eq2, eq1)[ref]) * (-1, 1)[ref]

        self.s1.add_mech_action(f_21, p, m_21)
        self.s2.add_mech_action(-f_21, p, -m_21)

        self.force = f_21
        self.torque = m_21
