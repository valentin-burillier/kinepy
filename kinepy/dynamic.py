from kinepy.geometry import *


class MechanicalAction:
    def __init__(self, f: np.ndarray, p: np.ndarray, m):
        self.f, self.p, self.m = f, p, m

    def babar(self, point):
        return self.m + det(self.p - point, self.f)


class Interaction:
    name: str

    def set_am(self, system):
        pass


class AccelerationField(Interaction):
    def __init__(self, g, name):
        self.g, self.name = g, name

    def set_am(self, system):
        for s in system.sols:
            s.mech_actions[self.name] = MechanicalAction(self.g * s.m, s.get_point(s.g), 0.)


class Spring(Interaction):
    def __init__(self, k, l0, s1, s2, p1, p2):
        self.s1, self.s2, self.p1, self.p2 = s1, s2, p1, p2
        self.l0, self.k = l0, k
        self.name = f'Spring({s2}/{s1})'

    def set_am(self, system):
        a, b = get_point(system, self.s1, self.p1), get_point(system, self.s2, self.p2)
        ab = b - a
        l_ = mag(ab)
        system.sols[self.s1].mech_actions[self.name] = MechanicalAction(ab * self.k * (1 - self.l0 / l_), a, 0.)
        system.sols[self.s2].mech_actions[self.name] = MechanicalAction(ab * self.k * (self.l0 / l_ - 1), b, 0.)


def tmd(system, point, eq) -> np.ndarray:
    return sum((sum(am.babar(point) for am in system.sols[s].mech_actions.values()) for s in eq), np.array((0.,)))


def trd(system, eq) -> np.ndarray:
    return sum((sum(am.f for am in system.sols[s].mech_actions.values()) for s in eq), np.array((0., 0.)))


def p_p_p(system, cycle, rev1, rev2, rev3, _, eq2, eq3):
    (s13, _), (s11, p11) = rev1
    (s21, _), (s22, p22) = rev2
    (s32, _), (s33, p33) = rev3
    p1, p2, p3 = get_point(system, s11, p11), get_point(system, s22, p22), get_point(system, s33, p33)
    p1p2, p3p2 = p2 - p1, p2 - p3

    inv_12, inv_32 = inv_mag(p1p2), inv_mag(p3p2)
    u1, u2 = p1p2 * inv_12, p3p2 * inv_32

    x1, x2 = -tmd(system, p1, eq2 + eq3) * inv_12, -tmd(system, p2, eq2) * inv_32
    y1 = (x2 - dot(u1, u2) * x1) / det(u1, u2)

    f2 = x1 * z_cross(u1) - y1 * u1
    f3 = -trd(system, eq2) - f2
    f1 = -trd(system, eq3) + f3

    n1 = system.joints[cycle[0]].name
    system.sols[s11].mech_actions[n1] = MechanicalAction(-f1, p1, 0.)
    system.sols[s13].mech_actions[n1] = MechanicalAction(f1, p1, 0.)

    n2 = system.joints[cycle[1]].name
    system.sols[s21].mech_actions[n2] = MechanicalAction(-f2, p1, 0.)
    system.sols[s22].mech_actions[n2] = MechanicalAction(f2, p1, 0.)

    n3 = system.joints[cycle[2]].name
    system.sols[s33].mech_actions[n3] = MechanicalAction(-f3, p1, 0.)
    system.sols[s32].mech_actions[n3] = MechanicalAction(f3, p1, 0.)


def p_p_g(system, cycle, rev1, rev2, pri, _, eq2, eq3):
    (s13, _), (s11, p11) = rev1
    (s21, _), (s22, p22) = rev2
    (s1, a1, _), (s1p, _, _) = pri

    p1, p2 = get_point(system, s11, p11), get_point(system, s22, p22)
    u = unit(system.get_ref(s1) + a1)
    n23 = z_cross(u) * (tmd(system, eq3, p1) + (t2 := tmd(system, eq2, p2))) / dot(u, p1 - p2)
    f1 = -trd(system, eq3) - n23
    f2 = -trd(system, eq2) + n23

    n1 = system.joints[cycle[0]].name
    system.sols[s13].mech_actions[n1] = MechanicalAction(f1, p1, 0.)
    system.sols[s11].mech_actions[n1] = MechanicalAction(-f1, p1, 0.)

    n2 = system.joints[cycle[1]].name
    system.sols[s21].mech_actions[n2] = MechanicalAction(-f2, p2, 0.)
    system.sols[s22].mech_actions[n2] = MechanicalAction(f2, p2, 0.)

    n3 = system.joints[cycle[2]].name
    system.sols[s1].mech_actions[n3] = MechanicalAction(-n23, p2, -t2)
    system.sols[s1p].mech_actions[n3] = MechanicalAction(n23, p2, t2)


def g_p_p(system, cycle, pri, rev1, rev2, _, eq2, eq3):
    (s1p, _, _), (s1, a1, _) = pri
    (s11, p11), (s12, _) = rev1
    (s22, p22), (s23, _) = rev2

    p1, p2 = get_point(system, s11, p11), get_point(system, s22, p22)
    u = unit(system.get_ref(s1) + a1)
    n13 = z_cross(u) * ((t3 := tmd(system, eq3, p2)) - tmd(system, eq3 + eq2, p1)) / dot(u, p2 - p1)
    f2 = -trd(system, eq3) - n13
    f1 = -trd(system, eq2) + f2

    n1 = system.joints[cycle[0]].name
    system.sols[s1p].mech_actions[n1] = MechanicalAction(n13, p2, -t3)
    system.sols[s1].mech_actions[n1] = MechanicalAction(-n13, p2, t3)

    n2 = system.joints[cycle[1]].name
    system.sols[s12].mech_actions[n2] = MechanicalAction(f1, p1, 0.)
    system.sols[s11].mech_actions[n2] = MechanicalAction(-f1, p1, 0.)

    n3 = system.joints[cycle[2]].name
    system.sols[s22].mech_actions[n3] = MechanicalAction(-f2, p2, 0)
    system.sols[s23].mech_actions[n3] = MechanicalAction(f2, p2, 0)


def g_g_p(system, cycle, pri1, pri2, rev, _, eq2, eq3):
    (s1p, _, _), (s1, a1, _) = pri1
    (s2, a2, _), (s2p, _, _) = pri2
    (s32, _), (s33, p33) = rev

    p = get_point(system, s33, p33)
    ux, uy = unit(system.get_ref(s1) + a1 + np.pi * .5), unit(system.get_ref(s2) + a2 + np.pi * .5)
    n13, n12 = mat_mul_n(inv_mat(ux, uy), -trd(system, eq3 + eq2))
    n13, n12 = n13 * ux, n12 * uy
    m31, m21 = tmd(system, p, eq3), tmd(system, p, eq2)
    f32 = trd(system, eq3) + n13

    n1 = system.joints[cycle[0]].name
    system.sols[s1p].mech_actions[n1] = MechanicalAction(n13, p, -m31)
    system.sols[s1].mech_actions[n1] = MechanicalAction(-n13, p, m31)

    n2 = system.joints[cycle[1]].name
    system.sols[s2].mech_actions[n2] = MechanicalAction(-n12, p, m21)
    system.sols[s2p].mech_actions[n2] = MechanicalAction(n12, p, -m21)

    n3 = system.joints[cycle[2]].name
    system.sols[s33].mech_actions[n3] = MechanicalAction(-f32, p, 0)
    system.sols[s32].mech_actions[n3] = MechanicalAction(f32, p, 0)


def p_g_g(system, cycle, rev, pri1, pri2, _, eq2, eq3):
    (s13, _), (s11, p11) = rev
    (s1, a1, _), (s1p, _, _) = pri1
    (s2, a2, _), (s2p, _, _) = pri2

    p = get_point(system, s11, p11)
    ux, uy = unit(system.get_ref(s1) + a1 + np.pi * .5), unit(system.get_ref(s2) + a2 + np.pi * .5)
    n21, n23 = mat_mul_n(inv_mat(ux, uy), trd(system, eq2))
    n21, n23 = ux * n21, uy * n23
    f31 = trd(system, eq3) + n23
    m21 = tmd(system, p, eq2 + eq3)
    m23 = tmd(system, p, eq2) - m21

    n1 = system.joints[cycle[0]].name
    system.sols[s11].mech_actions[n1] = MechanicalAction(f31, p, 0)
    system.sols[s13].mech_actions[n1] = MechanicalAction(-f31, p, 0)

    n2 = system.joints[cycle[1]].name
    system.sols[s1].mech_actions[n2] = MechanicalAction(n21, p, m21)
    system.sols[s1p].mech_actions[n2] = MechanicalAction(-n21, p, -m21)

    n3 = system.joints[cycle[2]].name
    system.sols[s2].mech_actions[n3] = MechanicalAction(-n23, p, -m23)
    system.sols[s2p].mech_actions[n3] = MechanicalAction(n23, p, m23)


def sp_p_1(system, cycle, pin, rev, _, eq2):
    (s1, a1, _), (s11, p11) = pin
    (s21, _), (s22, p22) = rev

    p1, p2 = get_point(system, s11, p11), get_point(system, s22, p22)
    u = unit(system.get_ref(s1) + a1)
    n12 = z_cross(u) * tmd(system, p2, eq2) / dot(p2 - p1, u)
    f12 = -trd(system, eq2) - n12

    n1 = system.joints[cycle[0]].name
    system.sols[s11].mech_actions[n1] = MechanicalAction(-n12, p1, 0)
    system.sols[s1].mech_actions[n1] = MechanicalAction(n12, p1, 0)

    n2 = system.joints[cycle[1]].name
    system.sols[s21].mech_actions[n2] = MechanicalAction(-f12, p2, 0)
    system.sols[s22].mech_actions[n2] = MechanicalAction(f12, p2, 0)


def sp_p_2(system, cycle, pin, rev, _, eq2):
    (s12, p12), (s1, a1, _) = pin
    (s21, _), (s22, p22) = rev

    p1, p2 = get_point(system, s12, p12), get_point(system, s22, p22)
    u = unit(system.get_ref(s1) + a1)
    n12 = z_cross(u) * tmd(system, p2, eq2) / dot(p2 - p1, u)
    f12 = -trd(system, eq2) - n12

    n1 = system.joints[cycle[0]].name
    system.sols[s12].mech_actions[n1] = MechanicalAction(n12, p1, 0)
    system.sols[s1].mech_actions[n1] = MechanicalAction(-n12, p1, 0)

    n2 = system.joints[cycle[1]].name
    system.sols[s21].mech_actions[n2] = MechanicalAction(-f12, p2, 0)
    system.sols[s22].mech_actions[n2] = MechanicalAction(f12, p2, 0)


def sp_p(system, cycle, pin, rev, eq1, eq2):
    return (sp_p_1, sp_p_2)[len(pin[0]) == 2](system, cycle, pin, rev, eq1, eq2)


def sp_g_1(system, cycle, pin, pri, _, eq2):
    (s1, a1, _), (s11, p11) = pin
    (s2, a2, _), (s2p, _, _) = pri

    p = get_point(system, s11, p11)
    ux, uy = unit(system.get_ref(s1) + a1 + np.pi * .5), unit(system.get_ref(s2) + a2 + np.pi * .5)
    n21sp, n21g = mat_mul_n(inv_mat(ux, uy), trd(system, eq2))
    n21sp, n21g = n21sp * ux, n21g * uy
    m21 = tmd(system, p, eq2)

    n1 = system.joints[cycle[0]].name
    system.sols[s1].mech_actions[n1] = MechanicalAction(-n21sp, p, 0)
    system.sols[s11].mech_actions[n1] = MechanicalAction(n21sp, p, 0)

    n2 = system.joints[cycle[1]].name
    system.sols[s2].mech_actions[n2] = MechanicalAction(n21g, p, m21)
    system.sols[s2p].mech_actions[n2] = MechanicalAction(-n21g, p, -m21)


def sp_g_2(system, cycle, pin, pri, _, eq2):
    (s12, p12), (s1, a1, _) = pin
    (s2, a2, _), (s2p, _, _) = pri

    p = get_point(system, s12, p12)
    ux, uy = unit(system.get_ref(s1) + a1 + np.pi * .5), unit(system.get_ref(s2) + a2 + np.pi * .5)
    n21sp, n21g = mat_mul_n(inv_mat(ux, uy), trd(system, eq2))
    n21sp, n21g = n21sp * ux, n21g * uy
    m21 = tmd(system, p, eq2)

    n1 = system.joints[cycle[0]].name
    system.sols[s12].mech_actions[n1] = MechanicalAction(-n21sp, p, 0)
    system.sols[s1].mech_actions[n1] = MechanicalAction(n21sp, p, 0)

    n2 = system.joints[cycle[1]].name
    system.sols[s2].mech_actions[n2] = MechanicalAction(n21g, p, m21)
    system.sols[s2p].mech_actions[n2] = MechanicalAction(-n21g, p, -m21)


def sp_g(system, cycle, pin, pri, eq1, eq2):
    (sp_p_1, sp_p_2)[len(pin[0]) == 2](system, cycle, pin, pri, eq1, eq2)


def t_p(system, cycle, rec, rev, _, eq2):
    (s2, _), (s1, _) = rec
    (s11, p11), (s12, _) = rev

    p = get_point(system, s11, p11)
    m21 = tmd(system, p, eq2)
    f21 = trd(system, eq2)

    n1 = system.joints[cycle[0]].name
    system.sols[s1].mech_actions[n1] = MechanicalAction(np.array((0., 0.)), p, m21)
    system.sols[s2].mech_actions[n1] = MechanicalAction(np.array((0., 0.)), p, -m21)

    n2 = system.joints[cycle[1]].name
    system.sols[s11].mech_actions[n2] = MechanicalAction(f21, p, 0)
    system.sols[s12].mech_actions[n2] = MechanicalAction(-f21, p, 0)


def block(system, index, eq1s1, eq2s2):
    system.joints[index].block(system, eq1s1, eq2s2)


dyn = {
    'P_P_P': p_p_p,
    'P_P_G': p_p_g,
    'G_P_P': g_p_p,
    'G_G_P': g_g_p,
    'P_G_G': p_g_g,
    'SP_P': sp_p,
    'SP_G': sp_g,
    'T_P': t_p,
    'Block': block
}
