from kinepy.geometry import *
from kinepy.interactions import MechanicalAction


def tmd(system, point, eq) -> np.ndarray:  # Intertie - somme(Moments connus) = Moments inconnus
    return -sum((sum(am.babar(point) for am in system.sols[s].mech_actions.values()) for s in eq), np.array((0.,)))


def trd(system, eq) -> np.ndarray:  # Intertie - somme(Forces connues) = Forces inconnues
    return -sum((sum(am.f for am in system.sols[s].mech_actions.values()) for s in eq), np.array((0., 0.)))


def p_p_p(system, cycle, rev1, rev2, rev3, _, eq2, eq3):
    (s13, _), (s11, p11) = rev1
    (s21, _), (s22, p22) = rev2
    (s32, _), (s33, p33) = rev3
    p1, p2, p3 = get_point(system, s11, p11), get_point(system, s22, p22), get_point(system, s33, p33)
    p1p2, p3p2 = p2 - p1, p2 - p3

    inv_12, inv_32 = inv_mag(p1p2), inv_mag(p3p2)
    u1, u2 = p1p2 * inv_12, p3p2 * inv_32

    #           f1/2 . (z ^ u1)                        f1/2 . (z ^ u2)
    x1, x2 = tmd(system, p1, eq2 + eq3) * inv_12, tmd(system, p3, eq2) * inv_32
    y1 = (x2 - dot(u1, u2) * x1) / det(u1, u2)

    f_12 = x1 * z_cross(u1) - y1 * u1
    f_32 = trd(system, eq2) - f_12
    f_13 = trd(system, eq3) + f_32

    p = system.joints[cycle[0]]
    system.sols[s11].mech_actions.append(MechanicalAction(-f_13, p1, 0.))
    system.sols[s13].mech_actions.append(MechanicalAction(f_13, p1, 0.))
    p.force = f_13 if s13 == p.s1 else -f_13

    p = system.joints[cycle[1]]
    system.sols[s21].mech_actions.append(MechanicalAction(-f_12, p2, 0.))
    system.sols[s22].mech_actions.append(MechanicalAction(f_12, p2, 0.))
    p.force = f_12 if s22 == p.s1 else -f_12

    p = system.joints[cycle[2]]
    system.sols[s33].mech_actions.append(MechanicalAction(-f_32, p3, 0.))
    system.sols[s32].mech_actions.append(MechanicalAction(f_32, p3, 0.))
    p.force = f_32 if s32 == p.s1 else -f_32


def p_p_g(system, cycle, rev1, rev2, pri, _, eq2, eq3):
    (s13, _), (s11, p11) = rev1
    (s21, _), (s22, p22) = rev2
    (s1, a1, _), (s1p, _, _) = pri

    p1, p2 = get_point(system, s11, p11), get_point(system, s22, p22)
    u = unit(system.get_ref(s1) + a1)
    n_23 = z_cross(u) * (normal := ((m_23 := tmd(system, eq3, p1)) + tmd(system, eq2, p2)) / dot(u, p1 - p2))
    f_13 = trd(system, eq3) - n_23
    f_12 = trd(system, eq2) + n_23

    p = system.joints[cycle[0]]
    system.sols[s13].mech_actions.append(MechanicalAction(f_13, p1, 0.))
    system.sols[s11].mech_actions.append(MechanicalAction(-f_13, p1, 0.))
    p.force = f_13 if s13 == p.s1 else -f_13

    p = system.joints[cycle[1]]
    system.sols[s21].mech_actions.append(MechanicalAction(-f_12, p2, 0.))
    system.sols[s22].mech_actions.append(MechanicalAction(f_12, p2, 0.))
    p.force = f_12 if s22 == p.s1 else -f_12

    g = system.joints[cycle[2]]
    p = system.get_origin(g.s1) + (g.d1 - g.d1p) * unit(system.get_ref(g.s1) + g.a1 + np.pi * .5)
    m_23 += det(p1 - p, n_23)
    system.sols[s1].mech_actions.append(MechanicalAction(-n_23, p, -m_23))
    system.sols[s1p].mech_actions.append(MechanicalAction(n_23, p, m_23))
    g.normal, g.torque = (normal, m_23) if s1p == g.s1 else (-normal, -m_23)


def g_p_p(system, cycle, pri, rev1, rev2, _, eq2, eq3):
    (s1p, _, _), (s1, a1, _) = pri
    (s11, p11), (s12, _) = rev1
    (s22, p22), (s23, _) = rev2

    p1, p2 = get_point(system, s11, p11), get_point(system, s22, p22)
    u = unit(system.get_ref(s1) + a1)
    n_13 = z_cross(u) * (normal := ((m_13 := tmd(system, eq3 + eq2, p1)) - tmd(system, eq3, p2)) / dot(u, p2 - p1))
    f_23 = trd(system, eq3) - n_13
    f_12 = trd(system, eq2) + f_23

    g = system.joints[cycle[0]]
    p = system.get_origin(g.s1) + (g.d1 - g.d1p) * unit(system.get_ref(g.s1) + g.a1 + np.pi * .5)
    m_13 += det(p1 - p, n_13)
    system.sols[s1].mech_actions.append(MechanicalAction(-n_13, p, -m_13))
    system.sols[s1p].mech_actions.append(MechanicalAction(n_13, p, m_13))
    g.normal, g.torque = (normal, m_13) if s1p == g.s1 else (-normal, -m_13)

    p = system.joints[cycle[1]]
    system.sols[s11].mech_actions.append(MechanicalAction(-f_12, p1, 0.))
    system.sols[s12].mech_actions.append(MechanicalAction(f_12, p1, 0.))
    p.force = f_12 if s12 == p.s1 else -f_12

    p = system.joints[cycle[2]]
    system.sols[s22].mech_actions.append(MechanicalAction(-f_23, p2, 0.))
    system.sols[s23].mech_actions.append(MechanicalAction(f_23, p2, 0.))
    p.force = f_23 if s23 == p.s1 else -f_23


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
