from kinepy.geometry import *
from kinepy.interactions import MechanicalAction, ZERO


def tmd(system, point, eq) -> np.ndarray:  # Intertie - somme(Moments connus) = Moments inconnus
    return np.array(-sum(sum(am.babar(point) for am in system.sols[s].mech_actions) for s in eq))


def trd(system, eq) -> np.ndarray:  # Intertie - somme(Forces connues) = Forces inconnues
    return np.array(-sum((sum(am.f for am in system.sols[s].mech_actions) for s in eq)))


def add_effort(j, value):
    j.__add_effort__(value)


def p_p_p(system, cycle, rev1, rev2, rev3, _, eq2, eq3):
    p1_, p2_, p3_ = (system.joints[i] for i in cycle)

    (s13, _), (s11, p11) = rev1
    (s21, p21), (s22, _) = rev2
    (s32, _), (s33, p33) = rev3
    p1, p2, p3 = get_point(p1_, 0), get_point(p2_, 0), get_point(p3_, 0)
    p1p2, p3p2 = p2 - p1, p2 - p3

    inv_12, inv_32 = inv_mag(p1p2), inv_mag(p3p2)
    u1, u2 = p1p2 * inv_12, p3p2 * inv_32

    #           f1/2 . (z ^ u1)                        f1/2 . (z ^ u2)
    x1, x2 = tmd(system, p1, eq2 + eq3) * inv_12, tmd(system, p3, eq2) * inv_32
    y1 = (x2 - dot(u1, u2) * x1) / det(u1, u2)

    f_12 = x1 * z_cross(u1) - y1 * u1
    f_32 = trd(system, eq2) - f_12
    f_13 = trd(system, eq3) + f_32

    system.sols[s11].mech_actions.append(MechanicalAction(-f_13, p1, 0.))
    system.sols[s13].mech_actions.append(MechanicalAction(f_13, p1, 0.))
    p1_.force_ = f_13 if p11 else -f_13

    system.sols[s21].mech_actions.append(MechanicalAction(-f_12, p2, 0.))
    system.sols[s22].mech_actions.append(MechanicalAction(f_12, p2, 0.))
    p2_.force_ = f_12 if p21 else -f_12

    system.sols[s33].mech_actions.append(MechanicalAction(-f_32, p3, 0.))
    system.sols[s32].mech_actions.append(MechanicalAction(f_32, p3, 0.))
    p3_.force_ = f_32 if p33 else -f_32


def p_p_g(system, cycle, rev1, rev2, pri, _, eq2, eq3):
    p1_, p2_, g = (system.joints[i] for i in cycle)
    (s13, _), (s11, p11) = rev1
    (s21, p21), (s22, _) = rev2
    (s1, s1_), (s1p, ) = pri

    p1, p2 = get_point(p1_, 0), get_point(p2_, 0)
    u = get_unit(g, 0)
    n_23 = z_cross(u) * (normal := ((m_23 := tmd(system, p1, eq3)) + tmd(system, p2, eq2)) / dot(u, p1 - p2))
    f_13 = trd(system, eq3) - n_23
    f_12 = trd(system, eq2) + n_23

    system.sols[s13].mech_actions.append(MechanicalAction(f_13, p1, 0.))
    system.sols[s11].mech_actions.append(MechanicalAction(-f_13, p1, 0.))
    p1_.force_ = f_13 if p11 else -f_13

    system.sols[s21].mech_actions.append(MechanicalAction(-f_12, p2, 0.))
    system.sols[s22].mech_actions.append(MechanicalAction(f_12, p2, 0.))
    p2_.force_ = f_12 if p21 else -f_12

    p = get_point(g, 0)
    m_23 += det(p1 - p, n_23)
    system.sols[s1].mech_actions.append(MechanicalAction(-n_23, p, -m_23))
    system.sols[s1p].mech_actions.append(MechanicalAction(n_23, p, m_23))
    g.normal_, g.torque_ = (normal, m_23) if s1_ else (-normal, -m_23)


def g_p_p(system, cycle, pri, rev1, rev2, _, eq2, eq3):
    g, p1_, p2_ = (system.joints[i] for i in cycle)
    (s1p, _), (s1, s1_) = pri
    (s11, p11), (s12, _) = rev1
    (s22, p22), (s23, _) = rev2

    p1, p2 = get_point(p1_, 0), get_point(p2_, 0)
    u = get_unit(g, 0)
    n_13 = z_cross(u) * (normal := ((m_13 := tmd(system, p1, eq3 + eq2)) - tmd(system, p2, eq3)) / dot(u, p2 - p1))
    f_23 = trd(system, eq3) - n_13
    f_12 = trd(system, eq2) + f_23

    p = get_point(g, 0)
    m_13 += det(p1 - p, n_13)
    system.sols[s1].mech_actions.append(MechanicalAction(-n_13, p, -m_13))
    system.sols[s1p].mech_actions.append(MechanicalAction(n_13, p, m_13))
    g.normal_, g.torque_ = (normal, m_13) if s1_ else (-normal, -m_13)

    system.sols[s11].mech_actions.append(MechanicalAction(-f_12, p1, 0.))
    system.sols[s12].mech_actions.append(MechanicalAction(f_12, p1, 0.))
    p1_.force_ = f_12 if p11 else -f_12

    system.sols[s22].mech_actions.append(MechanicalAction(-f_23, p2, 0.))
    system.sols[s23].mech_actions.append(MechanicalAction(f_23, p2, 0.))
    p2_.force_ = f_23 if p22 else -f_23


def g_g_p(system, cycle, pri1, pri2, rev, _, eq2, eq3):
    g1, g2, p_ = (system.joints[i] for i in cycle)
    (s1p, _), (s1, s1_) = pri1
    (s2, s2_), (s2p, _) = pri2
    (s32, p32), (s33, _) = rev

    p1 = get_point(p_, 0)
    ux, uy = z_cross(get_unit(g1, 0)), z_cross(get_unit(g2, 0))
    normal_13, normal_12 = mat_mul_n(inv_mat(ux, uy), trd(system, eq3 + eq2))
    n_13, n_12 = normal_13 * ux, normal_12 * uy
    f_23 = trd(system, eq3) - n_13

    p = get_point(g1, 0)
    m_13 = tmd(system, p, eq3) - det(p1 - p, f_23)
    system.sols[s1p].mech_actions.append(MechanicalAction(n_13, p, m_13))
    system.sols[s1].mech_actions.append(MechanicalAction(-n_13, p, -m_13))
    g1.normal_, g1.torque_ = (normal_13, m_13) if s1_ else (-normal_13, -m_13)

    p = get_point(g2, 0)
    m_12 = tmd(system, p, eq2) + det(p1 - p, f_23)
    system.sols[s2p].mech_actions.append(MechanicalAction(n_12, p, m_12))
    system.sols[s2].mech_actions.append(MechanicalAction(-n_12, p, -m_12))
    g2.normal_, g2.torque_ = (normal_12, m_12) if s2_ else (-normal_12, -m_12)

    system.sols[s32].mech_actions.append(MechanicalAction(-f_23, p1, 0.))
    system.sols[s33].mech_actions.append(MechanicalAction(f_23, p1, 0.))
    p_.force_ = f_23 if p32 else -f_23


def p_g_g(system, cycle, rev, pri1, pri2, _, eq2, eq3):
    p_, g1, g2 = (system.joints[i] for i in cycle)
    (s13, _), (s11, p11) = rev
    (s1, s1_), (s1p, _) = pri1
    (s2, s2_), (s2p, _) = pri2

    p1 = get_point(p_, 0)
    ux, uy = z_cross(get_unit(g1, 0)), z_cross(get_unit(g2, 0))
    normal_12, normal_32 = mat_mul_n(inv_mat(ux, uy), trd(system, eq2))
    n_12, n_32 = ux * normal_12, uy * normal_32
    f_13 = trd(system, eq3) + n_32

    system.sols[s13].mech_actions.append(MechanicalAction(f_13, p1, 0.))
    system.sols[s11].mech_actions.append(MechanicalAction(-f_13, p1, 0.))
    p_.force_ = f_13 if p11 else -f_13

    p = get_point(g1, 0)
    m_12 = tmd(system, p, eq2 + eq3) - det(p1 - p, f_13)
    system.sols[s1p].mech_actions.append(MechanicalAction(n_12, p, m_12))
    system.sols[s1].mech_actions.append(MechanicalAction(-n_12, p, -m_12))
    g1.normal_, g1.torque_ = (normal_12, m_12) if s1_ else (-normal_12, -m_12)

    p = get_point(g2, 0)
    m_23 = tmd(system, p, eq3) - det(p1 - p, f_13)
    system.sols[s2p].mech_actions.append(MechanicalAction(-n_32, p, m_23))
    system.sols[s2].mech_actions.append(MechanicalAction(n_32, p, -m_23))
    g2.normal_, g2.torque_ = (-normal_32, m_23) if s2_ else (normal_32, -m_23)


def sp_p(system, cycle, pin, rev, _, eq2):
    sp, p = (system.joints[i] for i in cycle)
    (s1p, _), (s1, s1_) = pin
    (s21, p21), (s22, _) = rev

    p1, p2 = get_point(sp, 1), get_point(p, 0)
    u = get_unit(sp, 0)
    n_12 = z_cross(u) * (normal_12 := tmd(system, p2, eq2) / dot(p1 - p2, u))
    f_12 = trd(system, eq2) - n_12

    system.sols[s1].mech_actions.append(MechanicalAction(-n_12, p1, 0))
    system.sols[s1p].mech_actions.append(MechanicalAction(n_12, p1, 0))
    sp.normal_ = normal_12 if s1_ else -normal_12

    system.sols[s21].mech_actions.append(MechanicalAction(-f_12, p2, 0.))
    system.sols[s22].mech_actions.append(MechanicalAction(f_12, p2, 0.))
    p.force_ = f_12 if p21 else -f_12


def sp_g(system, cycle, pin, pri, _, eq2):
    sp, g = (system.joints[i] for i in cycle)
    (s1p, _), (s1, s1_) = pin
    (s2, s2_), (s2p, _) = pri

    p1 = get_point(sp, 1)
    ux, uy = z_cross(get_unit(sp, 0)), z_cross(get_unit(g, 0))
    normal_12sp, normal_12g = mat_mul_n(inv_mat(ux, uy), trd(system, eq2))
    n_12sp, n_12g = normal_12sp * ux, normal_12g * uy

    system.sols[s1].mech_actions.append(MechanicalAction(-n_12sp, p1, 0))
    system.sols[s1p].mech_actions.append(MechanicalAction(n_12sp, p1, 0))
    sp.normal_ = normal_12sp if s1_ else -normal_12sp

    p = get_point(g, 0)
    m_12 = tmd(system, p, eq2) - det(p1 - p, n_12sp)
    system.sols[s2p].mech_actions.append(MechanicalAction(n_12g, p, m_12))
    system.sols[s2].mech_actions.append(MechanicalAction(-n_12g, p, -m_12))
    g.normal_, g.torque_ = (normal_12g, m_12) if s2_ else (-normal_12g, -m_12)


def t_p(system, cycle, rec, rev, _, eq2):
    t, p = (system.joints[i] for i in cycle)
    (s2, _), (s1, s1_) = rec
    (s11, p11), (s12, _) = rev

    p1 = get_point(p, 0)
    m_12 = tmd(system, p1, eq2)
    f_12 = trd(system, eq2)

    system.sols[s1].mech_actions.append(MechanicalAction(ZERO, p1, -m_12))
    system.sols[s2].mech_actions.append(MechanicalAction(ZERO, p1, m_12))
    t.torque_ = m_12 if s1_ else -m_12

    system.sols[s11].mech_actions.append(MechanicalAction(-f_12, p1, 0))
    system.sols[s12].mech_actions.append(MechanicalAction(f_12, p1, 0))
    p.force_ = f_12 if p11 else -f_12


def block(system, index, eq1s1, eq2s2):
    system.joints[index].block(eq1s1, eq2s2)


def rel_block(system, rel, index, eq1s1, eq2s2):
    system.relations[rel].rel_block(index, eq1s1, eq2s2)


dyn = {
    'P_P_P': p_p_p,
    'P_P_G': p_p_g,
    'G_P_P': g_p_p,
    'G_G_P': g_g_p,
    'P_G_G': p_g_g,
    'SP_P': sp_p,
    'SP_G': sp_g,
    'T_P': t_p,
    'Block': block,
    'RelBlock': rel_block
}
