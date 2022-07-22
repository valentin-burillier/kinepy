from kinepy.geometry import *


#  ---------------------------------------------------- P P P ----------------------------------------------------------

def p_p_p(system, cycle, rev1, rev2, rev3):
    sgn = system.signs[cycle]
    (s13, p13), (s11, p11) = rev1
    (s21, p21), (s22, p22) = rev2
    (s32, p32), (s33, p33) = rev3

    p11 = get_point(system, s11, p11)
    p13 = get_point(system, s13, p13)
    p21 = get_point(system, s21, p21)
    p22 = get_point(system, s22, p22)
    p32 = get_point(system, s32, p32)
    p33 = get_point(system, s33, p33)

    v12 = p21 - p11
    v32 = p22 - p32
    v13 = p33 - p13
    
    sq_a, sq_b, sq_c = sq_mag(v12), sq_mag(v13), sq_mag(v32)
    inv_a, inv_b, inv_c = sq_a ** -.5, sq_b ** -.5, sq_c ** -.5
    alpha = sgn * np.arccos(0.5 * (sq_a + sq_b - sq_c) * inv_a * inv_b)

    theta = get_angle2(v12, inv_a) + alpha
    gamma = theta - get_angle2(v13, inv_b)
    mat = rot(gamma)
    change_ref(system, s13, gamma, mat, p13, p11)

    p33 = p11 + unit(theta) / inv_b
    gamma = get_angle(p21 - p33) - get_angle2(v32, inv_c)
    mat = rot(gamma)
    change_ref(system, s22, gamma, mat, p32, p33)
    
    for p in cycle:
        p = system.joints[p]
        p.angle = system.get_ref(p.s2) - system.get_ref(p.s1)
        make_continuous(p.angle)
    
    return sum((system.eqs[s] for s in (s11, s22, s33)), ())
    

#  ---------------------------------------------------- G P P ----------------------------------------------------------

def g_p_p(system, cycle, pri, rev2, rev3):
    (s1p, alpha1p, d1p), (s1, alpha1, d1) = pri
    (s21, p21), (s22, p22) = rev2
    (s32, p32), (s33, p33) = rev3

    sgn = system.signs[cycle] * (2 * (s1 == system.joints[cycle[0]].s1) - 1)
    
    gamma = system.get_ref(s1) + alpha1 - system.get_ref(s1p) - alpha1p
    change_ref2(system, s1p, gamma, rot(gamma), get_point(system, s33, p33))

    p21, p22 = get_point(system, s21, p21), get_point(system, s22, p22)
    v = p21 - system.get_origin(s1) - (d1 - d1p) * z_cross(unit(system.get_ref(s1) + alpha1)) + system.get_origin(s1p)
    # v1 + rot(...) . v3

    v2 = p22 - get_point(system, s32, p32)
    mat_mul_r(rot(-system.get_ref(s1) - alpha1), v)  # v <- (X0; y)
    
    sq_z = sq_mag(v2)
    inv_z = sq_z ** -.5
    dx = sgn * (sq_z - v[1] ** 2) ** .5
    
    theta2 = np.arccos(-sgn * (1 - (v[1] * inv_z) ** 2) ** .5) * (2 * (v[1] >= 0) - 1)
    theta3 = get_angle2(v2, inv_z)
    
    gamma = theta2 + alpha1 + system.get_ref(s1) - theta3
    change_ref(system, s22, gamma, rot(gamma), p22, p21)
    trans(system, s1p, get_point(system, s32, p32))

    for p in cycle[1:]:
        p = system.joints[p]
        p.angle = system.get_ref(p.s2) - system.get_ref(p.s1)
        make_continuous(p.angle)
    pri = system.joints[cycle[0]]
    pri.delta = (v[0, :] + dx) * (2 * (s1 == pri.s1) - 1)
    
    return sum((system.eqs[s] for s in (s1, s22, s33)), ())


#  ---------------------------------------------------- P P G ----------------------------------------------------------

def p_p_g(system, cycle, rev1, rev2, pri):
    (s13, p13), (s11, p11) = rev1
    (s21, p21), (s22, p22) = rev2
    (s1, alpha1, d1), (s1p, alpha1p, d1p) = pri

    sgn = system.signs[cycle] * (2 * (s1 == system.joints[cycle[2]].s1) - 1)
    
    v1 = get_point(system, s21, p21) - get_point(system, s11, p11)
    v2 = get_point(system, s22, p22) - system.get_origin(s1) + \
        (d1p - d1) * unit(np.pi * .5 + system.get_ref(s1) + alpha1 - system.get_ref(s1p) - alpha1p)
    v = system.get_origin(s1p) - get_point(system, s13, p13)  # = v3
    
    mat_mul_r(rot(system.get_ref(s1) + alpha1 - system.get_ref(s1p) - alpha1p), v)
    v += v2
    
    mat_mul_r(rot(-system.get_ref(s1) - alpha1), v)  # v <- (X0; y)
    sq_z = sq_mag(v1)
    inv_z = sq_z ** -.5
    dx = sgn * (sq_z - v[1] ** 2) ** .5
    theta2 = (2 * (v[1] > 0) - 1) * np.arccos(-sgn * (1 - (v[1] * inv_z) ** 2) ** .5)
    theta3 = get_angle2(v1, inv_z)
    
    gamma = theta3 - theta2 - system.get_ref(s1) - alpha1
    change_ref(system, s22, gamma, rot(gamma), get_point(system, s22, p22), get_point(system, s21, p21))
    gamma = theta3 - theta2 - system.get_ref(s1p) - alpha1p
    change_ref(system, s13, gamma, rot(gamma), get_point(system, s13, p13), get_point(system, s11, p11))
    
    for p in cycle[:2]:
        p = system.joints[p]
        p.angle = system.get_ref(p.s2) - system.get_ref(p.s1)
    pri = system.joints[cycle[2]]
    pri.delta = (v[0] + dx) * (2 * (s1 == pri.s1) - 1)
    
    return sum((system.eqs[s] for s in (s11, s22, s1p)), ())


#  ---------------------------------------------------- P G G ----------------------------------------------------------
    
def p_g_g(system, cycle, rev, pri1, pri2):
    (s13, p13), (s11, p11) = rev
    (s1, alpha1, d1), (s1p, alpha1p, d1p) = pri1
    (s2, alpha2, d2), (s2p, alpha2p, d2p) = pri2
    
    gamma2 = (gamma1 := system.get_ref(s1) + alpha1 - system.get_ref(s1p) - alpha1p) + \
        system.get_ref(s2) + alpha2 - system.get_ref(s2p) - alpha2p
    
    change_ref2(system, s1p, gamma1, rot(gamma1), system.get_origin(s1p))
    change_ref(system, s13, gamma2, rot(gamma2), get_point(system, s13, p13), get_point(system, s11, p11))
    
    ux, uy = unit(system.get_ref(s1) + alpha1), unit(system.get_ref(s2) + alpha2)
    v1 = get_point(system, s11, p11) - system.get_origin(s1) - (d1 - d1p) * z_cross(ux)
    v2 = system.get_origin(s1p) - system.get_origin(s1p) - (d2 - d2p) * z_cross(uy)
    v3 = system.get_origin(s2p) - get_point(system, s13, p13)
    
    x, y = mat_mul_n(inv_mat(ux, uy), v1 + v2 + v3)
    
    trans(system, s1p, system.get_origin(s1) + (d1 - d1p) * z_cross(ux) + x * ux)
    
    g1 = system.joints[cycle[1]]
    g1.delta = x * (2 * (s1 == g1.s1) - 1)
    g2 = system.joints[cycle[2]]
    g2.delta = y * (2 * (s2 == g2.s1) - 1)
    p = system.joints[cycle[0]]
    p.angle = system.get_ref(p.s2) - system.get_ref(p.s1)
    
    return sum((system.eqs[s] for s in (s11, s1p, s2p)), ())


#  ---------------------------------------------------- G G P ----------------------------------------------------------

def g_g_p(system, cycle, pri1, pri2, rev):
    (s1p, alpha1p, d1p), (s1, alpha1, d1) = pri1
    (s2, alpha2, d2), (s2p, alpha2p, d2p) = pri2
    (s12, p12), (s13, p13) = rev
    
    gamma1, gamma2 = (
        system.get_ref(s1) + alpha1 - system.get_ref(s1p) - alpha1p,
        system.get_ref(s2) + alpha2 - system.get_ref(s2p) - alpha2p
    )
    
    change_ref2(system, s2p, gamma2, rot(gamma2), get_point(system, s12, p12))
    change_ref2(system, s1p, gamma1, rot(gamma1), system.get_ref(s1p))
    
    ux, uy = unit(system.get_ref(s1) + alpha1), unit(system.get_ref(s2) + alpha2)
    
    v1 = system.get_origin(s2) + (d2 - d2p) * z_cross(uy) - system.get_origin(s1) - (d1 - d1p) * z_cross(ux)
    v2 = -system.get_origin(s2p)
    v3 = -get_point(system, s13, p13)
    
    x, y = mat_mul_n(inv_mat(ux, uy), v1 + v2 + v3)
    
    trans(system, s1p, system.get_origin(s1) + (d1 - d1p) * z_cross(ux) + x * ux)
    trans(system, s2p, get_point(system, s13, p13))
    
    g1 = system.joints[cycle[0]]
    g1.delta = x * (2 * (s1 == g1.s1) - 1)
    g2 = system.joints[cycle[1]]
    g2.delta = y * (2 * (s2 == g2.s2) - 1)
    p = system.joints[cycle[2]]
    p.angle = system.get_ref(p.s2) - system.get_ref(p.s1)
    make_continuous(p.angle)
    
    return sum((system.eqs[s] for s in (s1, s1p, s2p)), ())
   

#  ------------------------------------------------ SP G ---------------------------------------------------------------

def sp_g_1(system, cycle, pin, pri):
    (s12, p12), (s, alpha, d) = pin
    (s1, alpha1, d1), (s1p, alpha1p, d1p) = pri
    
    gamma = system.get_ref(s1) + alpha1 - system.get_ref(s1p) - alpha1p
    change_ref2(system, s1p, gamma, rot(gamma), system.get_origin(s1p))
    
    ux, uy = unit(alpha + system.get_ref(s)), unit(alpha1 + system.get_ref(s1))
    
    v1 = system.get_origin(s) + d * z_cross(ux) - system.get_origin(s1) - (d1 - d1p) * z_cross(uy)
    v2 = -get_point(system, s12, p12)
    
    x, y = mat_mul_n(inv_mat(ux, uy), v1 + v2)
    trans(system, s1p, system.get_origin(s1) + (d1 - d1p) * z_cross(uy) + y * uy)
    
    sp = system.joints[cycle[0]]
    sp.delta = -x
    sp.angle = system.get_ref(sp.s2) - system.get_ref(sp.s1)
    make_continuous(sp.angle)
    g = system.joints[cycle[1]]
    g.delta = y * (2 * (s1 == g.s1) - 1)
    
    return system.eqs[s] + system.eqs[s1p]


def sp_g_2(system, cycle, pin, pri):
    (s, alpha, d), (s11, p11) = pin
    (s1, alpha1, d1), (s1p, alpha1p, d1p) = pri
    
    gamma = system.get_ref(s1) + alpha1 - system.get_ref(s1p) - alpha1p
    change_ref2(system, s1p, gamma, rot(gamma), system.get_origin(s1p))
    
    ux, uy = unit(alpha + system.get_ref(s)), unit(alpha1 + system.get_ref(s1))
    
    v1 = system.get_origin(s1) + (d1 - d1p) * z_cross(uy) - get_point(system, s11, p11)
    v2 = system.get_origin(s) + d * ux
    
    x, y = mat_mul_n(inv_mat(ux, uy), v1 + v2)
    trans(system, s1p, system.get_origin(s1) + (d1 - d1p) * z_cross(uy) - y * uy)
    
    sp = system.joints[cycle[0]]
    sp.delta = -x
    sp.angle = system.get_ref(sp.s2) - system.get_ref(sp.s1)
    make_continuous(sp.angle)
    g = system.joints[cycle[1]]
    g.delta = y * (-2 * (s1 == g.s1) + 1)
    
    return system.eqs[s] + system.eqs[s1]


def sp_g(system, cycle, pin, pri):
    return (sp_g_1 if len(pin[0]) == 2 else sp_g_2)(system, cycle, pin, pri)


#  ---------------------------------------------------- T P ------------------------------------------------------------

def t_p(system, cycle, rec, rev):
    (s2, _), (s1, alpha) = rec
    (s11, p11), (s12, p12) = rev
    
    gamma = system.get_ref(s1) + alpha - system.get_ref(s2)
    change_ref(system, s12, gamma, rot(gamma), get_point(system, s12, p12), get_point(system, s11, p11))
    
    p = system.joints[cycle[1]]
    p.angle = system.get_ref(p.s2) - system.get_ref(p.s1)
    t = system.joints[cycle[0]]
    v, theta = system.get_origin(t.s2) - system.get_origin(t.s1), system.get_ref(t.s1)
    
    mat_mul_r(inv_mat(unit(t.base[0] + theta), unit(t.base[1] + theta)), v)
    t.delta = v[:, 0, :]
    
    return system.eqs[s1] + system.eqs[s2]
    

#  -------------------------------------------------- SP P -------------------------------------------------------------

def sp_p_1(system, cycle, pin, rev):
    sgn = system.signs[cycle]
    
    (s, alpha, d), (s11, p11) = pin
    (s21, p21), (s22, p22) = rev
    
    v1 = get_point(system, s21, p21) - get_point(system, s11, p11)
    v2 = get_point(system, s22, p22) - system.get_origin(s) - d * unit(alpha + system.get_ref(s) + np.pi * .5)
    mat_mul_r(rot(-alpha - system.get_ref(s)), v2)   # v2 <- (X0; y)
    
    sq_z = sq_mag(v1)
    inv_z = sq_z ** -.5
    
    dx = sgn * (sq_z - v2[1] ** 2) ** .5
    theta2 = (2 * (v2[1] > 0) - 1) * np.arccos(-sgn * (1 - (v2[1] * inv_z) ** 2) ** .5)
    theta1 = get_angle2(v1, inv_z)
    
    gamma = theta1 - theta2 - alpha - system.get_ref(s)
    change_ref(system, s, gamma, rot(gamma), get_point(system, s22, p22), get_point(system, s21, p21))
    
    pin = system.joints[cycle[0]]
    pin.delta = v2[0] + dx
    pin.angle = system.get_ref(pin.s2) - system.get_ref(pin.s1)
    make_continuous(pin.angle)
    p = system.joints[cycle[1]]
    p.angle = system.get_ref(p.s2) - system.get_ref(p.s1)
    make_continuous(p.angle)
    
    return system.eqs[s] + system.eqs[s21]


def sp_p_2(system, cycle, pin, rev):
    sgn = system.signs[cycle]
    
    (s12, p12), (s, alpha, d) = pin
    (s21, p21), (s22, p22) = rev
    
    v1 = get_point(system, s21, p21) - system.get_origin(s) - d * unit(np.pi * .5 + alpha + system.get_ref(s))
    v2 = get_point(system, s22, p22) - get_point(system, s12, p12)
    
    mat_mul_r(rot(-alpha - system.get_ref(s)), v1)  # v1 <- (X0; y)
    sq_z = sq_mag(v2)
    inv_z = sq_z ** - .5
    
    dx = sgn * (sq_z - v1[1] ** 2) ** .5
    theta2 = (2 * (v1[1] > 0) - 1) * np.arccos(-sgn * (1 - (v1[1] * inv_z) ** 2) ** .5)
    theta1 = get_angle2(v2, inv_z)
    
    gamma = theta2 + alpha + system.get_ref(s) - theta1
    change_ref(system, s12, gamma, rot(gamma), get_point(system, s22, p22), get_point(system, s21, p21))
    
    pin = system.joints[cycle[0]]
    pin.delta = v1[0] + dx
    pin.angle = system.get_ref(pin.s2) - system.get_ref(pin.s1)
    make_continuous(pin.angle)
    p = system.joints[cycle[1]]
    p.angle = system.get_ref(p.s2) - system.get_ref(p.s1)
    make_continuous(p.angle)

    return system.eqs[s] + system.eqs[s22]


def sp_p(system, cycle, pin, rev):
    return (sp_p_1 if len(pin[0]) == 3 else sp_p_2)(system, cycle, pin, rev)


def pilot(system, index):
    return system.joints[index].pilot(system, system.indices[index])


kin = {
    'P_P_P': p_p_p,
    'P_P_G': p_p_g,
    'G_P_P': g_p_p,
    'G_G_P': g_g_p,
    'P_G_G': p_g_g,
    'SP_P': sp_p,
    'SP_G': sp_g,
    'T_P': t_p,
    'Pilot': pilot
}
