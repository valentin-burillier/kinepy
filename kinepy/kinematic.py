from kinepy.geometry import *


#  ---------------------------------------------------- P P P ----------------------------------------------------------

def p_p_p(system, cycle, rev1, rev2, rev3):
    sgn = system.signs[cycle]

    p1, p2, p3 = (system.joints[i] for i in cycle)

    (s13, p13), (s11, p11) = rev1
    (s21, p21), (s22, p22) = rev2
    (s32, p32), (s33, p33) = rev3

    p11 = get_point(p1, p11)
    p13 = get_point(p1, p13)
    p21 = get_point(p2, p21)
    p22 = get_point(p2, p22)
    p32 = get_point(p3, p32)
    p33 = get_point(p3, p33)

    v12 = p21 - p11
    v32 = p22 - p32
    v13 = p33 - p13
    
    sq_a, sq_b, sq_c = sq_mag(v12), sq_mag(v13), sq_mag(v32)
    inv_a, inv_b, inv_c = sq_a ** -.5, sq_b ** -.5, sq_c ** -.5
    alpha = sgn * np.arccos(0.5 * (sq_a + sq_b - sq_c) * inv_a * inv_b)

    theta = angle2(v12, inv_a) + alpha
    gamma = theta - angle2(v13, inv_b)
    mat = rot(gamma)
    change_ref(system, s13, gamma, mat, p13, p11)

    p33 = p11 + unit(theta) / inv_b
    gamma = angle(p21 - p33) - angle2(v32, inv_c)
    mat = rot(gamma)
    change_ref(system, s22, gamma, mat, p32, p33)
    
    for p in (p1, p2, p3):
        p.angle_ = system.get_ref(p.s2) - system.get_ref(p.s1)
        make_continuous(p.angle_)
    
    return sum((system.eqs[s] for s in (s11, s22, s33)), ())
    

#  ---------------------------------------------------- G P P ----------------------------------------------------------

def g_p_p(system, cycle, pri, rev2, rev3):
    g, p2, p3 = (system.joints[i] for i in cycle)

    (s1p, s1p_), (s1, s1_) = pri
    (s21, p21), (s22, p22) = rev2
    (s32, p32), (s33, p33) = rev3

    sgn = system.signs[cycle] * (2 * s1p_ - 1)
    
    gamma = get_angle(g, s1_) - get_angle(g, s1p_)
    change_ref2(system, s1p, gamma, rot(gamma), get_point(p3, p33))

    p21, p22 = get_point(p2, p21), get_point(p2, p22)
    v = p21 - get_point(g, s1_) + system.get_origin(s1p)
    # v1 + rot(...) . v3

    v2 = p22 - get_point(p3, p32)
    mat_mul_r(rot(-get_angle(g, s1_)), v)  # v <- (X0; y)
    
    sq_z = sq_mag(v2)
    inv_z = sq_z ** -.5
    dx = sgn * (sq_z - v[1] ** 2) ** .5
    
    theta2 = np.arccos(-sgn * (1 - (v[1] * inv_z) ** 2) ** .5) * (2 * (v[1] >= 0) - 1)
    theta3 = angle2(v2, inv_z)
    
    gamma = theta2 + get_angle(g, s1_) - theta3
    change_ref(system, s22, gamma, rot(gamma), p22, p21)
    trans(system, s1p, get_point(p3, p32))

    for p in (p2, p3):
        p.angle_ = system.get_ref(p.s2) - system.get_ref(p.s1)
        make_continuous(p.angle_)
    g.sliding_ = (v[0, :] + dx) * (2 * s1p_ - 1)
    
    return sum((system.eqs[s] for s in (s1, s22, s33)), ())


#  ---------------------------------------------------- P P G ----------------------------------------------------------

def p_p_g(system, cycle, rev1, rev2, pri):
    p1, p2, g = (system.joints[i] for i in cycle)

    (s13, p13), (s11, p11) = rev1
    (s21, p21), (s22, p22) = rev2
    (s1, s1_), (s1p, s1p_) = pri

    sgn = system.signs[cycle] * (2 * s1p_ - 1)
    
    v1 = get_point(p2, p21) - get_point(p1, p11)
    v2 = get_point(p2, p22) - get_point(g, s1_)
    v = system.get_origin(s1p) - get_point(p1, p13)  # = v3
    
    mat_mul_r(rot(get_angle(g, s1_) - get_angle(g, s1p_)), v)
    v += v2
    
    mat_mul_r(rot(-get_angle(g, s1_)), v)  # v <- (X0; y)
    sq_z = sq_mag(v1)
    inv_z = sq_z ** -.5
    dx = sgn * (sq_z - v[1] ** 2) ** .5
    theta2 = (2 * (v[1] > 0) - 1) * np.arccos(-sgn * (1 - (v[1] * inv_z) ** 2) ** .5)
    theta3 = angle2(v1, inv_z)
    
    gamma = theta3 - theta2 - get_angle(g, s1_)
    change_ref(system, s22, gamma, rot(gamma), get_point(p2, p22), get_point(p2, p21))
    gamma = theta3 - theta2 - get_angle(g, s1p_)
    change_ref(system, s13, gamma, rot(gamma), get_point(p1, p13), get_point(p1, p11))
    
    for p in (p1, p2):
        p.angle_ = system.get_ref(p.s2) - system.get_ref(p.s1)
        make_continuous(p.angle_)
    g.sliding_ = (v[0] + dx) * (2 * s1p_ - 1)
    
    return sum((system.eqs[s] for s in (s11, s22, s1p)), ())


#  ---------------------------------------------------- P G G ----------------------------------------------------------
    
def p_g_g(system, cycle, rev, pri1, pri2):
    p, g1, g2 = (system.joints[i] for i in cycle)

    (s13, p13), (s11, p11) = rev
    (s1, s1_), (s1p, s1p_) = pri1
    (s2, s2_), (s2p, s2p_) = pri2

    p11 = get_point(p, p11)

    gamma2 = (gamma1 := get_angle(g1, s1_) - get_angle(g1, s1p_)) + get_angle(g2, s2_) - get_angle(g2, s2_)
    
    change_ref2(system, s1p, gamma1, rot(gamma1), system.get_origin(s1p))
    change_ref(system, s13, gamma2, rot(gamma2), get_point(p, p13), p11)
    
    ux, uy = get_unit(g1, s1_), get_unit(g2, s2_)
    offset = get_dist(g1, s1_) * z_cross(ux)
    v1 = p11 - system.get_origin(s1) - offset
    v2 = system.get_origin(s1p) - system.get_origin(s2) - get_dist(g2, s2_) * z_cross(uy)
    v3 = system.get_origin(s2p) - get_point(p, p13)
    
    x, y = mat_mul_n(inv_mat(ux, uy), v1 + v2 + v3)
    
    trans(system, s1p, system.get_origin(s1) + offset + x * ux)

    g1.sliding_ = x * (2 * s1p_ - 1)
    g2.sliding_ = y * (2 * s2p_ - 1)
    p.angle_ = system.get_ref(p.s2) - system.get_ref(p.s1)
    make_continuous(p.angle_)

    return sum((system.eqs[s] for s in (s11, s1p, s2p)), ())


#  ---------------------------------------------------- G G P ----------------------------------------------------------

def g_g_p(system, cycle, pri1, pri2, rev):
    g1, g2, p1 = (system.joints[i] for i in cycle)

    (s1p, s1p_), (s1, s1_) = pri1
    (s2, s2_), (s2p, s2p_) = pri2
    (s12, p12), (s13, p13) = rev
    
    gamma1, gamma2 = get_angle(g1, s1_) - get_angle(g1, s1p_), get_angle(g2, s2_) - get_angle(g2, s2p_)
    
    change_ref2(system, s2p, gamma2, rot(gamma2), get_point(p1, p12))
    change_ref2(system, s1p, gamma1, rot(gamma1), system.get_ref(s1p))
    
    ux, uy = get_unit(g1, s1_), get_unit(g2, s2_)
    
    v1 = system.get_origin(s2) + get_dist(g2, s2_) * z_cross(uy) - \
        system.get_origin(s1) - get_dist(g1, s1_) * z_cross(ux)
    v2 = -system.get_origin(s2p)
    v3 = -get_point(p1, p13)
    
    x, y = mat_mul_n(inv_mat(ux, uy), v1 + v2 + v3)
    
    trans(system, s1p, system.get_origin(s1) + get_dist(g1, s1_) * z_cross(ux) + x * ux)
    trans(system, s2p, get_point(p1, p13))

    g1.sliding_ = x * (2 * s1p_ - 1)
    g2.sliding_ = y * (2 * s2p_ - 1)
    p1.angle_ = system.get_ref(p1.s2) - system.get_ref(p1.s1)
    make_continuous(p1.angle_)
    
    return sum((system.eqs[s] for s in (s1, s1p, s2p)), ())
   

#  ------------------------------------------------ SP G ---------------------------------------------------------------

def sp_g(system, cycle, pin, pri):
    sp, g = (system.joints[i] for i in cycle)

    (s0p, s0p_), (s0, s0_) = pin
    (s1, s1_), (s1p, s1p_) = pri

    gamma = get_angle(g, s1_) - get_angle(g, s1p_)
    change_ref(system, s1p, gamma, rot(gamma), system.get_origin(s1p), get_point(g, s1_))
    ux, uy = get_unit(sp, 0), get_unit(g, s1_)
    x, y = mat_mul_n(inv_mat(ux, uy), get_point(sp, s0_) - get_point(sp, s0p_))
    trans(system, s1, y * uy)

    g.sliding_ = y * (2 * s1p_ - 1)
    sp.sliding_ = x * (2 * s0_ - 1)
    sp.angle_ = system.get_ref(sp.s2) - system.get_ref(sp.s1)
    make_continuous(sp.angle_)
    return system.eqs[s0] + system.eqs[s1p]


#  ---------------------------------------------------- T P ------------------------------------------------------------

def t_p(system, cycle, rec, rev):
    t, p = (system.joints[i] for i in cycle)
    (s2, s2_), (s1, s1_) = rec
    (s11, p11), (s12, p12) = rev
    
    gamma = get_angle(t, s1) - system.get_ref(s2)
    change_ref(system, s12, gamma, rot(gamma), get_point(p, p12), get_point(p, p11))

    p.angle_ = system.get_ref(p.s2) - system.get_ref(p.s1)
    make_continuous(p.angle_)
    v, theta = system.get_origin(t.s2) - system.get_origin(t.s1), system.get_ref(t.s1)
    
    t.sliding_ = mat_mul_r(inv_mat(unit(t.base_[0] + theta), unit(t.base_[1] + theta)), v)
    return system.eqs[s1] + system.eqs[s2]
    

#  -------------------------------------------------- SP P -------------------------------------------------------------

def sp_p(system, cycle, pin, rev):
    sgn = system.signs[cycle]

    sp, p = (system.joints[i] for i in cycle)
    
    (s0p, s0p_), (s0, s0_) = pin
    (s21, p21), (s22, p22) = rev
    
    v1 = get_point(p, p21) - get_point(sp, s0_)
    v2 = get_point(p, p22) - get_point(sp, s0p_)

    v1, v2 = (v1, v2)[::(2 * s0_ - 1)]  # v1 = pp, v2 = sp
    mat_mul_r(rot(get_angle(sp, 0)), v2)   # v2 <- (X0; y)
    
    sq_z = sq_mag(v1)
    inv_z = sq_z ** -.5
    
    dx = sgn * (sq_z - v2[1] ** 2) ** .5
    theta2 = (2 * (v2[1] > 0) - 1) * np.arccos(-sgn * (1 - (v2[1] * inv_z) ** 2) ** .5)
    theta1 = angle2(v1, inv_z)
    
    gamma = (2 * s0_ - 1) * (theta1 - theta2 - get_angle(sp, 0))
    change_ref(system, s21, gamma, rot(gamma), get_point(p, p22), get_point(p, p21))

    sp.sliding_ = v2[0] + dx
    sp.angle_ = system.get_ref(sp.s2) - system.get_ref(sp.s1)
    make_continuous(sp.angle_)
    p.angle_ = system.get_ref(p.s2) - system.get_ref(p.s1)
    make_continuous(p.angle_)
    
    return system.eqs[s0] + system.eqs[s0p]


def pilot(system, joint, s_index):
    return system.joints[joint].pilot(system.indices[joint], s_index)


def rel_pilot(system, rel, j_index, s_index):
    return system.relations[rel].rel_pilot(j_index, s_index)


kin = {
    'P_P_P': p_p_p,
    'P_P_G': p_p_g,
    'G_P_P': g_p_p,
    'G_G_P': g_g_p,
    'P_G_G': p_g_g,
    'SP_P': sp_p,
    'SP_G': sp_g,
    'T_P': t_p,
    'Pilot': pilot,
    'RelPilot': rel_pilot
}
