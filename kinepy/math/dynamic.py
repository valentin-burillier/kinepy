from kinepy.math.geometry import np, det, dot, sq_mag, z_cross, unit, get_angle, get_zero, get_point, rvec
from kinepy.math.calculus import derivative2, derivative2_vec


def tmd(point, eq) -> np.ndarray:  # Intertie - somme(Moments connus) = Moments inconnus
    return np.array(-sum(s.babar(point) for s in eq))


def trd(eq) -> np.ndarray:  # Intertie - somme(Forces connues) = Forces inconnues
    return np.array(-sum(s.trd for s in eq))


def set_force(rev, b, force):  # Force en rev.point(), par (s1, s2)[b] sur l'autre
    p = get_point(rev, 0)
    rev.s1.add_mech_action(force * (-1, 1)[b], p, 0.)
    rev.s2.add_mech_action(force * (1, -1)[b], p, 0.)
    rev.force = force * (-1, 1)[b]


def set_normal(pri, b, normal, torque, u):  # Force normale en get_zero(pri, 0)
    p = get_zero(pri, 0, u)
    n_ = normal * (-1, 1)[b]
    pri.s1.add_mech_action(n_, p, torque * (-1, 1)[b])
    pri.s2.add_mech_action(-n_, p, torque * (1, -1)[b])
    pri.normal = det(u, n_)
    pri.torque = torque * (-1, 1)[b]


def group_tmd(eqs, indices, ref, point):
    # gets the sum of unknown values in the tmd of eqs described by indices
    key = 0
    for index in indices:
        key |= 1 << index
    mask = (key >> ref) & 1
    eq = sum((e for i, e in enumerate(eqs) if ((key >> i) & 1) ^ mask), ())
    return tmd(point, eq) * (1, -1)[mask]


def group_trd(eqs, indexes, ref):
    key = 0
    for index in indexes:
        key |= 1 << index
    mask = (key >> ref) & 1
    eq = sum((e for i, e in enumerate(eqs) if ((key >> i) & 1) ^ mask), ())
    return trd(eq) * (1, -1)[mask]


def solve_graph0(eqs, joints, ref):
    (P0, b0), (P1, b1), (P2, b2) = joints

    m10 = group_tmd(eqs, (0,), ref, get_point(P1, 0))
    m10_ = group_tmd(eqs, (0, 2), ref, get_point(P2, 0))

    vx, vy = get_point(P0, 0) - get_point(P1, 0), get_point(P0, 0) - get_point(P2, 0)
    inv_x, inv_y = sq_mag(vx) ** -.5, sq_mag(vy) ** -.5
    vx *= inv_x
    vy *= inv_y

    t10, t10_ = m10 * inv_x, m10_ * inv_y
    f10 = t10 * z_cross(vx) + vx * (t10_ - dot(vx, vy) * t10) / det(vy, vx)
    set_force(P0, b0, f10)

    f20 = group_trd(eqs, (0,), ref)
    f21 = group_trd(eqs, (1,), ref)

    set_force(P1, b1, f20)
    set_force(P2, b2, f21)


def solve_graph1(eqs, joints, ref):
    (P0, b0), (P1, b1), (G2, b2) = joints

    m10 = group_tmd(eqs, (0,), ref, get_point(P1, not b1))

    ux = get_point(P0, not b0) - get_point(P1, not b1)

    uy = unit(get_angle(G2, 0))

    t10_ = dot(uy, f := group_trd(eqs, (0, 2), ref))

    f10 = t10_ * uy + z_cross(uy) * (m10 - t10_ * det(ux, uy)) / dot(uy, ux)
    set_force(P0, b0, f10)

    m21 = group_tmd(eqs, (1,), ref, get_zero(G2, 0, uy))
    f21 = f10 - f

    f20 = group_trd(eqs, (0,), ref)

    set_force(P1, b1, f20)
    set_normal(G2, b2, f21, m21, uy)


def solve_graph2(eqs, joints, ref):
    (G0, b0), (G1, b1), (P2, b2) = joints

    f = group_trd(eqs, (0,), ref)

    ux, uy = unit(get_angle(G0, 0)), unit(get_angle(G1, 0))
    d = det(ux, uy)
    n10, n20 = z_cross(ux) * dot(f, uy) / d, z_cross(uy) * -dot(f, ux) / d

    f21 = group_trd(eqs, (1,), ref)
    set_force(P2, b2, f21)

    m10 = group_tmd(eqs, (0, 2), ref, get_zero(G0, 0, ux))
    m20 = group_tmd(eqs, (0, 1), ref, get_zero(G1, 0, uy))

    set_normal(G0, b0, n10, m10, ux)
    set_normal(G1, b1, n20, m20, uy)


def solve_graph3(eqs, joints, ref):
    pass


def solve_graph4(eqs, joints, ref):
    (P0, b0), (P1, b1), (G2, b2), (P3, b3), (P4, b4), (G5, b5) = joints

    f = group_trd(eqs, (0, 1, 2, 4), ref)
    ux, uy = unit(get_angle(G2, 0)), unit(get_angle(G5, 0))
    n30, n34 = dot(f, uy) / det(ux, uy) * ux, dot(f, ux) / det(uy, ux) * uy

    m10 = group_tmd(eqs, (0, 2, 3, 4), ref, get_point(P3, 0))
    v1 = get_point(P0, 0) - get_point(P3, 0)
    inv1 = sq_mag(v1) ** -.5
    v1 *= inv1
    t10 = z_cross(v1) * m10 * inv1

    m20 = group_tmd(eqs, (0, 1, 3, 4), ref, get_point(P4, 0))
    v2 = get_point(P1, 0) - get_point(P4, 0)
    inv2 = sq_mag(v2) ** -.5
    v2 *= inv2
    t20 = z_cross(v2) * m20 * inv2

    f = group_trd(eqs, (0,), ref) - n30 - t10 - t20
    n10, n20 = det(f, v2) / det(v1, v2) * v1, det(f, v1) / det(v2, v1) * v2

    set_force(P0, b0, n10 + t10)
    set_force(P1, b1, n20 + t20)
    set_force(P3, b3, group_trd(eqs, (1,), ref))
    set_force(P4, b4, group_trd(eqs, (2,), ref))

    m30 = group_tmd(eqs, (3, 4), ref, get_zero(G2, 0, ux))
    set_normal(G2, b2, n30, m30, ux)

    m43 = group_tmd(eqs, (3,), ref, get_zero(G5, 0, uy))
    set_normal(G5, b5, -n34, m43, uy)


def solve_graph5(eqs, joints, ref):
    pass


def solve_graph6(eqs, joints, ref):
    pass


def solve_graph7(eqs, joints, ref):
    pass


def solve_graph8(eqs, joints, ref):
    pass


def solve_graph9(eqs, joints, ref):
    pass


def solve_graph10(eqs, joints, ref):
    pass


def solve_graph11(eqs, joints, ref):
    pass


def solve_graph12(eqs, joints, ref):
    pass


#  ---------------------------------------------------------------------------------------------------------------------


def solve_block(joint, eq1, eq2, ref):
    joint.block(eq1, eq2, ref)


def solve_graph(index, eqs, joints, ref):
    SOLVE_GRAPH[index](eqs, joints, ref)


def solve_relation(rel, b, eq1, eq2, ref):
    rel.rel_block(b, eq1, eq2, ref)


def compute_inertia(system):
    dt = system.dt
    for s in system.dyn_sols:
        s.trd = -s.m * derivative2_vec(s.og, dt)
        s.g_tmd = -s.j * derivative2(s.angle, dt)


def compute_ma(system):
    for inter in system.interactions:
        inter.set_ma(system)


def rec_ghosts(system):
    for j in system.dyn_ghosted:
        j.dyn_recover_ghosts()


SOLVE_GRAPH = (
    solve_graph0,
    solve_graph1,
    solve_graph2,
    solve_graph3,
    solve_graph4,
    solve_graph5,
    solve_graph6,
    solve_graph7,
    solve_graph8,
    solve_graph9,
    solve_graph10,
    solve_graph11,
    solve_graph12
)

DYN = (
    solve_block,
    solve_graph,
    solve_relation,
    compute_ma,
    compute_inertia,
    rec_ghosts
)
