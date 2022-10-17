from kinepy.geometry import np, det, dot, sq_mag, z_cross, unit, get_angle, get_zero, get_point, derivative2_vec, \
    derivative2, get_g, rot, mat_mul_n2
from kinepy.interactions import MechanicalAction, ZERO


def tmd(point, eq) -> np.ndarray:  # Intertie - somme(Moments connus) = Moments inconnus
    return np.array(-sum(sum(am.babar(point) for am in s.mech_actions) for s in eq))


def trd(eq) -> np.ndarray:  # Intertie - somme(Forces connues) = Forces inconnues
    return np.array(-sum((sum(am.f for am in s.mech_actions) for s in eq)))


def set_force(rev, b, force):  # Force en rev.point(), par (s1, s2)[b] sur l'autre
    p = get_point(rev, 0)
    rev.s1.mech_actions.append(MechanicalAction(force * (-1, 1)[b], p, 0.))
    rev.s2.mech_actions.append(MechanicalAction(force * (1, -1)[b], p, 0.))
    rev.force_ = force * (-1, 1)[b]


def set_normal(pri, b, normal, torque, u):  # Force normale en get_zero(pri, 0)
    p = get_zero(pri, 0, u)
    n_ = normal * (-1, 1)[b]
    pri.s1.mech_actions.append(MechanicalAction(n_, p, torque * (-1, 1)[b]))
    pri.s2.mech_actions.append(MechanicalAction(-n_, p, torque * (1, -1)[b]))
    pri.normal_ = det(u, n_)
    pri.torque_ = torque * (-1, 1)[b]


def solve_graph0(eqs, js, ref):
    eq0, eq1, eq2 = eqs
    (P0, b0), (P1, b1), (P2, b2) = js

    k = (0b001 >> ref) & 1
    m10 = tmd(get_point(P1, 0), (eq0, eq1 + eq2)[k]) * (1, -1)[k]

    k = (0b101 >> ref) & 1
    m10_ = tmd(get_point(P2, 0), (eq0 + eq2, eq1)[k]) * (1, -1)[k]

    vx, vy = get_point(P0, 0) - get_point(P1, 0), get_point(P0, 0) - get_point(P2, 0)
    inv_x, inv_y = sq_mag(vx) ** -.5, sq_mag(vy) ** -.5
    vx *= inv_x
    vy *= inv_y

    t10, t10_ = m10 * inv_x, m10_ * inv_y
    f10 = t10 * z_cross(vx) + vx * (t10_ - dot(vx, vy) * t10) / det(vy, vx)
    set_force(P0, b0, f10)

    k = (0b001 >> ref) & 1
    f20 = trd((eq0, eq1 + eq2)[k]) * (1, -1)[k]
    k = (0b010 >> ref) & 1
    f21 = trd((eq1, eq0 + eq2)[k]) * (1, -1)[k]

    set_force(P1, b1, f20)
    set_force(P2, b2, f21)


def solve_graph1(eqs, js, ref):
    eq0, eq1, eq2 = eqs
    (P0, b0), (P1, b1), (G2, b2) = js

    k = (0b001 >> ref) & 1
    m10 = tmd(get_point(P1, b1), (eq0, eq1 + eq2)[k]) * (1, -1)[k]

    ux = get_point(P0, b0) - get_point(P1, b1)
    inv_x = sq_mag(ux) ** -.5
    t10 = m10 * inv_x

    ux *= inv_x
    uy = unit(get_angle(G2, 0))

    k = (0b101 >> ref) & 1
    t10_ = det(uy, f := trd((eq0 + eq2, eq1)[k]) * (1, -1)[k])
    f10 = t10 * z_cross(ux) - ux * (t10_ - det(uy, ux) * t10) / dot(uy, ux)
    set_force(P0, b0, f10)

    m21 = tmd(get_zero(G2, 0, uy), (eq1, eq0 + eq2)[not k]) * (-1, 1)[k]
    f21 = f10 - f

    k = (0b001 >> ref) & 1
    f20 = trd((eq0, eq1 + eq2)[k]) * (1, -1)[k]

    set_force(P1, b1, f20)
    set_normal(G2, b2, f21, m21, uy)


def solve_graph2(eqs, js, ref):
    eq0, eq1, eq2 = eqs
    (G0, b0), (G1, b1), (P2, b2) = js

    k = (0b001 >> ref) & 1
    f = trd((eq0, eq1 + eq2)[k]) * (1, -1)[k]
    ux, uy = unit(get_angle(G0, 0)), unit(get_angle(G1, 0))
    d = det(ux, uy)
    n10, n20 = z_cross(ux) * dot(f, uy) / d, z_cross(uy) * -dot(f, ux) / d

    k = (0b010 >> ref) & 1
    f21 = trd((eq1, eq2 + eq0)[k]) * (1, -1)[k] - n10
    set_force(P2, b2, f21)

    m10 = tmd(get_zero(G0, 0, ux), (eq1, eq2 + eq0)[k]) * (-1, 1)[k]

    k = (0b011 >> ref) & 1
    m20 = tmd(get_zero(G1, 0, uy), (eq0 + eq1, eq2)[k]) * (1, -1)[k]

    set_normal(G0, b0, n10, m10, ux)
    set_normal(G1, b1, n20, m20, uy)


def solve_graph3(eqs, js, ref):
    pass


def solve_graph4(eqs, js, ref):
    eq0, eq1, eq2, eq3, eq4 = eqs
    (P0, b0), (P1, b1), (G2, b2), (P3, b3), (P4, b4), (G5, b5) = js

    k = (0b10111 >> ref) & 1
    f = trd((eq0 + eq1 + eq2 + eq4, eq3)[k]) * (1, -1)[k]
    ux, uy = unit(get_angle(G2, 0)), unit(get_angle(G5, 0))
    n30, n34 = dot(f, uy) / det(ux, uy) * ux, dot(f, ux) / det(uy, ux) * uy

    k = (0b00010 >> ref) & 1
    m10 = tmd(get_point(P3, 0), (eq1, eq0 + eq2 + eq3 + eq4)[k]) * (-1, 1)[k]
    v1 = get_point(P0, 0) - get_point(P3, 0)
    inv1 = sq_mag(v1) ** -.5
    v1 *= inv1
    t10 = z_cross(v1) * m10 * inv1

    k = (0b00100 >> ref) & 1
    m20 = tmd(get_point(P4, 0), (eq2, eq0 + eq1 + eq3 + eq4)[k]) * (-1, 1)[k]
    v2 = get_point(P1, 0) - get_point(P4, 0)
    inv2 = sq_mag(v2) ** -.5
    v2 *= inv2
    t20 = z_cross(v2) * m20 * inv2

    k = (0b00001 >> ref) & 1
    f = trd((eq0, eq1 + eq2 + eq3 + eq4)[k]) * (1, -1)[k] - n30 - t10 - t20
    n10, n20 = det(f, v2) / det(v1, v2) * v1, det(f, v1) / det(v2, v1) * v2

    set_force(P0, b0, n10 + t10)
    set_force(P1, b1, n20 + t20)

    k = (0b00010 >> ref) & 1
    set_force(P3, b3, trd((eq1, eq0 + eq2 + eq3 + eq4)[k]) * (1, -1)[k])

    k = (0b00100 >> ref) & 1
    set_force(P4, b4, trd((eq2, eq0 + eq1 + eq3 + eq4)[k]) * (1, -1)[k])

    k = (0b11000 >> ref) & 1
    m30 = tmd(get_zero(G2, 0, ux), (eq3 + eq4, eq0 + eq1 + eq2)[k]) * (1, -1)[k]
    set_normal(G2, b2, n30, m30, ux)

    k = (0b01000 >> ref) & 1
    m43 = tmd(get_zero(G5, 0, uy), (eq3, eq0 + eq1 + eq2 + eq4)[k]) * (-1, 1)[k]
    set_normal(G5, b5, -n34, m43, uy)


def solve_graph5(eqs, js, ref):
    pass


def solve_graph6(eqs, js, ref):
    pass


def solve_graph7(eqs, js, ref):
    pass


def solve_graph8(eqs, js, ref):
    pass


def solve_graph9(eqs, js, ref):
    pass


def solve_graph10(eqs, js, ref):
    pass


def solve_graph11(eqs, js, ref):
    pass


def solve_graph12(eqs, js, ref):
    pass


#  ---------------------------------------------------------------------------------------------------------------------


def solve_block(j, eq1, eq2, ref):
    j.block(eq1, eq2, ref)


def solve_graph(index, eqs, js, ref):
    SOLVE_GRAPH[index](eqs, js, ref)


def solve_relation(rel, b, eq1, eq2, ref):
    rel.rel_block(b, eq1, eq2, ref)


def compute_inertia(system):
    dt = system.dt
    for s in system.sols:
        og = get_g(s)
        s.mech_actions.append(MechanicalAction(-s.m_ * derivative2_vec(og, dt), og, -s.j_ * derivative2(s.angle_, dt)))


def compute_ma(system):
    for s in system.sols:
        r = rot(s.angle_)
        og = mat_mul_n2(r, s.og_)
        f_tot, t_tot = ZERO, 0.
        for f, t, p in s.external_actions:
            f_tot += (f := f())
            t_tot += t() + det(mat_mul_n2(r, p) - og, f)
        s.mech_actions.append(MechanicalAction(f_tot, s.origin_ + og, t_tot))
    for inter in system.interations:
        inter.set_ma()


def rec_ghosts(system):
    pass


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
