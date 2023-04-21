import numpy as np
from kinepy.math.calculus import make_continuous
from kinepy.math.geometry import get_angle, rotate_eq, get_point, move_eq, unit, get_zero, det, dot, sq_mag, angle2


def solve_p(p, b, eq):
    move_eq(eq, get_point(p, not b) - get_point(p, b))
    p.angle = p.s2.angle - p.s1.angle
    make_continuous(p.angle)


def pp_grouping(eq1, p0, p1, sq_z):
    (P0, b0), (P1, b1) = p0, p1
    v1 = get_point(P1, b1) - get_point(P0, b0)
    v2 = get_point(P1, not b1) - get_point(P0, not b0)
    alpha = np.arccos(np.maximum(-1, np.minimum(1., dot(v1, v2) / sq_z))) * (2 * (det(v1, v2) > 0) - 1)
    rotate_eq(eq1, alpha)
    solve_p(P0, b0, eq1)
    P1.angle = P1.s2.angle - P1.s1.angle
    make_continuous(P1.angle)


def gg_grouping(eq1, g0, g1):
    (G0, b0), (G1, b1) = g0, g1
    u1, u2 = unit(get_angle(G0, 0)), unit(get_angle(G1, 0))
    coeff = det(u1, u2)
    v1 = get_zero(G0, b0, u1) - get_zero(G0, not b0, u1)
    v2 = get_zero(G1, b1, u2) - get_zero(G1, not b1, u2)
    vec = (det(v1, u1) * u2 + det(u2, v2) * u1) / coeff
    move_eq(eq1, vec)
    G0.sliding, G1.sliding = dot(u1, v1 + vec) * (-1, 1)[b0], dot(u2, v2 + vec) * (-1, 1)[b1]


def g_chain(gs, eqs):
    for i, (pri, b) in enumerate(gs):
        rotate_eq(eqs[i], get_angle(pri, not b) - get_angle(pri, b))


def solve_graph0(eqs, js, sgn):
    eq0, eq1, eq2 = eqs
    (P0, b0), (P1, b1), (P2, b2) = js

    v01, v02 = get_point(P1, not b1) - get_point(P0, not b0), get_point(P2, not b2) - get_point(P0, b0)
    sq_z = sq_mag(get_point(P2, b2) - get_point(P1, b1))
    sq_a, sq_b = sq_mag(v01), sq_mag(v02)
    m = (sq_a * sq_b) ** -.5
    alpha = np.arccos(0.5 * (sq_b + sq_a - sq_z) * m) * sgn
    rotate_eq(eq1, np.arccos(dot(v01, v02) * m) * (2 * (det(v02, v01) > 0) - 1) + alpha)
    solve_p(P0, b0, eq1)

    pp_grouping(eq2, (P1, b1), (P2, b2), sq_z)


def solve_graph1(eqs, js, sgn, chain=True):
    eq0, eq1, eq2 = eqs
    (P0, b0), (P1, b1), (G2, b2) = js

    if chain:
        g_chain(((G2, b2),), (eq2,))

    u = unit(get_angle(G2, 0))
    x0 = dot(u, v := get_point(P0, b0) + (v1 := get_zero(G2, b2, u) - get_zero(G2, not b2, u)) - get_point(P1, b1))

    sq_z = sq_mag(get_point(P0, not b0) - get_point(P1, not b1))
    dx = (sq_z - det(v, u) ** 2) ** 0.5 * (-sgn, sgn)[b2]

    move_eq(eq2, -v1 + u * (x0 + dx))
    G2.sliding = (x0 + dx) * (-1, 1)[b2]

    pp_grouping(eq0, (P0, not b0), (P1, not b1), sq_z)


def solve_graph2(eqs, js, sgn=None):
    eq0, eq1, eq2 = eqs
    (G0, b0), (G1, b1), (P2, b2) = js

    g_chain(((G0, not b0), (G1, b1)), (eq0, eq2))
    move_eq(eq2, get_point(P2, not b2) - get_point(P2, b2))
    P2.angle = P2.s2.angle - P2.s1.angle
    make_continuous(P2.angle)

    gg_grouping(eq1 + eq2, (G0, b0), (G1, b1))


def solve_graph3(eqs, js, sgn=None):
    pass


def solve_graph4(eqs, js, sgn):
    eq0, eq1, eq2, eq3, eq4 = eqs
    (P0, b0), (P1, b1), (G2, b2), (P3, b3), (P4, b4), (G5, b5) = js

    g_chain(((G2, b2), (G5, b5)), (eq3, eq4))

    v = get_point(P1, not b1) - get_point(P0, not b0) + get_point(P3, b3) - get_point(P4, b4)
    sq_z = sq_mag(v)
    sq_a, sq_b = sq_mag(get_point(P3, not b3) - get_point(P0, b0)), sq_mag(get_point(P4, not b4) - get_point(P1, b1))
    alpha = np.arccos(0.5 * (sq_a + sq_z - sq_b) * (sq_z * sq_a) ** -.5) * sgn
    move_eq(eq4, unit(angle2(v, sq_z ** -.5) + alpha) * sq_a ** .5 + get_point(P0, not b0) - get_point(P3, b3))

    pp_grouping(eq1, (P0, b0), (P3, not b3), sq_a)
    pp_grouping(eq2, (P1, b1), (P4, not b4), sq_b)
    gg_grouping(eq3, (G2, b2), (G5, not b5))


def solve_graph5(eqs, js, sgn):
    eq0, eq1, eq2, eq3, eq4 = eqs
    (P0, b0), (P1, b1), (P2, b2), (G3, b3), (G4, b4), (G5, b5) = js

    g_chain(((G3, b3), (G4, not b4)), (eq4, eq2))
    g_chain(((G5, not b5),), (eq3,))

    u1, u2, u3 = unit(get_angle(G3, 0)), unit(get_angle(G4, 0)), unit(get_angle(G5, 0))
    c0, c1 = det(u1, u2), det(u3, u2)

    p1, p2, p3 = (
        get_point(P0, b0) - get_zero(G3, not b3, u1) + get_zero(G3, b3, u1),
        get_point(P1, b1) - get_zero(G4, not b4, u2) + get_zero(G4, b4, u2),
        get_point(P2, b2) - get_zero(G5, not b5, u3) + get_zero(G5, b5, u3)
    )

    k0 = det(u1, p2 - p1) * c1 + c0 * det(u3, p3 - p2)

    p1, p2, p3 = (
        get_point(P0, not b0),
        get_point(P1, not b1),
        get_point(P2, not b2),
    )

    v0, v1 = (p2 - p1) * c1, (p3 - p2) * c0

    x, y = det(u1, v0) + det(u3, v1), dot(u1, v0) + dot(u3, v1)
    inv_mag = (x * x + y * y) ** -.5

    rotate_eq(eq0, sgn * np.arccos(k0 * inv_mag) + np.arccos(x * inv_mag) * (2 * (0 < y) - 1))
    solve_p(P0, b0, eq1)
    solve_p(P1, b1, eq2)
    solve_p(P2, b2, eq3)
    gg_grouping(eq4, (G3, b3), (G4, b4))
    G5.sliding = dot(u3, get_zero(G5, 1, u3) - get_zero(G5, 0, u3))


def solve_graph6(eqs, js, sgn=None):
    pass


def solve_graph7(eqs, js, sgn):
    eq0, eq1, eq2, eq3, eq4 = eqs
    (G0, b0), (P1, b1), (G2, b2), (G3, b3), (P4, b4), (P5, b5) = js

    g_chain(((G2, not b2), (G0, b0), (G3, b3)), (eq0, eq1, eq0))
    move_eq(eq4, get_point(P5, not b5) - get_point(P5, b5))
    P5.angle = P5.s2.angle - P5.s1.angle
    make_continuous(P5.angle)

    solve_graph1((eq2, eq0, eq4 + eq3), ((P1, not b1), (P4, b4), (G2, b2)), sgn, False)
    gg_grouping(eq1, (G0, b0), (G3, not b3))


def solve_graph8(eqs, js, sgn=None):
    pass


def solve_graph9(eqs, js, sgn=None):
    eq0, eq1, eq2, eq3, eq4 = eqs
    (G0, b0), (G1, b1), (P2, b2), (G3, b3), (P4, b4), (G5, b5) = js

    g_chain(((G1, not b1), (G0, b0), (G3, b3), (G5, not b5)), (eq0, eq1, eq4, eq3))

    solve_p(P2, b2, eq3)
    solve_p(P4, b4, eq4)

    gg_grouping(eq2 + eq4, (G1, b1), (G5, b5))
    gg_grouping(eq1, (G0, b0), (G3, not b3))


def solve_graph10(eqs, js, sgn=None):
    eq0, eq1, eq2, eq3, eq4 = eqs
    (G0, b0), (P1, b1), (P2, b2), (G3, b3), (G4, b4), (G5, b5) = js

    g_chain(((G0, b0), (G3, b3), (G5, not b5)), (eq1, eq4, eq3))
    g_chain(((G4, not b4),), (eq2,))

    solve_p(P1, b1, eq2)
    solve_p(P2, b2, eq3)

    gg_grouping(eq4, (G4, b4), (G5, b5))
    gg_grouping(eq1, (G0, b0), (G3, not b3))


def solve_graph11(eqs, js, sgn=None):
    pass


def solve_graph12(eqs, js, sgn=None):
    pass


#  ---------------------------------------------------------------------------------------------------------------------


def solve_graph(system, index, eqs, js, key):
    SOLVE_GRAPHS[index](eqs, js, system.signs.get(system.tags[key], None))


def continuous_solid_angle(system):
    for s in system.sols:
        make_continuous(s.angle)


def solve_relation(rel, j_index, eq1, eq2):
    rel.rel_pilot(j_index, eq1, eq2)


def solve_pilot(system, eq_order):
    index = 0
    for joint, (eq1, eq2) in zip(system.piloted, eq_order):
        joint.pilot(eq1, eq2, *(system.inputs[index + i] for i in range(joint.dof)))
        index += joint.dof


def set_origin(system):
    move_eq(system.kin_sols, -system.sols[0].origin)
    rotate_eq(system.kin_sols, -system.sols[0].angle)


def recover_ghosts(system):
    for j in system.kin_ghosted:
        j.kin_recover_ghosts()


SOLVE_GRAPHS = (
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

KIN = (
    solve_pilot,
    solve_graph,
    solve_relation,
    continuous_solid_angle,
    set_origin,
    recover_ghosts
)
