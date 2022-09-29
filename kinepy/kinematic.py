import numpy as np

from kinepy.geometry import get_angle, rotate_eq, get_point, move_eq, make_continuous, unit, get_zero, det, dot, sq_mag


def pp_grouping(eq1, p0, p1, sq_z):
    (P0, b0), (P1, b1) = p0, p1
    v1 = get_point(P1, b1) - get_point(P0, b0)
    v2 = get_point(P1, not b1) - get_point(P0, not b0)
    alpha = np.arccos(dot(v1, v2) / sq_z) * (2 * (det(v1, v2) > 0) - 1)
    rotate_eq(eq1, alpha)
    move_eq(eq1, get_point(P0, not b0) - get_point(P0, b0))
    P1.angle_ = P1.s2.angle_ - P1.s1.angle_
    make_continuous(P0.angle_)
    P1.angle_ = P1.s2.angle_ - P1.s1.angle_
    make_continuous(P1.angle_)


def gg_grouping(eq1, g0, g1):
    (G0, b0), (G1, b1) = g0, g1
    u1, u2 = unit(get_angle(G0, 0)), unit(get_angle(G1, 0))
    coeff = det(u1, u2)
    v1 = get_zero(G0, b0, u1) - get_zero(G0, not b0, u1)
    v2 = get_zero(G1, b1, u2) - get_zero(G1, not b1, u2)
    vec = (det(v1, u1) * u2 + det(u2, v2) * u1) / coeff
    move_eq(eq1, vec)
    G0.sliding_, G1.sliding_ = dot(u1, v1 + vec) * (-1, 1)[b0], dot(u2, v2 + vec) * (-1, 1)[b1]


def g_chain(gs, eqs):
    for i, (pri, b) in enumerate(gs):
        rotate_eq(eqs[i+1], get_angle(pri, not b) - get_angle(pri, b))


def solve_graph0(eqs, js, sgn):
    eq0, eq1, eq2 = eqs
    (P0, b0), (P1, b1), (P2, b2) = js

    v01, v02 = get_point(P1, not b1) - get_point(P0, not b0), get_point(P2, not b2) - get_point(P0, b0)
    sq_z = sq_mag(get_point(P2, b2) - get_point(P1, b1))
    sq_a, sq_b = sq_mag(v01), sq_mag(v02)
    m = (sq_a * sq_b) ** -.5
    alpha = np.arccos(0.5 * (sq_b + sq_a - sq_z)) * m * sgn
    rotate_eq(eq1, np.arccos(dot(v01, v02) * m) * (2 * (det(v02, v01) > 0) - 1) + alpha)
    move_eq(eq1, get_point(P0, not b0)) - get_point(P0, b0)
    P0.angle_ = P0.s2.angle_ - P0.s1.angle_
    make_continuous(P0.angle_)

    pp_grouping(eq2, (P1, b0), (P2, b0), sq_z)


def solve_graph1(eqs, js, sgn):
    eq0, eq1, eq2 = eqs
    (P0, b0), (P1, b1), (G2, b2) = js

    g_chain(((G2, b2),), (eq1, eq2))
    u = unit(get_angle(G2, 0))
    x0 = dot(u, v := get_point(P0, b0) - (v1 := get_zero(G2, not b2, u) + get_zero(G2, b2, u)) - get_point(P1, b1))

    sq_z = sq_mag(get_point(P0, not b0) - get_point(P1, not b1))
    dx = (sq_z - det(v, u) ** 2) ** -0.5 * sgn * (1, -1)[b2]
    move_eq(eq2, v1 + u * (x0 + dx))
    G2.sliding_ = (x0 + dx) * (1, -1)[b2]

    pp_grouping(eq0, (P0, not b0), (P1, not b1), sq_z)


def solve_graph2(eqs, js):
    eq0, eq1, eq2 = eqs
    (G0, b0), (G1, b1), (P2, b2) = js

    g_chain(((G0, not b0), (G1, b1)), (eq1, eq0, eq2))
    move_eq(eq2, get_point(P2, not b2) - get_point(P2, b2))
    P2.angle_ = P2.s2.angle_ - P2.s1.angle_
    make_continuous(P2.angle_)

    gg_grouping(eq1 + eq2, (G0, b0), (G1, b1))
