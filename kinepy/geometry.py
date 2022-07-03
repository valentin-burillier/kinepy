import numpy as np


def rot(theta):  # rotation matrix
    return np.array([[(c := np.cos(theta)), (s := -np.sin(theta))], [-s, c]])


def unit(theta):
    return np.array([[np.cos(theta)], [np.sin(theta)]])


def cross_z(vec):  # (2, 1, n) -> (2, 1, n), z ^ v
    return np.array((vec[1], -vec[0]))


def mag(vec):  # (2, 1, n) -> (n,)
    return np.sum(vec * vec, axis=0)[0] ** .5


def sq_mag(vec):  # (2, 1, n) -> (n,)
    return np.sum(vec * vec, axis=0)[0]


def inv_mag(vec):  # (2, 1, n) -> (n,)
    return np.sum(vec * vec, axis=0)[0] ** -.5


def inv_mat(u1, u2):  # (2, 1, n) x (2, 1, n) -> (2, 2, n)
    return np.array([[u2[1][0], -u2[0][0]], [-u1[1][0], u1[0][0]]]) / (u1[0][0] * u2[1][0] - u1[1][0] * u2[0][1])


def get_angle(vec):  # (2, 1, n) -> (n,)
    return np.arccos(vec[0][0] * inv_mag(vec)) * (2 * (vec[1][0] > 0) - 1)


def get_angle1(vec, mg):
    return np.arccos(vec[0][0] / mg) * (2 * (vec[1][0] > 0) - 1)


def get_angle2(vec, im):
    return np.arccos(vec[0][0] * im) * (2 * (vec[1][0] > 0) - 1)


def mat_mul_n(mat, vecs):  # (2, 2, n) x (2, m, n) -> (2, m, n), Nouveau tableau
    return np.einsum('ikl,kjl->ijl', mat, vecs)


def mat_mul_r(mat, vecs):  # (2, 2, n) x (2, m, n) -> (2, m, n), Remplace le tableau
    np.einsum('ikl,kjl->ijl', mat, vecs, out=vecs)


def mat_mul_2n(mat, vecs):  # (2, 2) x (2, m, n) -> (2, m, n)
    return np.einsum('ik,kjl->ijl', mat, vecs)


def mat_mul_2r(mat, vecs):  # (2, 2) x (2, m, n) -> (2, m, n)
    return np.einsum('ik,kjl->ijl', mat, vecs, out=vecs)


def change_ref(system, sol, angle, mat, center, vec):
    for s in system.eqs[sol]:
        s._points -= center
        matMulR(mat, s._points)
        s._points += vec
        s.angle += angle


def change_ref2(system, sol, angle, mat, center):
    for s in system.eqs[sol]:
        s._points -= center
        matMulR(mat, s._points)
        s.angle += angle


def trans(system, sol, vec):
    for s in system.eqs[sol]:
        s._points += vec