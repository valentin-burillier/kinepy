import numpy as np

R = lambda theta: np.array([[(c := np.cos(theta)), (s := -np.sin(theta))], [-s, c]])
U = lambda theta: np.array([[np.cos(theta)], [np.sin(theta)]])


def crossZ(vec):  # (2, 1, n) -> (2, 1, n), z ^ v
    return np.array((vec[1], -vec[0]))


def mag(vec):  # (2, 1, n) -> (n,)
    return np.sum(vec * vec, axis=0)[0] ** .5


def sqMag(vec):  # (2, 1, n) -> (n,)
    return np.sum(vec * vec, axis=0)[0]


def invMag(vec):  # (2, 1, n) -> (n,)
    return np.sum(vec * vec, axis=0)[0] ** -.5


def invMat(u1, u2):  # (2, 1, n) x (2, 1, n) -> (2, 2, n)
    return np.array([[u2[1][0], -u2[0][0]], [-u1[1][0], u1[0][0]]]) / (u1[0][0] * u2[1][0] - u1[1][0] * u2[0][1])


def angle(vec):  # (2, 1, n) -> (n,)
    return np.arccos(vec[0][0] * invMag(vec)) * (2 * (vec[1][0] > 0) - 1)


def angle1(vec, mg):
    return np.arccos(vec[0][0] / mg) * (2 * (vec[1][0] > 0) - 1)


def angle2(vec, im):
    return np.arccos(vec[0][0] * im) * (2 * (vec[1][0] > 0) - 1)


def matMulN(mat, vecs):  # (2, 2, n) x (2, m, n) -> (2, m, n), Nouveau tableau
    return np.einsum('ikl,kjl->ijl', mat, vecs)


def matMulR(mat, vecs):  # (2, 2, n) x (2, m, n) -> (2, m, n), Remplace le tableau
    np.einsum('ikl,kjl->ijl', mat, vecs, out=vecs)


def matMul2N(mat, vecs):  # (2, 2) x (2, m, n) -> (2, m, n)
    return np.einsum('ik,kjl->ijl', mat, vecs)


def matMul2R(mat, vecs):  # (2, 2) x (2, m, n) -> (2, m, n)
    return np.einsum('ik,kjl->ijl', mat, vecs, out=vecs)


def changeRef(system, sol, angle, mat, center, vec):
    for s in system.eqs[sol]:
        s._points -= center
        matMulR(mat, s._points)
        s._points += vec
        s.angle += angle


def changeRef2(system, sol, angle, mat, center):
    for s in system.eqs[sol]:
        s._points -= center
        matMulR(mat, s._points)
        s.angle += angle


def trans(system, sol, vec):
    for s in system.eqs[sol]:
        s._points += vec