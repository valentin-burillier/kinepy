import numpy as np

ZERO_POINT = np.zeros((2,))
ZERO_21 = np.zeros((2, 1))
ZERO_F = (lambda: 0)
ZERO_ARRAY_F = (lambda: ZERO_21)


def rot(theta):  # rotation matrix
    return np.array([[(c := np.cos(theta)), (s := -np.sin(theta))], [-s, c]])


def unit(theta):  # (n,) -> (2, n)
    return np.array([np.cos(theta), np.sin(theta)])


def z_cross(vec):  # (2, ...) -> (2, ...), z ^ v
    return np.array((-vec[1], vec[0]))


def det(v1, v2):  # (2, n) x (2, n) -> (n,)
    return v1[0] * v2[1] - v2[0] * v1[1]


def dot(v1, v2):
    return np.sum(v1 * v2, axis=0)


def sq_mag(vec):  # (2, n) -> (n,)
    return np.sum(vec * vec, axis=0)


def angle2(vec, im):
    return np.arccos(np.maximum(np.minimum(vec[0] * im, 1.), -1)) * (2 * (vec[1] > 0) - 1)


def rvec(angle_, vec):
    return np.einsum('ikl,k->il', rot(angle_), vec)


def mat_mul_r(mat, vec):  # (2, 2, n) x (2, n) -> (2, n), Replace array
    np.einsum('ikl,kl->il', mat, vec, out=vec)


def get_angle(pri, index):
    return (pri.s1, pri.s2)[index].angle + (pri.a1, pri.a2)[index]


def get_zero(pri, index, u):
    s = (pri.s1, pri.s2)[index]
    return s.origin + (pri.d1, pri.d2)[index] * z_cross(u)


def get_point(rev, index):
    s = (rev.s1, rev.s2)[index]
    return s.origin + rvec(s.angle, (rev.p1, rev.p2)[index])


def rotate_eq(eq, theta):
    r = rot(theta)
    for s in eq:
        s.angle += theta
        mat_mul_r(r, s.origin)


def move_eq(eq, vec):
    for s in eq:
        s.origin += vec
