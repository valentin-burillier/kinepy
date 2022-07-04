import numpy as np


def rot(theta):  # rotation matrix
    return np.array([[(c := np.cos(theta)), (s := -np.sin(theta))], [-s, c]])


def unit(theta):  # (n,) -> (2, n)
    return np.array([np.cos(theta), np.sin(theta)])


def cross_z(vec):  # (2, ...) -> (2, ...), z ^ v
    return np.array((-vec[1], vec[0]))


def mag(vec):  # (2, n) -> (n,)
    return np.sum(vec * vec, axis=0) ** .5


def sq_mag(vec):  # (2, n) -> (n,)
    return np.sum(vec * vec, axis=0)


def inv_mag(vec):  # (2, n) -> (n,)
    return np.sum(vec * vec, axis=0) ** -.5


def inv_mat(u1, u2):  # (2, n) x (2, n) -> (2, 2, n)
    return np.array([[u2[1], -u2[0]], [-u1[1], u1[0]]]) / (u1[0] * u2[1] - u1[1] * u2[0])


def get_angle(vec):  # (2, n) -> (n,)
    return np.arccos(vec[0] * inv_mag(vec)) * (2 * (vec[1] > 0) - 1)


def get_angle1(vec, mg):
    return np.arccos(vec[0] / mg) * (2 * (vec[1] > 0) - 1)


def get_angle2(vec, im):
    return np.arccos(vec[0] * im) * (2 * (vec[1] > 0) - 1)


def mat_mul_n(mat, vec):  # (2, 2, n) x (2, n) -> (2, n), New array
    return np.einsum('ikl,kl->il', mat, vec)


def mat_mul_r(mat, vec):  # (2, 2, n) x (2, n) -> (2, n), Replace array
    np.einsum('ikl,kl->il', mat, vec, out=vec)


def get_point(system, sol, p):
    return system.get_origin(sol) + np.einsum('ikl,k->il', rot(system.get_ref(sol)), system.sols[sol].points[p])


def change_ref(system, sol, angle, mat, center, vec):
    for i in system.eqs[sol]:
        s = system.sols[i]
        s.origin -= center
        mat_mul_r(mat, s.origin)
        s.origin += vec
        s.angle += angle


def change_ref2(system, sol, angle, mat, center):
    for i in system.eqs[sol]:
        s = system.sols[i]
        s.origin -= center
        mat_mul_r(mat, s.origin)
        s.angle += angle


def trans(system, sol, vec):
    for i in system.eqs[sol]:
        system.sols[i].origin += vec


def make_continuous(angles):
    l0 = 0
    for i, l in enumerate(angles):
        if not np.isnan(angles[i]):
            k = (l - l0 + np.pi)//(2*np.pi)
            angles[i] -= k*2*np.pi
            l0 = angles[i]
        else:
            l0 = 0