import numpy as np


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


def mag(vec):  # (2, n) -> (n,)
    return np.sum(vec * vec, axis=0) ** .5


def sq_mag(vec):  # (2, n) -> (n,)
    return np.sum(vec * vec, axis=0)


def inv_mag(vec):  # (2, n) -> (n,)
    return np.sum(vec * vec, axis=0) ** -.5


def inv_mat(u1, u2):  # (2, n) x (2, n) -> (2, 2, n)
    return np.array([[u2[1], -u2[0]], [-u1[1], u1[0]]]) / (u1[0] * u2[1] - u1[1] * u2[0])


def angle(vec):  # (2, n) -> (n,)
    return np.arccos(np.maximum(np.minimum(vec[0] * inv_mag(vec), 1.), -1)) * (2 * (vec[1] > 0) - 1)


def angle2(vec, im):
    return np.arccos(np.maximum(np.minimum(vec[0] * im, 1.), -1)) * (2 * (vec[1] > 0) - 1)


def mat_mul_n(mat, vec):  # (2, 2, n) x (2, n) -> (2, n), New array
    return np.einsum('ikl,kl->il', mat, vec)


def mat_mul_n2(mat, vec):  # (2, 2, n) x (2,) -> (2, n)
    return np.einsum('ikl,k->il', mat, vec)


def mat_mul_r(mat, vec):  # (2, 2, n) x (2, n) -> (2, n), Replace array
    np.einsum('ikl,kl->il', mat, vec, out=vec)


def get_point(j, index):
    return j.__get_point__(index)


def get_dist(g, index):
    return g.__get_dist__(index)


def get_unit(j, index):
    return j.__get_unit__(index)


def get_angle(j, index):
    return j.__get_angle__(index)


def change_ref(system, sol, angle, mat, center, vec):
    # O' = R(angle).(O - center) + vec
    for i in system.eqs[sol]:
        s = system.sols[i]
        s.origin_ -= center
        mat_mul_r(mat, s.origin_)
        s.origin_ += vec
        s.angle_ += angle


def change_ref2(system, sol, angle, mat, center):
    # O' = R(angle).(O - center)
    for i in system.eqs[sol]:
        s = system.sols[i]
        s.origin_ -= center
        mat_mul_r(mat, s.origin_)
        s.angle_ += angle


def trans(system, sol, vec):
    # O' = O + vec
    for i in system.eqs[sol]:
        system.sols[i].origin_ += vec


# ----------------------------------------------- Calculus -------------------------------------------------------------

def make_continuous(angles):
    l0 = 0
    for i, l in enumerate(angles):
        if not np.isnan(angles[i]):
            k = (l - l0 + np.pi)//(2*np.pi)
            angles[i] -= k*2*np.pi
            l0 = angles[i]
        else:
            l0 = 0


nan = [np.nan]


def derivative1(table, dt):  # f', (n,) -> (n,)
    return np.concatenate((nan, .5 * (table[2:] - table[:-2]) / dt, nan))


def derivative2(table, dt):  # f'', (n,) -> (n,)
    return np.concatenate((nan, (table[2:] + table[:-2] - 2 * table[1:-1]) / (dt * dt), nan))


nan_vec = np.array(([np.nan], [np.nan]))


def derivative2_vec(table, dt):  # f'', (2, n) -> (2, n)
    return np.concatenate((nan_vec, (table[:, 2:] + table[:, :-2] - 2 * table[:, 1:-1]) / (dt * dt), nan_vec), axis=1)
