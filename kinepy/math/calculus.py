import numpy as np


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


def derivative(table, dt):  # f', (n,) -> (n,)
    return np.concatenate((nan, .5 * (table[2:] - table[:-2]) / dt, nan))


def derivative2(table, dt):  # f'', (n,) -> (n,)
    return np.concatenate((nan, (table[2:] + table[:-2] - 2 * table[1:-1]) / (dt * dt), nan))


nan_vec = np.array(([np.nan], [np.nan]))


def derivative_vec(table, dt):  # f', (2, n) -> (2, n)
    return np.concatenate((nan_vec, .5 * (table[:, 2:] - table[:, :-2]) / dt, nan_vec), axis=1)


def derivative2_vec(table, dt):  # f'', (2, n) -> (2, n)
    return np.concatenate((nan_vec, (table[:, 2:] + table[:, :-2] - 2 * table[:, 1:-1]) / (dt * dt), nan_vec), axis=1)
