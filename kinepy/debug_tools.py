from system import *
from kinematic import *
import matplotlib.pyplot as plt


def debug_ppp(n):
    # 4-segments
    sy = System((Solid(), Solid(), Solid(), Solid()))
    p0 = sy.add_revolute(1, 2, (1.5, 0))
    p1 = sy.add_revolute(2, 3, (11., 0.))
    p2 = sy.add_revolute(0, 3, (3., 0.), (10.5, 0))
    s0, s1, s2, s3 = sy.sols

    print(s0.points)
    print(s1.points)
    print(s2.points)
    print(s3.points)

    sy.signs[(0, 2, 1)] = 1

    sy.reset(n)
    s1.angle = np.linspace(0., 4 * np.pi, n)  # i.e pilotage
    sy.eqs = [(0, 1), (0, 1), (2,), (3,)]
    p_p_p(sy, (0, 2, 1), p0.identifier[::-1], p2.identifier, p1.identifier[::-1])

    # raccords de points
    x, y = get_point(sy, 1, 0)
    plt.plot(x, y, 'x')
    x, y = get_point(sy, 2, 0)
    plt.plot(x, y, '.')

    x, y = get_point(sy, 2, 1)
    plt.plot(x, y, 'x')
    x, y = get_point(sy, 3, 0)
    plt.plot(x, y, '.')

    x, y = get_point(sy, 3, 1)
    plt.plot(x, y, 'x')
    x, y = get_point(sy, 0, 0)
    plt.plot(x, y, '.')

    plt.axis('equal')
    plt.show()

    # ----------------

    plt.clf()
    # angles
    plt.plot(p0.angle, label='p0')
    plt.plot(p1.angle, label='p1')
    plt.plot(p2.angle, label='p2')
    plt.legend()

    plt.show()


def debug_gpp(n):
    # piston
    sy = System((Solid(), Solid(), Solid(), Solid()))
    p0 = sy.add_revolute(1, 2, (2, 0))
    p1 = sy.add_revolute(2, 3, (6, 0))
    g2 = sy.add_prismatic(0, 3, d1=2, d2=0)

    s0, s1, s2, s3 = sy.sols

    print(s0.points)
    print(s1.points)
    print(s2.points)
    print(s3.points)

    sy.signs[(2, 0, 1)] = 1

    sy.reset(n)
    s1.angle = np.linspace(0., 4 * np.pi, n)  # i.e pilotage
    sy.eqs = [(0, 1), (0, 1), (2,), (3,)]
    g_p_p(sy, (2, 0, 1), g2.identifier[::-1], p0.identifier, p1.identifier)

    # raccords de points
    x, y = get_point(sy, 1, 0)
    plt.plot(x, y, 'x')
    x, y = get_point(sy, 2, 0)
    plt.plot(x, y, '.')

    x, y = get_point(sy, 2, 1)
    plt.plot(x, y, 'x')
    x, y = get_point(sy, 3, 0)
    plt.plot(x, y, '.')

    plt.axis('equal')
    plt.show()
    # ----------------

    plt.clf()
    # angles, delta
    plt.plot(g2.delta, label='g2')
    plt.plot(p0.angle, label='p0')
    plt.plot(p1.angle, label='p1')
    plt.legend()
    plt.show()


debug_gpp(1001)
