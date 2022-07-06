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
    # piston 1 (stationary)
    sy = System((Solid(), Solid(), Solid(), Solid()))
    p0 = sy.add_revolute(1, 2, (2, 0))
    p1 = sy.add_revolute(2, 3, (6, 0))
    g2 = sy.add_prismatic(0, 3, d1=0, d2=0)

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


def debug_ppg(n):
    # piston 2 (mobil)
    sy = System((Solid(), Solid(), Solid(), Solid()))
    p0 = sy.add_revolute(1, 2, p1=(1, 0))
    g1 = sy.add_prismatic(2, 3)
    p2 = sy.add_revolute(0, 3, p1=(4, 0))

    s0, s1, s2, s3 = sy.sols

    print(s0.points)
    print(s1.points)
    print(s2.points)
    print(s3.points)

    sy.signs[0, 2, 1] = 1

    sy.reset(n)
    s1.angle = np.linspace(0, 2 * np.pi, n)
    sy.eqs = [(0, 1), (0, 1), (2,), (3,)]

    p_p_g(sy, (0, 2, 1), p0.identifier[::-1], p2.identifier, g1.identifier[::-1])

    # a compléter


def debug_pgg(n):
    # "Polar to cartesian" system
    sy = System((Solid(), Solid(), Solid(), Solid()))
    p0 = sy.add_revolute(1, 2, p1=(5, 0))
    g1 = sy.add_prismatic(2, 3)
    g2 = sy.add_prismatic(0, 3, alpha1=np.pi/2, alpha2=np.pi/2)

    s0, s1, s2, s3 = sy.sols

    print(s0.points)
    print(s1.points)
    print(s2.points)
    print(s3.points)

    sy.reset(n)
    s1.angle = np.linspace(0., 4 * np.pi, n)  # i.e pilotage
    sy.eqs = [(0, 1), (0, 1), (2,), (3,)]

    p_g_g(sy, (0, 2, 1), p0.identifier[::-1], g2.identifier, g1.identifier[::-1])

    # a completer


def debug_ggp(n):
    # Oldham joint
    sy = System((Solid(), Solid(), Solid(), Solid(), Solid()))
    g0 = sy.add_prismatic(1, 2)
    p1 = sy.add_revolute(2, 3)
    g2 = sy.add_prismatic(3, 4, alpha2=np.pi/2)

    s0, s1, s2, s3, s4 = sy.sols

    print(s0.points)
    print(s1.points)
    print(s2.points)
    print(s3.points)
    print(s4.points)

    sy.reset(n)
    s1.angle = s4.angle = np.linspace(0., 4 * np.pi, n)
    sy.eqs = [(0, 1, 4), (0, 1, 4), (2,), (3,), (0, 1, 4)]

    g_g_p(sy, (0, 2, 1), g0.identifier[::-1], g2.identifier[::-1], p1.identifier[::-1])

    # a completer


def debug_spp1(n):
    # piston 2
    sy = System((Solid(), Solid(), Solid()))
    p0 = sy.add_revolute(1, 2, p1=(2, 0))
    sp1 = sy.add_slide_curve(0, 2, (5, 0))

    s0, s1, s2 = sy.sols

    print(s0.points)
    print(s1.points)
    print(s2.points)

    sy.signs[(1, 0)] = 1

    sy.reset(n)
    s1.angle = np.linspace(0., 4 * np.pi, n)
    sy.eqs = [(0, 1), (0, 1), (2,)]

    sp_p_1(sy, (1, 0), sp1.identifier[::-1], p0.identifier)


def debug_spp2(n):
    # piston 1
    sy = System((Solid(), Solid(), Solid()))
    p0 = sy.add_revolute(1, 2, p1=(2, 0))
    sp1 = sy.add_slide_curve(2, 0, (5, 0))

    s0, s1, s2 = sy.sols

    print(s0.points)
    print(s1.points)
    print(s2.points)

    sy.signs[(1, 0)] = 1

    sy.reset(n)
    s1.angle = np.linspace(0., 4 * np.pi, n)
    sy.eqs = [(0, 1), (0, 1), (2,)]

    sp_p_2(sy, (1, 0), sp1.identifier, p0.identifier)


def debug_spg2(n):
    # Polair -> cartesien
    sy = System((Solid(), Solid(), Solid()))
    sp0 = sy.add_slide_curve(1, 2, p=(2, 0))
    g1 = sy.add_prismatic(0, 2, alpha1=np.pi/2, alpha2=np.pi/2)

    s0, s1, s2 = sy.sols

    print(s0.points)
    print(s1.points)
    print(s2.points)

    sy.reset(n)
    s1.angle = np.linspace(0., 4 * np.pi, n)
    sy.eqs = [(0, 1), (0, 1), (2,)]

    sp_g_2(sy, (0, 1), sp0.identifier[::-1], g1.identifier)


def debug_spg1(n):
    # Oldham joint

    sy = System((Solid(), Solid(), Solid(), Solid()))
    sp0 = sy.add_slide_curve(2, 1)
    g1 = sy.add_prismatic(2, 3, alpha1=np.pi/2, alpha2=np.pi/2)

    s0, s1, s2, s3 = sy.sols

    print(s0.points)
    print(s1.points)
    print(s2.points)
    print(s3.points)

    sy.reset(n)
    s1.angle = s3.angle = np.linspace(0., 4 * np.pi, n)
    sy.eqs = [(0, 1, 3), (0, 1, 3), (2,), (0, 1, 3)]

    sp_g_1(sy, (0, 1), sp0.identifier, g1.identifier[::-1])

