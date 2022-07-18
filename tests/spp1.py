from system import *
from kinematic import *
import matplotlib.pyplot as plt
import tools as to

#%%

n = 1000

sy = System((Solid(), Solid(), Solid()))
p0 = sy.add_revolute(1, 2, p1=(2, 0), p2=(0, 0))
sp1 = sy.add_slide_curve(0, 2, (1, 1))

s0, s1, s2 = sy.sols

print(s0.points)
print(s1.points)
print(s2.points)

sy.signs[(1, 0)] = 1

sy.reset(n)
s1.angle = np.linspace(0., 2 * np.pi, n)
sy.eqs = [(0, 1), (0, 1), (2,)]

sp_p_1(sy, (1, 0), sp1.identifier[::-1], p0.identifier)
"""
C = get_point(sy, 1, 0)
plt.plot(C[0], C[1], 'x')
x, y = get_point(sy, 2, 0)
plt.plot(x, y, '.')

plt.axis('equal')
plt.show()
"""

# angles, delta
plt.plot(s1.angle, to.make_continuous(p0.angle), label='p0') #
plt.plot(s1.angle, to.make_continuous(sp1.angle), label='sp1a')
plt.plot(s1.angle, sp1.delta, label='sp1d')
plt.axis('equal')
plt.legend()

"""
# ---------------
# angles
plt.plot(s0.angle, label='s0')
plt.plot(s1.angle, label='s1')
plt.plot(to.make_continuous(s2.angle), label='s2')

plt.legend()

plt.show()"""