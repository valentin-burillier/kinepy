from system import *
from kinematic import *
import matplotlib.pyplot as plt
import tools as to

#%%

n = 1000

sy = System((Solid(), Solid(), Solid(), Solid()))
p0 = sy.add_revolute(1, 2, p1=(1, 0))
g1 = sy.add_prismatic(2, 3, d2=1, alpha2=-np.pi/2)
p2 = sy.add_revolute(0, 3, p1=(2, 1))

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

"""
# raccords de points
B = get_point(sy, 1, 0)
plt.plot(B[0], B[1], 'x')
x, y = get_point(sy, 2, 0)
plt.plot(x, y, '.')

C = get_point(sy, 0, 0)
plt.plot(C[0], C[1], 'x')
x, y = get_point(sy, 3, 0)
plt.plot(x, y, '.')

plt.axis('equal')
plt.show()
# ----------------
"""
# angles, delta
plt.plot(s1.angle, p0.angle, label='p0')
plt.plot(s1.angle, g1.delta, label='g1')
plt.plot(s1.angle, p2.angle, label='p2')
plt.axis('equal')
plt.legend()
"""
# ---------------
# angles
plt.plot(s0.angle, label='s0')
plt.plot(s1.angle, label='s1')
plt.plot(s2.angle, label='s2')
plt.plot(s3.angle, label='s3')

plt.legend()

plt.show()"""