from system import *
from kinematic import *
import matplotlib.pyplot as plt
import tools as to

#%%

n = 1000


sy = System((Solid(), Solid(), Solid(), Solid()))
p0 = sy.add_revolute(1, 2, p1=(5, 0))
g1 = sy.add_prismatic(2, 3, alpha2=-np.pi/4)
g2 = sy.add_prismatic(0, 3, d1=-2*np.sqrt(2), alpha1=np.pi/4)


s0, s1, s2, s3 = sy.sols

print(s0.points)
print(s1.points)
print(s2.points)
print(s3.points)

sy.reset(n)
s1.angle = np.linspace(0., np.pi*2, n)  # i.e pilotage
sy.eqs = [(0, 1), (0, 1), (2,), (3,)]

p_g_g(sy, (0, 2, 1), p0.identifier[::-1], g2.identifier, g1.identifier[::-1])


# angles, delta
plt.plot(s1.angle, p0.angle, label='p0')
plt.plot(s1.angle, g1.delta, label='g1')
plt.plot(s1.angle, g2.delta, label='g2')
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