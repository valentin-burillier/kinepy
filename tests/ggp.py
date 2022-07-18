from system import *
from kinematic import *
import matplotlib.pyplot as plt
import tools as to

#%%

n = 1000


sy = System((Solid(), Solid(), Solid(), Solid()))
g0 = sy.add_prismatic(1, 2, alpha1=-np.pi/2, d1=1)
p1 = sy.add_revolute(2, 3)
g2 = sy.add_prismatic(0, 3, d1=-3/np.sqrt(2), alpha1=np.pi/4)


s0, s1, s2, s3 = sy.sols

print(s0.points)
print(s1.points)
print(s2.points)
print(s3.points)

sy.reset(n)
s1.angle = np.linspace(0., np.pi*2, n)  # i.e pilotage
sy.eqs = [(0, 1), (0, 1), (2,), (3,)]

g_g_p(sy, (0, 2, 1), g0.identifier[::-1], g2.identifier, p1.identifier[::-1])

seuil = 10
g2.delta[g2.delta>seuil] = seuil
g2.delta[g2.delta<-seuil] = -seuil
g0.delta[g0.delta>seuil] = seuil
g0.delta[g0.delta<-seuil] = -seuil

"""
# angles, delta
plt.plot(s1.angle, p1.angle, label='p1')
plt.plot(s1.angle, g0.delta, label='g0')
plt.plot(s1.angle, g2.delta, label='g2')
#plt.axis('equal')
plt.legend()

"""
# ---------------
# angles
plt.plot(s0.angle, label='s0')
plt.plot(s1.angle, label='s1')
plt.plot(s2.angle, label='s2')
plt.plot(s3.angle, label='s3')

plt.legend()

plt.show()