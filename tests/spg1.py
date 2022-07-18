from system import *
from kinematic import *
import matplotlib.pyplot as plt
import tools as to

#%%

n = 1000


sy = System((Solid(), Solid(), Solid()))
sp0 = sy.add_slide_curve(2, 1, alpha=-np.pi/2, d=1)
g1 = sy.add_prismatic(0, 2, d1=-3/np.sqrt(2), alpha1=np.pi/2)

s0, s1, s2 = sy.sols

print(s0.points)
print(s1.points)
print(s2.points)

sy.reset(n)
s1.angle = np.linspace(0., 2 * np.pi, n)
sy.eqs = [(0, 1), (0, 1), (2,), (0, 1)]

sp_g_1(sy, (0, 1), sp0.identifier, g1.identifier[::-1])

seuil = 10
sp0.delta[sp0.delta>seuil] = seuil
sp0.delta[sp0.delta<-seuil] = -seuil
g1.delta[g1.delta>seuil] = seuil
g1.delta[g1.delta<-seuil] = -seuil

"""
# angles, delta
plt.plot(s1.angle, sp0.angle, label='sp0a')
plt.plot(s1.angle, sp0.delta, label='sp0d')
plt.plot(s1.angle, g1.delta, label='g1')
#plt.axis('equal')
plt.legend()

"""
# ---------------
# angles
plt.plot(s0.angle, label='s0')
plt.plot(s1.angle, label='s1')
plt.plot(s2.angle, label='s2')

plt.legend()

plt.show()