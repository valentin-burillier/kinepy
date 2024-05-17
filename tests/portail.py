# -*- coding: utf-8 -*-
"""
Created on Mon Feb 27 09:19:36 2023
@author: buril
"""

import numpy as np
import matplotlib.pyplot as plt
import kinepy as k
import kinepy.tools as t
import kinepy.units as units
print(k)

#%%

a = 100
b = 200
c = 30
d = 260
e = 280
f = 800
h = 400

#%%

n, T = 10001, 5
sys = k.System()

s1 = sys.add_solid('Ventail', m=50, g=(f/2, 0))
s2 = sys.add_solid('Bras')
s3 = sys.add_solid('Bielle')

r1 = sys.add_revolute(0, 1, p1=(a, 0))
r2 = sys.add_revolute(1, 2, p1=(d, c), p2=(e, 0))
r3 = sys.add_revolute(0, 3, p1=(0, b))
r4 = sys.add_revolute(3, 2, p1=(e, 0))

sys.pilot(r1)

sys.compile()

#%%

a = t.direct_input(0, np.pi/2, T, n, phy=units.ANGLE)
sys.solve_kinematics(a)

#%%
b_min, b_max = np.min(r3.angle), np.max(r3.angle)

sys.pilot(r3)

sys.compile()
sys.change_signs(-1)

b_ = t.sinusoidal_input(b_min, b_max, T, n, v_max=0.5, phy=units.ANGLE)
sys.solve_kinematics(b_)
#r1.set_torque(10)
sys.solve_dynamics(b_, T)
H = s1.get_point((d, 0))
P = s1.get_point((f, 0))

VP = t.derivative(P, T, phy=units.ANGLE)

#%%

time = np.linspace(0, T, n)
v = t.derivative(b_, T, phy=units.ANGLE)
plt.plot(time, -r3.torque)
plt.plot(time, v)
plt.plot(time, v * -r3.torque)
plt.plot(time, [np.trapz((v * -r3.torque)[1:x], dx=T / n) for x in range(0, n)])

plt.show()
print(np.trapz((-r3.torque*v)[1:-1], dx=T/n))  # calcule de l'énergie nécessaire

#%%

# anim = t.animate([[r1.point, H, r2.point, r4.point, r3.point], [H, P]], list_vectors=[(P, VP)], vector_scale=0.5)
# plt.show()