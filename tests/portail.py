# -*- coding: utf-8 -*-
"""
Created on Mon Feb 27 09:19:36 2023
@author: buril
"""

import numpy as np
import matplotlib.pyplot as plt
import kinepy as k
import kinepy.tools as t
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

a = t.direct_input(0, np.pi/2, 5, 101)
sys.solve_kinematics(a)
H = s1.get_point((d, 0))
P = s1.get_point((f, 0))

VP = t.get_speed(P, 5)

#%%
b_min, b_max = np.min(r3.angle), np.max(r3.angle)

sys.pilot(r3)

sys.compile()
sys.change_signs(-1)

b_ = t.sinusoidal_input(b_min, b_max, 5, 101, v_max=0.5)

sys.solve_kinematics(b_)
sys.solve_dynamics(5, inputs=b_)
H = s1.get_point((d, 0))
P = s1.get_point((f, 0))


VP = t.get_speed(P, 5)
VP = np.concatenate((VP, [[0.], [0.]]), axis=1)
print(VP.shape, P.shape)
#%%
#
anim = t.animate([[r1.point, H, r2.point, r4.point, r3.point], [H, P]], list_vectors=[(P, VP)], vector_scale=0.5)
# plt.show()
#%%

plt.plot(t.norm(VP)/1000)
plt.show()



