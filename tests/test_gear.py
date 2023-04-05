# -*- coding: utf-8 -*-
"""
Created on Mon Apr  3 18:12:08 2023

@author: buril
"""

import numpy as np
import matplotlib.pyplot as plt
import kinepy as k
import kinepy.tools as t
print(k)

#%%

n, T = 101, 5

sys = k.System()

R1, R2 = 1, 4

s1 = sys.add_solid('Roue 1')
s2 = sys.add_solid('Roue 2')

r1 = sys.add_revolute(0, 1)
r2 = sys.add_revolute(0, 2, p1=(R1+R2, 0))

gear = sys.add_gear(r1, r2, -R1/R2, 0)

sys.pilot(r1)

sys.compile()

print(gear._object.common_eq)
s2.add_force(np.ones(n)*np.array([[0], [-10]]), (-R2, 0))


#%%
  
a = t.direct_input(0, 2*np.pi, T, n)*R2
sys.solve_dynamics(a)
P1 = s1.get_point((R1, 0))
P2 = s2.get_point((-R2, 0))


anim = t.animate([P1, r1.point, r2.point, P2], list_vectors=[(r2.point, -r2.force), (P2, np.ones(n)*np.array([[0], [-10]])), (gear.contact_point, -gear.contact_force)], vector_scale=0.1)
# anim.save('anim.gif')
plt.show()
plt.plot(r1.torque)
plt.show()
#%%

