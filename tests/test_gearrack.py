# -*- coding: utf-8 -*-
"""
Created on Tue Apr  4 21:05:43 2023

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

R, d = 3, -5

s1 = sys.add_solid('Roue 1')
s2 = sys.add_solid('Crémaillere')

r1 = sys.add_revolute(0, 1)
p1 = sys.add_prismatic(0, 2, d1=d)

sys.add_gearrack(r1, p1, -R, 0, 0)

sys.pilot(r1)

sys.compile()

# s1.add_torque(.1)
# s1.add_torque(-.1)
# s2.add_force(np.ones(n)*np.array([[0], [-10]]), (-R2, 0))


#%%
  
a = t.direct_input(0, 2*np.pi, T, n)
sys.solve_dynamics(a)
P1 = s1.get_point((R, 0))
P2 = sys.sols[0].get_point((0, -d))

#%%

anim = t.animate([P1, r1.point, P2, s2.origin])
# anim.save('anim.gif')
plt.show()

#%%

