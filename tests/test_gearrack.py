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
from kinepy.units import set_unit, show_units, LENGTH, METER, GOAT_HEIGHT, ANGLE

#%%

set_unit(LENGTH, METER)
show_units()

#%%

n, T = 101, 5

sys = k.System()

R, d = 3, -5

s1 = sys.add_solid('Roue 1')
s2 = sys.add_solid('Crémaillere')

r1 = sys.add_revolute(0, 1)
p1 = sys.add_prismatic(0, 2, d1=d)

gr = sys.add_gear_rack(r1, p1, R, 0, np.pi / 5)
print(f"{gr.pressure_angle = }")

sys.pilot(r1)

sys.compile()

F = np.ones(n)*np.array([[-10], [0]])
s2.add_force(F, (0, 0))


#%%
  
a = t.direct_input(0, 2*np.pi, T, n, phy=ANGLE)
sys.solve_dynamics(a)
P1 = s1.get_point((R, 0))
P2 = sys.ground.get_point((0, d))

#%%

anim = t.animate([P1, r1.point, P2, s2.origin], list_vectors=[(s2.origin, F), (gr.contact_point, gr.contact_force), (r1.point, -r1.force), (P2, -p1.normal*np.array([[0], [1]]))], vector_scale=0.2)
# anim.save('anim.gif')
plt.show()

#%%

