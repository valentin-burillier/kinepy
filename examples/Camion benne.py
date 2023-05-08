import kinepy as k
from kinepy.units import *
import kinepy.tools as to

import numpy as np
import matplotlib.pyplot as plt
from kinepy.gui import display


#%%

set_unit_system(DEFAULT_SYSTEM)
show_units()

#%%

sys = k.System()

s1 = sys.add_solid('Benne', m=15000, g=(0, 1000))
s2 = sys.add_solid('Compas inférieur')
s3 = sys.add_solid('Compas supérieur')
s4 = sys.add_solid('Corps du vérin')
s5 = sys.add_solid('Tige du vérin')

r1 = sys.add_revolute(0, 2, (-2050, 0))
r2 = sys.add_revolute(2, 4, (200, 0))
r3 = sys.add_revolute(2, 3, (1325, 0))
r4 = sys.add_revolute(3, 5, (300, 200))
r5 = sys.add_revolute(3, 1, (1325, 0))
r6 = sys.add_revolute(0, 1, p2=(2100, -200))
p1 = sys.add_prismatic(4, 5)

sys.pilot(p1)

sys.compile()

sys.change_signs({'2 RRR': 1, '3 RRR': -1})

sys.add_gravity()

#%%

n, t = 101, 3

l = np.linspace(900, 1400, n)

sys.solve_statics(l)

#%%

display(sys, animation_time=t)

#%%

plt.plot(l, -r6.angle)
plt.plot(l, to.distance(r1.point, r5.point))

#%%

plt.plot(l, np.abs(p1.tangent))















