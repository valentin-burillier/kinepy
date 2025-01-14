import kinepy as k
from kinepy.units import *
import kinepy.tools as to
import kinepy.gui as kd

import numpy as np
import matplotlib.pyplot as plt

#%%

set_unit_system(DEFAULT_SYSTEM)
#set_unit(SPEED, MILLIMETER_PER_SECOND)
show_units()

#%% 

D, d = 100, 70
d1 = 2*d - D
d2 = D - d

sys = k.System()

s1 = sys.add_solid('Porte-satellite')
s2 = sys.add_solid('Satellite')
s3 = sys.add_solid('Planétaire')

r1 = sys.add_revolute(0, 1)
r2 = sys.add_revolute(1, 2, p1=(d/2, 0))
r3 = sys.add_revolute(1, 3)
g1 = sys.add_gear_pair(r1, r2, r=-D / d2)
g2 = sys.add_gear_pair(r2, r3, r=-d2 / d1)


sys.pilot(r1)

sys.compile()

#%%

def change(d):
    d1 = 2*d - D
    d2 = D - d
    r2.p1 = (d/2, 0)
    g1.r = -D/d2
    g2.r = -d2/d1

#%%

t = 10
a = np.linspace(0, 2*np.pi, 1001)

sys.solve_dynamics(a, t)

#%%

kd.system(sys)

kd.animation_time(t)

kd.show()

#%%

M = []
d_range = range(51, 100)
for d in d_range:
    change(d)
    sys.solve_dynamics(a, t)
    M.append(s3.angle[-1]/2/np.pi)

#%%

plt.plot(d_range, M)
