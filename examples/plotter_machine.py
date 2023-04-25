import kinepy as k
from kinepy.units import *
import kinepy.tools as to

import numpy as np
import matplotlib.pyplot as plt

#%%

set_unit_system(DEFAULT_SYSTEM)
show_units()

#%%

sys = k.System()

l = 1000

s1 = sys.add_solid('Tige gauche')
s2 = sys.add_solid('Corps gauche')
s3 = sys.add_solid('Corps droit')
s4 = sys.add_solid('Tige droite')
s5 = sys.add_solid('Guide vertical')
s6 = sys.add_solid('Guide horizontal')

r1 = sys.add_revolute(0, 1)
r2 = sys.add_revolute(0, 4)
r3 = sys.add_revolute(2, 3)
p1 = sys.add_prismatic(1, 2)
p2 = sys.add_prismatic(3, 4)

r4 = sys.add_revolute(5, 2)
p3 = sys.add_prismatic(0, 6)
p4 = sys.add_prismatic(6, 5, a1=np.pi/2, a2=np.pi/2)

sys.pilot([p3, p4])

sys.compile()

#%%

t, n = 10, 1001


r = 100
x0, y0 = 600, -500
angle = np.linspace(0, 2*np.pi, n)

x, y = x0 - r*np.cos(angle), y0 + r*np.sin(angle)

plt.axis('equal')
plt.plot(x[:900], y[:900])


#%%


y0 = -400
x, y = np.linspace(0, l, n), np.full((n,), y0)

plt.axis('equal')
plt.plot(x[:900], y[:900])
