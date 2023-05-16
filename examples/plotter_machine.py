import kinepy as k
from kinepy.units import *
import kinepy.tools as to
import kinepy.gui as kd

import numpy as np
import matplotlib.pyplot as plt

#%%

set_unit_system(DEFAULT_SYSTEM)
show_units()

#%%

sys = k.System()

l = 1000

s1 = sys.add_solid('Tige gauche')
s2 = sys.add_solid('Corps gauche', m=1.)
s3 = sys.add_solid('Corps droit')
s4 = sys.add_solid('Tige droite')
s5 = sys.add_solid('Guide vertical')
s6 = sys.add_solid('Guide horizontal')

r1 = sys.add_revolute(0, 1)
r2 = sys.add_revolute(0, 4, p1=(l, 0))
r3 = sys.add_revolute(2, 3)
p1 = sys.add_prismatic(1, 2)
p2 = sys.add_prismatic(4, 3)

r4 = sys.add_revolute(5, 2)
p3 = sys.add_prismatic(0, 6)
p4 = sys.add_prismatic(6, 5, a1=np.pi/2, a2=np.pi/2)

sys.pilot(p3, p4)

sys.block(p1, p2)

sys.add_gravity()

sys.compile()

#%%

t, n = 5, 1001
time = np.linspace(0, t, n)

r = 350
x0, y0 = 600, -500
angle = to.sinusoidal_input(0, 2*np.pi, t, n, v_max=1.9)

x, y = x0 - r*np.cos(angle), y0 + r*np.sin(angle)

#%%

sys.solve_dynamics([x, y], t)

#%%

kd.system(sys)

kd.animation_time(t)
kd.grid()
kd.graduation()
kd.light_mode()
kd.dark_mode()
kd.frames_of_reference()

kd.show()
