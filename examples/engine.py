import kinepy as k
from kinepy.units import *
import kinepy.tools as to
import kinepy.gui as kd

import numpy as np
import matplotlib.pyplot as plt

#%%

r, l = 80, 170

sys = k.System()

s1 = sys.add_solid('Cranckshaft', m=3, j=to.cylinder_inertia(1, 2*r), g=(-58, 0))
s2 = sys.add_solid('Connecting rod', m=1, j=to.parallelepiped_inertia(1, l, 15), g=(l/2, 0))
s3 = sys.add_solid('Piston', m=2)

r1 = sys.add_revolute(0, 1)
r2 = sys.add_revolute(1, 2, (r, 0))
r3 = sys.add_revolute(2, 3, (l, 0))
p1 = sys.add_prismatic(0, 3, a1=np.pi/2, a2=np.pi/2)

sys.pilot(r1)

sys.compile()

sys.change_signs({})

#%%

t, n = 1, 101

a = np.linspace(0, 2*np.pi, n)

sys.solve_dynamics(a, t)

#%%

kd.system(sys)

kd.animation_time(t)
kd.grid()
kd.graduation()
kd.light_mode()
kd.dark_mode()
kd.frames_of_reference()

kd.show()

#%%

to.animate([r1.point, r2.point, r3.point], list_vectors=[(r1.point, -r1.force)], anim_time=t, vector_scale=10)

#%%

plt.plot(-r1.force[0], -r1.force[1])
plt.axis('equal')

#%%

list_force = []
d = np.linspace(0, 2*r, 100)
for x in d:
    s1.g = (-x, 0)
    sys.solve_dynamics(a, t)
    list_force.append(np.nanmax(to.norm(r1.force)))
    
plt.plot(d, list_force)

print('rayon pour le g optimal :', d[np.argmin(list_force)])
