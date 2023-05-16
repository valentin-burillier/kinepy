import kinepy as k
from kinepy.units import *
import kinepy.tools as to
import kinepy.gui as kd

import numpy as np
import matplotlib.pyplot as plt

#%%

set_unit_system(DEFAULT_SYSTEM)
set_unit(LENGTH, METER)
set_unit(FORCE, KILONEWTON)
show_units()

#%%

sys = k.System()

s1 = sys.add_solid('Flèche')
s2 = sys.add_solid('Balancier')
s3 = sys.add_solid('Godet')
s4 = sys.add_solid('Corps vérin flèche')
s5 = sys.add_solid('Tige vérin flèche')
s6 = sys.add_solid('Corps vérin balancier')
s7 = sys.add_solid('Tige vérin balancier')
s8 = sys.add_solid('Corps vérin godet')
s9 = sys.add_solid('Tige vérin godet')
s10 = sys.add_solid('Bielle maitien')
s11 = sys.add_solid('Bielle')

r1 = sys.add_revolute(0, 1, (0, 2.1))
r2 = sys.add_revolute(0, 4, (1.1, 1.5))
p1 = sys.add_prismatic(4, 5)
r3 = sys.add_revolute(1, 5, (3.8, 0))
r4 = sys.add_revolute(1, 6, (4.5, 0))
r5 = sys.add_revolute(1, 2, (7.2, -4))
p2 = sys.add_prismatic(6, 7)
r6 = sys.add_revolute(2, 7, (-1.5, 0))
r7 = sys.add_revolute(2, 8, (0, 1))
p3 = sys.add_prismatic(8, 9)
r8 = sys.add_revolute(2, 10, (3.5, 0))
r9 = sys.add_revolute(10, 9, (0.8, 0))
r10 = sys.add_revolute(2, 3, (4.5, 0))
r11 = sys.add_revolute(10, 11, (0.8, 0))
r12 = sys.add_revolute(11, 3, (1.2, 0), (0.6, 0))

sys.pilot(p1, p2, p3)

sys.compile()

sys.change_signs({'2 RRR': 1, '3 RRR': -1,'4 RRR': -1, '5 RRR': -1})

#%%

def phase(p, num, value, n):
    num -= 1
    M = np.zeros((3, n))
    M[num] = np.linspace(p[num], value, n)
    p[num] = value
    M[(num+1)%3] = np.full((n,), p[(num+1)%3])
    M[(num+2)%3] = np.full((n,), p[(num+2)%3])
    return M

def shape(n):
    n //= 6
    phases = []
    p = [3, 3.5, 3] # 3-4.5, 3.5-5.5, 3-3.98
    phases.append(phase(p, 2, 5.5, n))
    phases.append(phase(p, 3, 3.98, n))
    phases.append(phase(p, 1, 4.5, n))
    phases.append(phase(p, 2, 3.5, n))
    phases.append(phase(p, 3, 3, n))    
    phases.append(phase(p, 1, 3, n))
    return np.concatenate(phases, axis=1)

def fill(n):
    n = int(n**0.333)
    L = []
    for x1 in np.linspace(3, 4.5, n):
        for x2 in np.linspace(3.5, 5.5, n):
            for x3 in np.linspace(3, 3.98, n):
                L.append([x1, x2, x3])
    return np.array(L).T
#%%
sys.solve_kinematics(shape(1000))

kd.system(sys)

kd.figure_size((800, 800))
kd.animation_time(10)
kd.grid()
kd.graduation()
kd.dark_mode()
kd.frames_of_reference()

kd.show()

#kd.save('pelleteuse.gif')

#%%

plt.axis('equal')

sys.solve_kinematics(fill(10000))
p = s3.get_point((-0.5, -1))
plt.scatter(p[0], p[1])
plt.plot((-5, 0), (0, 0))
