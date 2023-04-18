import kinepy as k
from kinepy.units import *
import kinepy.tools as to
import numpy as np
import matplotlib.pyplot as plt

#%%

set_unit(SPEED, 0.001, unit='mm/s')
show_units()

#%%

rA, rB, l = 7, 70, 130

sys = k.System()

s1 = sys.add_solid('Gear wheel', m=0.2)
s2 = sys.add_solid('Crank', m=0.5, g=(l, 0))
s3 = sys.add_solid('Support arm', m=0.5, g=(l, 0))
s4 = sys.add_solid('Glass', m=1.5, g=(-l, 0))

r1 = sys.add_revolute(0, 1, p1=(rA+rB, 0))
r2 = sys.add_revolute(0, 2)
gear = sys.add_gear(r1, r2, -rA/rB, np.pi)
r3 = sys.add_revolute(2, 3, p1=(l, 0), p2=(l, 0))
r4 = sys.add_revolute(3, 4, p1=(2*l, 0))
ps1 = sys.add_pin_slot(0, 3)
ps2 = sys.add_pin_slot(4, 2, p2=(2*l, 0))

sys.pilot(r1)

sys.add_gravity()

sys.compile()

sys.change_signs({'3 RRP':-1, '4 RRP':-1})

#%%

t, n = 3, 101
time = np.linspace(0, t, n)
angle = to._trapezoidal_input(-np.pi/4*rB/rA, np.pi/4*rB/rA, t, n, v_max=6, phy=ANGLE)
sys.solve_dynamics(angle, t)

#%%a

P = s1.get_point((-rA, 0))
G = s2.get_point((-rB, 0))
_ = to.animate([[G, ps2.point, r4.point, ps1.point], [P, s1.origin]])
plt.show()
#%%

plt.plot(time, s4.origin[1])
plt.plot(time, ps1.sliding)
plt.show()
#%%

plt.plot(ps1.normal)
plt.plot(to.norm(r1.force))
plt.show()
