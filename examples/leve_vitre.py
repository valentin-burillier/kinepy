import kinepy as k
from kinepy.units import *
import kinepy.tools as to
from kinepy.math.geometry import get_point, z_cross, unit
import numpy as np
import matplotlib.pyplot as plt

#%%

set_unit(SPEED, 0.001, unit='mm/s')
show_units()

#%%

rA, rB, l = 7, 70, 130

sys = k.System()

s1 = sys.add_solid('Gear wheel', m=0.) # 0.2
s2 = sys.add_solid('Crank', m=0., g=(l, 0)) # 0.5
s3 = sys.add_solid('Support arm', m=0., g=(l, 0)) # 0.5
s4 = sys.add_solid('Glass', m=1.5, g=(-l, 0))

r1 = sys.add_revolute(0, 1, p1=(rA+rB, 0))
r2 = sys.add_revolute(0, 2, p1=(0, 0))
gear = sys.add_gear(r1, r2, -rA/rB, np.pi)
r3 = sys.add_revolute(2, 3, p1=(l, 0), p2=(l, 0))
r4 = sys.add_revolute(3, 4, p1=(2*l, 0))
ps1 = sys.add_pin_slot(0, 3)
ps2 = sys.add_pin_slot(4, 2, p2=(2*l, 0))

sys.pilot(r1)

grav = sys.add_gravity()

sys.compile()
print(*sys._object.kin_instr, sep='\n')

sys.change_signs({'3 RRP':-1, '4 RRP':-1})

#%%

t, n = 3, 101
time = np.linspace(0, t, n)
angle = to.trapezoidal_input(-np.pi/4*rB/rA, np.pi/4*rB/rA, t, n, v_max=6, phy=ANGLE)
sys.solve_dynamics(angle, t)

#%%

P = s1.get_point((-rA, 0))
G_ = s2.get_point((-rB, 0))
_ps1, _ps2 = ps1._object, ps2._object
# isole s4
_ = to.animate([[G_, ps2.point, r4.point, ps1.point], [P, s1.origin]], list_vectors=[(s4.get_point(s4.g), np.reshape(s4.m * grav.g, (2, 1)) * np.ones((2, n)) * .001), (r4.point, -r4.force), (ps2.point, ps2.normal * z_cross(unit(_ps2.s1.angle + _ps2.a1)))], vector_scale=10)
# isole s3
# _ = to.animate([[G_, ps2.point, r4.point, ps1.point], [P, s1.origin]], list_vectors=[(r4.point, r4.force), (ps1.point, -ps1.normal * z_cross(unit(_ps1.s1.angle + _ps1.a1))), (r3.point, -r3.force)], vector_scale=10)
# isole s2
# _ = to.animate([[G_, ps2.point, r4.point, ps1.point], [P, s1.origin]], list_vectors=[(r2.point, -r2.force), (ps2.point, -ps2.normal * z_cross(unit(_ps2.s1.angle + _ps1.a1))), (r3.point, r3.force), (gear.contact_point, -gear.contact_force)], vector_scale=10)
# isole s1
# _ = to.animate([[G_, ps2.point, r4.point, ps1.point], [P, s1.origin]], list_vectors=[(r1.point, -r1.force), (gear.contact_point, gear.contact_force)], vector_scale=10)



plt.show()
#%%
print(_ps1.ghost_j2.s1, _ps1.ghost_j2.p1, _ps1.ghost_j2.s2, _ps1.ghost_j2.p2)
print(_ps2.ghost_j2.s1, _ps2.ghost_j2.p1, _ps2.ghost_j2.s2, _ps2.ghost_j2.p2)
plt.plot(*get_point(_ps1.ghost_j2, 0), label='ps1, ghost_j2, s1')
plt.plot(*get_point(_ps1.ghost_j2, 1), label='ps1, ghost_j2, s2')
plt.axis('equal')
plt.legend()
plt.show()


plt.plot(*get_point(_ps2.ghost_j2, 0), '.', label='ps2, ghost_j2, s1')
plt.plot(*get_point(_ps2.ghost_j2, 1), label='ps2, ghost_j2, s2')
plt.axis('equal')
plt.legend()
plt.show()

plt.plot(_ps1.ghost_sol.angle, label='ps1, ghost_sol, a')
plt.plot(_ps2.ghost_sol.angle, label='ps2, ghost_sol, a')
plt.legend()
plt.show()

plt.plot(*_ps1.ghost_sol.origin, label='ps1, ghost_sol, o')
plt.plot(*_ps2.ghost_sol.origin, '.', label='ps2, ghost_sol, o')
plt.axis('equal')
plt.legend()
plt.show()

# plt.plot(time, s4.origin[1])
# plt.plot(time, ps1.sliding)
# plt.show()
# #%%
#
#
# plt.plot(ps1.normal, label="normal1")
# plt.plot(ps2.normal, label="normal2")
# plt.plot(to.norm(gear.contact_force), label="contact norm")
# plt.plot()
# plt.legend()
# plt.show()
# #%%
#
# plt.plot(time, r1.torque)
# plt.show()