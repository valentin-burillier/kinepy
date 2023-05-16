import kinepy as k
from kinepy.units import *
import kinepy.tools as to

import numpy as np
import matplotlib.pyplot as plt

#%%

set_unit_system(DEFAULT_SYSTEM)
set_unit(SPEED, MILLIMETER_PER_SECOND)
show_units()

#%% 

rA, rB, l = 7, 70, 130

sys = k.System()

s1 = sys.add_solid('Gear wheel', m=0.2, j=to.cylinder_inertia(2*rA, 0.2))
s2 = sys.add_solid('Main arm', m=0.5, g=(l, 0), j=to.parallelepiped_inertia(2*l, 30, 0.5))
s3 = sys.add_solid('Secondary arm', m=0.5, g=(l, 0), j=to.parallelepiped_inertia(2*l, 30, 0.5))
s4 = sys.add_solid('Glass', m=1.5, g=(-l, 0))

sys.bill_of_materials()

r1 = sys.add_revolute(0, 1, p1=(rA+rB, 0))
r2 = sys.add_revolute(0, 2, p1=(0, 0))
gear = sys.add_gear(r1, r2, -rA/rB, np.pi)
r3 = sys.add_revolute(2, 3, p1=(l, 0), p2=(l, 0))
r4 = sys.add_revolute(3, 4, p1=(2*l, 0))
ps1 = sys.add_pin_slot(0, 3)
ps2 = sys.add_pin_slot(4, 2, p2=(2*l, 0))

sys.pilot(r1)

sys.compile()

sys.change_signs({'3 RRP':-1, '4 RRP':-1})

#%%

sys.add_gravity()

s4.add_force((0, -2), (0, 0))
s4.add_force((0, -2), (-2*l, 0))

#%%

t, n = 3, 1001

time = np.linspace(0, t, n)
angle = to.trapezoidal_input(-np.pi/4*rB/rA, np.pi/4*rB/rA, t, n, v_max=6, phy=ANGLE)

plt.grid()
plt.plot(time, to.derivative(angle, t, phy=ANGLE))
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Input angular velocity (in {get_unit(ANGULAR_VELOCITY)})')

# plt.show()

#%%

sys.solve_dynamics(angle, t)
import kinepy.gui as gui
gui.system(sys)
gui.show()
#%%

_ = to.animate([[s2.get_point((-rB, 0)), ps2.point, r4.point, ps1.point], [s1.get_point((-rA, 0)), s1.origin]])
# plt.show()

#%%

glass_heigth = s4.origin[1]
glass_speed = to.derivative(glass_heigth, t)

plt.subplot(2, 1, 1)
plt.grid()
plt.plot(time, glass_heigth)
plt.ylabel(f'Glass height (in {get_unit(LENGTH)})')

plt.subplot(2, 1, 2)
plt.grid()
plt.plot(time, glass_speed)
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Glass speed (in {get_unit(SPEED)})')

# plt.show()

#%%

print('Glass stroke :', round(np.max(glass_heigth) - np.min(glass_heigth)), get_unit(LENGTH))
print('Maximum glass speed :', round(np.nanmax(glass_speed)), get_unit(SPEED))

#%%

input_torque = -r1.torque

plt.grid()
plt.plot(time, input_torque)
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Input torque (in {get_unit(TORQUE)})')

# plt.show()

#%%

print('Maximum input torque :', round(np.nanmax(input_torque), 2), get_unit(TORQUE))

#%%

main_arm_torque = input_torque*rB/rA

k = (main_arm_torque[1] - main_arm_torque[-2])/(r2.angle[1] - r2.angle[-2])
a0 = np.nanmean(main_arm_torque)/k

print('Torsion spring stiffness constant :', round(k, 2), f'{get_unit(TORQUE)}/{get_unit(ANGLE)}')
print('Preload angle :', round(a0, 1), get_unit(ANGLE), '-->', round(get_value(ANGLE)*a0/2/np.pi, 1) , 'r')

#%%

r2.set_torque(lambda : k*(r2.angle - np.pi + a0))
sys.solve_dynamics(angle, t)

#%%

input_torque = -r1.torque

plt.grid()
plt.plot(time, input_torque)
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Input torque (in {get_unit(TORQUE)})')

# plt.show()

#%%

print('Maximum input torque :', round(np.nanmax(np.abs(input_torque)), 2), get_unit(TORQUE))
