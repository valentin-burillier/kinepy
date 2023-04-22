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
s2 = sys.add_solid('Crank', m=0.5, g=(l, 0), j=to.parallelepiped_inertia(2*l, 30, 0.5))
s3 = sys.add_solid('Support arm', m=0.5, g=(l, 0), j=to.parallelepiped_inertia(2*l, 30, 0.5))
s4 = sys.add_solid('Glass', m=1.5, g=(-l, 0))

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

s4.add_force((0, -10), (0, 0))
# s4.add_force((0, -1000), (-2*l, 0))

#%%

t, n = 3, 1001

time = np.linspace(0, t, n)
angle = to.trapezoidal_input(-np.pi/4*rB/rA, np.pi/4*rB/rA, t, n, v_max=6, phy=ANGLE)

plt.grid()
plt.plot(time, to.derivative(angle, t, phy=ANGLE))
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Input angular velocity (in {get_unit(ANGULAR_VELOCITY)})')

plt.show()

#%%

# sys.solve_dynamics(angle, t)
sys.solve_statics(angle)

#%%

_ = to.animate([[s2.get_point((-rB, 0)), ps2.point, r4.point, ps1.point], [s1.get_point((-rA, 0)), s1.origin]])
plt.show()

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

plt.show()

#%%

print('Glass stroke :', round(np.max(glass_heigth) - np.min(glass_heigth)), get_unit(LENGTH))
print('Maximum glass speed :', round(np.nanmax(glass_speed)), get_unit(SPEED))

#%%

motor_torque = -r1.torque

plt.grid()
plt.plot(time, motor_torque)
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Motor torque (in {get_unit(TORQUE)})')

plt.show()

# Plusieurs truc à dire :
#     - discontinuité car discontinuité de l'accélération du au trapèze de vitesse
#     - les effets d'inertie ne s'appliquent que sur la phase d'accélérations et de décélération
#     - le maximum est atteint vers la moitié de la course
#     - le couple > 0 <=> le systeme combat en permanance le poid des pièces
#     - Le couple au début et plus faible qu'à la fin : les effets d'inertie s'oppose à la rotation du moteur au début et l'aide à la fin 


#%%

print('Maximum motor torque :', round(np.nanmax(motor_torque), 2), get_unit(TORQUE))

#%%

crank_torque = motor_torque*rB/rA

k = (crank_torque[1] - crank_torque[-2])/(r2.angle[1] - r2.angle[-2])
a0 = np.nanmean(crank_torque)/k

print('Torsion spring stiffness constant :', round(k, 2), f'{get_unit(TORQUE)}/{get_unit(ANGLE)}')
print('Preload angle :', round(a0, 1), get_unit(ANGLE), '-->', round(get_value(ANGLE)*a0/2/np.pi, 1) , 'r')

#%%

plt.grid()
plt.plot(time, crank_torque)
plt.plot(time, r2.angle - np.pi)
plt.plot(time, crank_torque - k*(r2.angle - np.pi + a0))

plt.show()

#%%

# le "-" de la formule précédente est enlevé car cela correspond au couple de la manivelle sur le bâti
r2.set_torque(lambda : k*(r2.angle - np.pi + a0))
# sys.solve_dynamics(angle, t)

#%%

motor_torque = -r1.torque

plt.grid()
plt.plot(time, motor_torque)
plt.xlabel(f'Time (in {get_unit(TIME)})')
plt.ylabel(f'Motor torque (in {get_unit(TORQUE)})')

plt.show()

#%%

print('Maximum motor torque :', round(np.nanmax(np.abs(motor_torque)), 2), get_unit(TORQUE))

# La puissance nécessaire du moteur est réduite par 5 donc c'est super cool ( =
