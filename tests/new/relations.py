import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid()
s2 = sys.add_solid(mass=3, g=(1, 0))

#r1 = sys.add_revolute(s0, s1)
#r2 = sys.add_revolute(s0, s2, p1=(1, 0))
p1 = sys.add_prismatic(s0, s2, alpha1=np.pi/2)
p2 = sys.add_prismatic(s0, s1, alpha1=0)

#gp = sys.add_gear_pair(r1, r2, r=2, pressure_angle=0)
#belt = sys.add_belt(r1, r2, r1=0.5, r2=1.6, t0=10)
#sys.add_gear_rack(r1, p1, r=2, pressure_angle=np.pi/8)
hyd = sys.add_hydraulic_link(p1, p2, surface_ratio=2, v0=1)

sys.add_gravity((-5, -10))

p1.pilot()
p2.work()

sys.determine_computation_order()
n = 101
sys.set_sim_parameters(n, 2)

#%%

#angle = np.linspace(0, 6*np.pi, n)
#r1.set_input(angle)

sliding = np.linspace(0, 6, n)
p1.set_input(sliding)

sys.solve_kinematics()

#plt.plot(r1.get_value(), r2.get_value())
plt.plot(p1.get_value(), p2.get_value())

#%%

kd = sys.kinematic_diagram()

kd.add_solid_point(s1, (1, 0))
kd.add_solid_point(s2, (1, 0))

kd.show()


#%%

# hydraulic
sys.solve_dynamics()

plt.plot(p1.get_force()) # devrait être 0 selon Y
#plt.plot(p2.get_force())


#%%

# belt
#sys.solve_dynamics()

#plt.plot(angle, r1.get_torque(), label="Torque") # devrait être +15, 0, -15
#plt.plot(angle, r1.get_force(), label="Force1") # valeur en X ok mais en Y devrait être -15, 0, +15 (Couple_D/r1*(r2 - r1)/a = 15/0.4*0.2/1)  (* 2) r vs D ! -> 2/D = 1/r2 OK, mais D - d = 2 * (r2 - r1)
#plt.plot(angle, r2.get_force(), label="Force2") # valeur en X ok mais en Y devrait être 15, 30, 45 
#plt.legend()
#plt.show()


#%%

# gear pair
#sys.solve_dynamics()

#plt.plot(angle, r1.get_torque())
#plt.plot(angle, r1.get_force().swapaxes(0, 1)) # résultat opposé attendu : -45 au début puis 0 et +45 à la fin
#plt.plot(angle, r2.get_force()) # résultat attendu : angle = 0 : +75, angle = pi, +30, angle = 2pi : -15

#plt.show()

#%%


# gear rack
#sys.solve_dynamics()

#plt.plot(angle, r1.get_torque())
#plt.plot(angle, r1.get_force())
#plt.plot(angle, p1.get_force())

#plt.show()
