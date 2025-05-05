import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid()
s2 = sys.add_solid(mass=3, g=(1, 0))

r1 = sys.add_revolute(s0, s1)
r2 = sys.add_revolute(s0, s2, p1=(1, 0))
#p1 = sys.add_prismatic(s0, s2, alpha1=np.pi/2)

gp = sys.add_gear_pair(r1, r2, r=-1/2, pressure_angle=0)
#sys.add_belt(r1, r2, r1=2.0, t0=10)
#sys.add_gear_rack(r1, p1, pressure_angle=np.pi/4)

sys.add_gravity((0, -10))

r1.pilot()
r1.work()


sys.determine_computation_order()
n = 101
sys.set_sim_parameters(n, 2)

#%%

angle = np.linspace(0, 2*np.pi, n)
r1.set_input(angle)

sys.solve_kinematics()

#%%

plt.plot(r1.get_value(), r2.get_value())
#plt.plot(r1.get_value(), p1.get_value())
plt.show()

#%%

# gear pair
sys.solve_dynamics()

#plt.plot(angle, r1.get_torque())
plt.plot(angle, r1.get_force()) # résultat opposé attendu : -45 au début puis 0 et +45 à la fin
#plt.plot(angle, r2.get_force()) # résultat attendu : angle = 0 : +75, angle = pi, +30, angle = 2pi : -15

plt.show()

#%%

# gear rack
sys.solve_dynamics()

#plt.plot(angle, r1.get_torque())
plt.plot(angle, r1.get_force()) # signe inversé en x
#plt.plot(angle, p1.get_force()) # signe inversé en x

plt.show()

#%%

# belt
sys.solve_dynamics()

plt.plot(angle, r1.get_torque()) # que des 0
#plt.plot(angle, r1.get_force()) # que des 0
#plt.plot(angle, r2.get_force()) # que des 0

plt.show()
