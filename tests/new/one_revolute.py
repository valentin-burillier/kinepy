import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid(g=(1, 0), mass=2)

r1 = sys.add_revolute(s0, s1)

r1.pilot()
r1.work()

sys.add_gravity(g=(0, -10))

sys.determine_computation_order()
n = 101
sys.set_sim_parameters(n, 5)

#%%

angle = np.linspace(0, 2*np.pi, n)
r1.set_input(angle)

sys.solve_kinematics()

#%%

kd = sys.kinematic_diagram()

#kd.add_solid_point(s1, (1, 0))

kd.show()


#%%

sys.solve_dynamics()

#%%

plt.plot(r1.get_value(), r1.get_torque())
