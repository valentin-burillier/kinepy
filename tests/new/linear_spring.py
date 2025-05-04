import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid()

p1 = sys.add_prismatic(s0, s1)

p1.pilot()
p1.work()

sys.add_linear_spring(s0, s1, p1=(0, 0), p2=(0, 0), k=10, l0=0)

sys.determine_computation_order()
n = 101
sys.set_sim_parameters(n, 5)

#%%

sliding = np.linspace(0, 1, n)
p1.set_input(sliding)

sys.solve_kinematics()

#%%

kd = sys.kinematic_diagram()

#kd.add_solid_point(s1, (1, 0))

kd.show()


#%%

sys.solve_dynamics()

#%%

plt.plot(p1.get_value(), p1.get_force()[:, 0])
plt.show()