import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid()
s2 = sys.add_solid()

r1 = sys.add_revolute(s0, s1)
r2 = sys.add_revolute(s0, s2)
#p1 = sys.add_prismatic(s0, s2)

gp = sys.add_gear_pair(r1, r2)
#b = sys.add_belt(r1, r2)
#gr = sys.add_gear_rack(r1, p1)

r1.pilot()

sys.determine_computation_order()
n = 101
sys.set_frame_count(n)

#%%

angle = np.linspace(0, 2*np.pi, n)
r1.set_input(angle)

sys.solve_kinematics()

#%%

plt.plot(r1.get_value(), r2.get_value())
plt.show()


