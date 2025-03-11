import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid()
s2 = sys.add_solid()
s3 = sys.add_solid()

p1 = sys.add_prismatic(s0, s1)
p2 = sys.add_prismatic(s0, s2, alpha1=np.pi/2, alpha2=np.pi/2)
r1 = sys.add_revolute(s1, s3)
r2 = sys.add_revolute(s2, s3, p2=(5, 0))

r1.pilot()

sys.determine_computation_order()
n = 101
sys.set_frame_count(n)

"""
https://www.geogebra.org/calculator/dxqhvvm3
"""

#%%

angle = np.linspace(0, 2*np.pi, n)
r1.set_input(angle)

sys.solve_kinematics()

#%%

plt.plot(r1.get_value(), p1.get_value())
plt.show()
plt.plot(r1.get_value(), s1.get_origin()[0])
plt.plot(r1.get_value(), s3.get_origin()[0])
plt.plot(r1.get_value(), s2.get_origin()[1])
plt.show()

# plt.plot(r1.get_value(), p2.get_value())
plt.plot(*s2.get_origin())
plt.show()
plt.plot(*s3.get_point((1, 1)))
plt.show()
