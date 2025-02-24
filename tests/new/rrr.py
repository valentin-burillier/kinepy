import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.get_ground()
s1 = sys.add_solid()
s2 = sys.add_solid()
s3 = sys.add_solid()

r1 = sys.add_revolute(s0, s1, p1=(7, 0))
r2 = sys.add_revolute(s1, s2, p1=(2, 0))
r3 = sys.add_revolute(s2, s3, p1=(6, 0))
r4 = sys.add_revolute(s0, s3, p1=(0, 3), p2=(5, 0))

sys.pilot(r1)

sys.determine_computation_order()

"""
https://www.geogebra.org/calculator/h9pm9rs9
"""

#%%

angle = np.linspace(0, 2*np.pi, 101).reshape(1, 101)

sys.solve_kinematics(angle)

#%%

plt.plot(s1.get_angle())





