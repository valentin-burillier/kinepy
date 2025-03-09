import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid()
s2 = sys.add_solid()
s3 = sys.add_solid()

r1 = sys.add_revolute(s0, s1, p1=(7, 0))
r2 = sys.add_revolute(s1, s2, p1=(2, 0))
r3 = sys.add_revolute(s2, s3, p1=(6, 0), p2=(5, 0))
r4 = sys.add_revolute(s0, s3, p1=(0, 3))

r1.pilot()

sys.determine_computation_order()
n = 101
sys.set_frame_count(n)

"""
Faire tourner le point C :
https://www.geogebra.org/calculator/h9pm9rs9
"""

#%%

angle = np.linspace(0, 2*np.pi, n)
r1.set_input(angle)

sys.solve_kinematics()

#%%

plt.plot(r4.get_value(), label='r4')
plt.plot(s3.get_angle(), label='s3')
plt.legend()
plt.show()

#%%

plt.plot(s1.get_origin()[0], s1.get_origin()[1], '.-', label='s1.O')
plt.plot(*s1.get_point(r2.p1), '.-', label='s1.r2.p1')
plt.plot(s2.get_origin()[0], s2.get_origin()[1], '.-', label='s2.O')
plt.plot(*s2.get_point(r3.p1), '.-', label='s2.r3.p1')
plt.plot(s3.get_origin()[0], s3.get_origin()[1], '.-', label='s3.O')
plt.plot(*s3.get_point(r3.p2), '.-', label='s3.r3.p2')
plt.legend()
plt.axis('equal')
plt.show()








