import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid()

p1 = sys.add_prismatic(s0, s1)

p1.pilot()

sys.determine_computation_order()
n = 101
sys.set_frame_count(n)
#%%

p1.set_input(np.linspace(0, 4, n))

sys.solve_kinematics()

#%%
print(sys._System__config.results.solid_values)
print(sys.  _System__config.results.joint_values)
plt.plot(s1.get_origin()[0])
plt.show()

"""
D:\KINEPY\new 22.02.2025\kinepy-dev\kinepy-dev\kinepy\math\geometry.py:113: RuntimeWarning: divide by zero encountered in power
  return Geometry.sq_mag(vec) ** -0.5
D:\KINEPY\new 22.02.2025\kinepy-dev\kinepy-dev\kinepy\math\kinematics.py:56: RuntimeWarning: invalid value encountered in multiply
  total_rotation = Orientation.sub(s1_point, s2_point) * (Geometry.inv_mag(p1) * Geometry.inv_mag(p2))
D:\KINEPY\new 22.02.2025\kinepy-dev\kinepy-dev\kinepy\math\kinematics.py:60: RuntimeWarning: invalid value encountered in multiply
  Geometry.move_eq(eq2, solid_values, Position.get(solid_values, s1) + s1_point + Geometry.det_z(s1_point) * Geometry.inv_mag(p1) * value[np.newaxis, ...])
"""
