import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.get_ground()
s1 = sys.add_solid()

p1 = sys.add_prismatic(s0, s1)

sys.pilot(p1)

sys.determine_computation_order()

#%%

dist = np.linspace(0, 4, 101).reshape(1, 101)

sys.solve_kinematics(dist)

#%%

plt.plot(s1.get_origin()[0])

"""
D:\KINEPY\new 22.02.2025\kinepy-dev\kinepy-dev\kinepy\math\geometry.py:113: RuntimeWarning: divide by zero encountered in power
  return Geometry.sq_mag(vec) ** -0.5
D:\KINEPY\new 22.02.2025\kinepy-dev\kinepy-dev\kinepy\math\kinematics.py:56: RuntimeWarning: invalid value encountered in multiply
  total_rotation = Orientation.sub(s1_point, s2_point) * (Geometry.inv_mag(p1) * Geometry.inv_mag(p2))
D:\KINEPY\new 22.02.2025\kinepy-dev\kinepy-dev\kinepy\math\kinematics.py:60: RuntimeWarning: invalid value encountered in multiply
  Geometry.move_eq(eq2, solid_values, Position.get(solid_values, s1) + s1_point + Geometry.det_z(s1_point) * Geometry.inv_mag(p1) * value[np.newaxis, ...])
"""
