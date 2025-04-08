import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

"""
https://www.geogebra.org/calculator/v7a32dfu
"""

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid()
s2 = sys.add_solid()
s3 = sys.add_solid()

r1 = sys.add_revolute(s0, s1, p1=(0, 2))
r2 = sys.add_revolute(s2, s1, p2=(5, 0))
p1 = sys.add_prismatic(s0, s3)
p2 = sys.add_prismatic(s2, s3, alpha1=np.pi/2, alpha2=np.pi/2)

p1.pilot()
r1.work()

sys.determine_computation_order()
n = 101
sys.set_frame_count(n)

pa = (0, 1)

class Load(kp.Interaction):
    def register_actions(self):
        self.add_action(s3, s3.get_point(pa), [[1], [2]], 0)

sys.add_interaction(Load())


a, = sys.get_steps_with_multiple_solutions()
a.solution_index = 1

#%%

deplacement = np.linspace(0, 5, n)
p1.set_input(deplacement)

sys.solve_kinematics()

#%%

plt.plot(p1.get_value(), r1.get_value())

#%%

sys.solve_dynamics()

frame = 60

meca = np.c_[s1.get_origin()[:, frame], s2.get_origin()[:, frame], s3.get_origin()[:, frame]]
plt.plot(*meca)
plt.axis('equal')

P = np.c_[s3.get_point(pa)[:, frame], s3.get_point(pa)[:, frame] + np.array([1, 2])]
plt.plot(*P)

R1 = np.c_[s1.get_origin()[:, frame], s1.get_origin()[:, frame] + r1.get_force()[:, frame]]
plt.plot(*R1)

R2 = np.c_[s2.get_origin()[:, frame], s2.get_origin()[:, frame] + r2.get_force()[:, frame]]
plt.plot(*R2)

P2 = np.c_[s3.get_origin()[:, frame], s3.get_origin()[:, frame] + p2.get_force()[:, frame]]
plt.plot(*P2)

P1 = np.c_[s3.get_origin()[:, frame], s3.get_origin()[:, frame] + p1.get_force()[:, frame]]
plt.plot(*P1)

plt.plot()

#%%

plt.plot(p1.get_value(), p1.get_torque())
plt.plot(p1.get_value(), p2.get_torque())
plt.plot(p1.get_value(), r1.get_torque())
