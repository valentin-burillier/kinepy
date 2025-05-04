import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid(mass=2, g=(3, 0))
s2 = sys.add_solid(mass=0)

r1 = sys.add_revolute(s0, s1, p1=(7, 0))
r2 = sys.add_revolute(s1, s2, p1=(6, 0), p2=(5, 0))
r3 = sys.add_revolute(s0, s2, p1=(0, 3))

sys.determine_computation_order()
n = 1
sys.set_sim_parameters(n)
sys.solve_kinematics()

sys.add_interaction(kp.Gravity(g=(0, -1)))


class Load(kp.Interaction):
    def register_actions(self):
        self.add_action(s1, s1.get_point(s1.g), [[0], [-1]], 0)


sys.add_interaction(Load())


sys.solve_dynamics()


#%%
"""
https://www.geogebra.org/calculator/s9fbpzdt
"""


meca = np.c_[s1.get_origin(), s1.get_point((6, 0)), s2.get_origin()]
plt.plot(*meca)

P = np.c_[s1.get_point((3, 0)), s1.get_point((3, 0))+np.array([[0], [-2]])]
plt.plot(*P)

R3 = np.c_[s2.get_origin(), s2.get_origin() + r3.get_force()]
plt.plot(*R3)

R2 = np.c_[s1.get_point((6, 0)), s1.get_point((6, 0)) + r2.get_force()]
plt.plot(*R2)

R1 = np.c_[s1.get_origin(), s1.get_origin() + r1.get_force()]
plt.plot(*R1)

plt.axis('equal')
plt.show()

#%%

print(r1.get_force())
print(r2.get_force())
print(r3.get_force())
