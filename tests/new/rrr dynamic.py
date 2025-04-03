import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid(mass=2)
s2 = sys.add_solid(mass=0)

r1 = sys.add_revolute(s0, s1, p1=(7, 0))
r2 = sys.add_revolute(s1, s2, p1=(6, 0), p2=(5, 0))
r3 = sys.add_revolute(s0, s2, p1=(0, 3))

sys.determine_computation_order()
n = 5
sys.set_frame_count(n)
sys.solve_kinematics()

sys.add_interaction(kp.Gravity())
"""
class Load(kp.Interaction):
    def register_actions(self):
        self.add_action(s1, s1.get_point((1, 0)), [[0], [-1]], 0)
sys.add_interaction(Load())
"""

sys.solve_dynamics()


#%%


plt.plot(r1.get_value())
plt.plot(r2.get_value())
plt.plot(r3.get_value())
plt.show()

#%%

print(r1.get_force())
print(r2.get_force())
print(r3.get_force())
