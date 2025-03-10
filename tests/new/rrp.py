import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

sys = kp.System()

s0 = sys.ground
s1 = sys.add_solid()
s2 = sys.add_solid()

r1 = sys.add_revolute(s0, s1, p1=(-1, 2))
r2 = sys.add_revolute(s0, s2, p1=(6, 4))
ps = sys.add_pin_slot(s1, s2, p1=(3, 0), distance2=1)

r1.pilot()

sys.determine_computation_order()
n = 101
sys.set_frame_count(n)

"""
Faire tourner le point C :
https://www.geogebra.org/calculator/uf4dfap6
"""

#%%

angle = np.linspace(0, 2*np.pi, n)
r1.set_input(angle)

sys.solve_kinematics()

#%%

plt.plot(s1.get_origin()[0], s1.get_origin()[1], '.-', label='s1.O')
plt.plot(*s1.get_point(np.array([3, 0])), '.-', label='ps.p1')
plt.plot(s2.get_origin()[0], s2.get_origin()[1], '.-', label='s2.O')
plt.plot(*s2.get_point(np.array([0, 1])), '.-', label='ps.d2') # hmmmmmmmmmm
plt.legend()
plt.axis('equal')
plt.show()


#%%

# la cinématique est bonne mais 
# décalage de pi ? distance inversé dans le paramétrage de ps ? s2 a l'envers ?
plt.plot(r1.get_value(), s2.get_angle(), label='s2')
plt.plot(r1.get_value(), r2.get_value(), label='r2')
