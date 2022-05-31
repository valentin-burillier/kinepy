import main as m
import numpy as np
import matplotlib.pyplot as plt
import tools as t
#%matplotlib qt

S = m.System()

S.add(s1 := m.Solid())
S.add(s2 := m.Solid())
S.add(s3 := m.Solid())

S.add(P1 := m.Pivot(sol1=0, sol2=1))
S.add(P2 := m.Pivot(sol1=1, sol2=2, p1=(1, 0)))
S.add(P3 := m.Pivot(sol1=2, sol2=3, p1=(3, 0)))
S.add(P4 := m.Pivot(sol1=0, sol2=3, p1=(2, 0), p2=(3, 0)))

S.pilot(P1)
S.block(P1)

S.compile_static({(1, 2, 3): -1})


data = -np.linspace(0, 2*np.pi, 101)
#data = np.array([0, np.pi/2])
S.solve_kinematic(data)

A, B, C, D = [L.point for L in S.liaisons]

anim = t.animate([A, B, C, D], list_vectors=[(B, t.get_speed(B, 1)), (C, t.get_speed(C, 1))])
plt.show()

"""
plt.figure()
plt.plot(data, t.make_continuous(P1.angle))
plt.plot(data, t.make_continuous(P2.angle))
plt.plot(data, t.make_continuous(P3.angle))
plt.plot(data, t.make_continuous(P4.angle))
#plt.plot(data, t.make_continuous(P4.angle))
plt.axis('equal')
plt.legend(['s0', 's1', 's2', 's3'])
"""


