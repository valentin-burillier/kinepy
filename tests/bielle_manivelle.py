import main as m
import numpy as np
import matplotlib.pyplot as plt
import tools as t

S = m.System(name='bielle-manivelle')

s1 = S.add(m.Solid())
s2 = S.add(m.Solid())
s3 = m.Solid()
s3['D'] = (1, 0)
S.add(s3)

P1 = S.add(m.Pivot(sol1=0, sol2=1))
P2 = S.add(m.Pivot(sol1=1, sol2=2, p1=(1, 0)))
P3 = S.add(m.Pivot(sol1=2, sol2=3, p1=(3, 0)))
G = S.add(m.Glissiere(sol1=0, sol2=3))

S.pilot(P1)

S.compile_kinematic()

data = np.linspace(0, 2*np.pi)
S.solve_kinematic(data)

A = P1.point
B = P2.point
C = P3.point
D = s3.get_point('D')

anim = t.animate([A, B, C, D])
plt.show()
