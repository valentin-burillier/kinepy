import main as m
import numpy as np
import matplotlib.pyplot as plt
import tools as t

S = m.System()

S.add(s1 := m.Solid(name='eq1'))
S.add(s2 := m.Solid(name='eq2'))
S.add(s3 := m.Solid(name='eq3'))
s2['D'] = (1/2, 1/2)

S.add(P1 := m.Pivot(sol1=0, sol2=1))
S.add(P2 := m.Pivot(sol1=1, sol2=2, p1=(1, 0)))
S.add(P3 := m.Pivot(sol1=0, sol2=3, p1=(2, 0)))
S.add(G := m.Glissiere(sol1=2, sol2=3))

S.pilot(G)

sgns = {(0, 1, 2): -1}
S.compile_kinematic(sgns)

data = np.linspace(1, 3, 101) # marche super bien
#data = np.linspace(1.5, 2.5, 101) # ne marche pas
S.solve_kinematic(data)

A, B, C = [L.point for L in S.liaisons[:-1]]
D = s2.get_point('D')
anim = t.animate([A, B, D, B, C])
plt.show()

plt.figure()
plt.plot(data, t.make_continuous(P1.angle))
plt.plot(data, t.make_continuous(P2.angle))
plt.plot(data, t.make_continuous(P3.angle))
plt.plot(data, t.make_continuous(G.value))
plt.axis('equal')
plt.legend(['P1', 'P2', 'P3', 'G'])
