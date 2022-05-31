import main as m
import numpy as np

# le graph représenté ici correspond à celui décrit dans le fichier statique

S = m.System()

sols = [S.add(m.Solid()) for _ in range(9)]

S.add(m.Pivot(sol1=0, sol2=1))
S.add(m.Pivot(sol1=1, sol2=2))
P2 = S.add(m.Pivot(sol1=2, sol2=3))
S.add(m.Pivot(sol1=3, sol2=6))
S.add(m.Pivot(sol1=6, sol2=4))
P1 = S.add(m.Pivot(sol1=1, sol2=4))
S.add(m.Pivot(sol1=4, sol2=5))
S.add(m.Pivot(sol1=0, sol2=5))
S.add(m.Pivot(sol1=5, sol2=7))
S.add(m.Pivot(sol1=7, sol2=8))
P3 = S.add(m.Pivot(sol1=8, sol2=9))
S.add(m.Pivot(sol1=6, sol2=8))

S.pilot([P1, P2, P3])
S.block([P1, P2, P3])

#S.compile_kinematic()
S.compile_static()

data = np.linspace(0, 2*np.pi)
S.solve_static([data]*3)
