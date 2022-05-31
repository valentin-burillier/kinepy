# System

# Paramétrage

```python
S = m.System()

s1 = m.Solid() # fixe l'index de s1 à 1
S.add(s1)
s2 = S.add(m.Solid())
S.add(s3 := m.Solid())

S.add(P1 := m.Pivot(sol1=0, sol2=1))
S.add(P2 := m.Pivot(sol1=1, sol2=2, p1=(1, 0)))
S.add(P3 := m.Pivot(sol1=2, sol2=3, p1=(3, 0)))
S.add(P4 := m.Pivot(sol1=0, sol2=3, p1=(2, 0), p2=(3, 0)))
```

ou

```python
SOLIDS = (m.solid(), )*3
LINKAGES = (m.Pivot(sol1=0, sol2=1),
            m.Pivot(sol1=1, sol2=2, p1=(1, 0)),
            m.Pivot(sol1=2, sol2=3, p1=(3, 0)),
            m.Pivot(sol1=0, sol2=3, p1=(2, 0), p2=(3, 0)))

S = m.System(SOLIDS, LINKAGES)
```

# cinématique

```python
S.pilot([P1, P2])

sgns = {}
S.compile_kinematic(sgns)

data1 = np.linspace(0, 2*np.pi, 101)
data2 = np.linspace(-np.pi, np.pi, 101)
S.solve_kinematic([data1, data2])
```
