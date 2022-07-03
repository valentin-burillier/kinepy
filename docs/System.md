# System

## Documentation
`System(sols=(), joints=(), piloted=(), signs=None)`


## Initialisation examples

```python
from kinepy.system import System

S = System() # "Ground" solid is created

# Solids initialisations
s1 = S.add_solid()
s2 = S.add_solid()
s3 = S.add_solid()

# Joints
P1 = S.add_revolute(sol1=0, sol2=1)
P2 = S.add_revolute(sol1=1, sol2=2, p1=(1., 0.))
P3 = S.add_revolute(sol1=2, sol2=3, p1=(3., 0.))
P4 = S.add_revolute(sol1=0, sol2=3, p1=(2., 0.), p2=(3., 0.))
```

```python
from kinepy.system import *

SOLIDS = (
    Solid([(0., 0.), (3., 0.)]),
    Solid([(0., 0.), (1., 0.)]),
    Solid([(0., 0.), (3., 0.)]),
    Solid([(0., 0.), (2., 0.)])
)

JOINTS = (
    RevoluteJoint(0, 1, 0, 0),
    RevoluteJoint(1, 2, 1, 0),
    RevoluteJoint(2, 3, 1, 0),
    RevoluteJoint(0, 3, 1, 1)
)

S = System(SOLIDS, JOINTS)
```

## Kinematics

```python
# To be continued
```
