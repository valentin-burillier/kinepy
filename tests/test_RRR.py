import matplotlib.pyplot as plt

from kinepy import System
from kinepy.tools import animate

import numpy as np

s = System()
[s.add_solid() for _ in range(3)]
s0, s1, s2, s3 = s._object.sols

r0 = s.add_revolute(0, 1, (50., 0), (170, 25))
r1 = s.add_revolute(1, 2, (100., 0), (12, -13))
r2 = s.add_revolute(2, 3, (125., 0), (-30, 17))
r3 = s.add_revolute(0, 3, (300., -70.), (250, 0.))


s.pilot(r0)
s.compile()
s.solve_kinematics(np.linspace(0, 2 * np.pi, 1001))

anim = animate(((r0.point, r1.point, r2.point, r3.point, r0.point),), anim_time=2)
plt.show()
