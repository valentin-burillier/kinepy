import matplotlib.pyplot as plt

from kinepy import System
from kinepy.tools import animate

s = System()
[s.add_solid() for _ in range(2)]
s0, s1, s2 = s._object.sols
r0 = s.add_revolute(0, 1)
r1 = s.add_revolute(0, 2, (100., 0))
r2 = s.add_revolute(1, 2, (300., 0), (250, 0.))

print(r0.p1, r0.p2)
print(r1.p1, r1.p2)
print(r2.p1, r2.p2)

s.compile()
s.solve_kinematics(((0.,),))
print('Origins', *(sol.origin[:, 0] for sol in (s0, s1, s2)), sep='\n', end='\n\n')
print('Angles', *(sol.angle[0] for sol in (s0, s1, s2)), sep='\n', end='\n\n')

anim = animate(((r0.point, r1.point, r2.point, r0.point),), anim_time=2)
print('Points', *(p[:, :] for p in (r0.point, r1.point, r2.point)), sep='\n', end='\n\n')
plt.show()