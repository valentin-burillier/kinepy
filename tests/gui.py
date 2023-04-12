# -*- coding: utf-8 -*-
"""
Created on Sun Apr  9 09:48:36 2023

@author: buril
"""

import numpy as np
import matplotlib.pyplot as plt
import kinepy as k
import kinepy.tools as t
print(k)

#%%

sys = k.System()

s1 = sys.add_solid('pignon entrée')
s2 = sys.add_solid('roue')
s3 = sys.add_solid('bielle')
s4 = sys.add_solid('piston')
s5 = sys.add_solid('glisseur')

r1 = sys.add_revolute(0, 1, p1=(-35, 35))
r2 = sys.add_revolute(0, 2)
r3 = sys.add_revolute(2, 3, p1=(30, 0))
r4 = sys.add_revolute(3, 4, p1=(50, 0))
p1 = sys.add_prismatic(0, 4, d1=10)
r5 = sys.add_revolute(4, 5, (40, 0))
ps = sys.add_pin_slot(0, 5, a1=np.pi/4, d1=-60, p2=(30, 0))

R = 1/2
gear = sys.add_gear(r1, r2, -R)

sys.pilot(r1)

sys.compile()

#%%

a = np.linspace(0, 4*np.pi, 1001)
sys.solve_kinematics(a)
P1 = s1.get_point((R*50/(R + 1), 0))
P2 = s2.get_point((50/(R + 1), 0))

#%%

anim = t.animate([P1, r1.point, r2.point, P2, r3.point, r4.point, r5.point, ps.point])





