"""
Created on Mon Dec 26 15:52:35 2022
@author: buril
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import kinepy.tools as t
import kinepy as k


#%%

sys = k.System()

# mains
s1 = sys.add_solid(name='Roue_scie')
s2 = sys.add_solid(name='Tige_1')
s3 = sys.add_solid(name='Bielle_1')
s4 = sys.add_solid(name='Scie')

# tete
s5 = sys.add_solid(name='Roue_corps')
s6 = sys.add_solid(name='Tige_2')
s7 = sys.add_solid(name='Bielle_2')

# corps
s8 = sys.add_solid(name='Corps')
s9 = sys.add_solid(name='Bielle_3')

# jambes
s10 = sys.add_solid(name='CuisseD')
s11 = sys.add_solid(name='TibiaD')
s12 = sys.add_solid(name='CuisseG')
s13 = sys.add_solid(name='TibiaG')

# bras
s14 = sys.add_solid(name='Bras')
s15 = sys.add_solid(name='Avant-bras')

# scie
s16 = sys.add_solid(name='Bûche')





R1, R2 = 43.6, 15
#R1, R2 = 50, 31 # max
#R1, R2 = 14, 14 # min

r1 = sys.add_revolute(0, 1, p1=(197.4, 68.9))
r2 = sys.add_revolute(0, 3, p1=(78.2, 45.))
r3 = sys.add_revolute(1, 2, p1=(R1, 0.))
r4 = sys.add_revolute(2, 3, p1=(182., 0.), p2=(195., 0.))

r6 = sys.add_revolute(0, 5, p1=(250., 68.9))
r7 = sys.add_revolute(0, 7, p1=(70., 50.))
r8 = sys.add_revolute(5, 6, p1=(R2, 0))
r9 = sys.add_revolute(6, 7, p1=(200., 0), p2=(212., 0))

r10 = sys.add_revolute(6, 8, p1=(200. + 80., 0))
r11 = sys.add_revolute(8, 9, p1=(82.1, 0))
r12 = sys.add_revolute(0, 9, p1=(50, 125), p2=(130, 0))

cuisse, tibia = 64, 67
r13 = sys.add_revolute(0, 11, p1=(40., 150.))
r14 = sys.add_revolute(0, 13, p1=(185., 150.))
r15 = sys.add_revolute(11, 10, p1=(tibia, 0))
r16 = sys.add_revolute(13, 12, p1=(tibia, 0))
r17 = sys.add_revolute(10, 8, p1=(cuisse, 0), p2=(82.1, 0))
r18 = sys.add_revolute(12, 8, p1=(cuisse, 0), p2=(82.1, 0))

r19 = sys.add_revolute(2, 4, p1=(182+42.7, 0))
p = sys.add_prismatic(16, 4, d1=23.4)
r20 = sys.add_revolute(0, 16, p1=(262., 237. - 30))

r21 = sys.add_revolute(8, 14, p1=(19, 0))
r22 = sys.add_revolute(14, 15, p1=(45, 0))
r23 = sys.add_revolute(4, 15, p2=(43, 0))

sys.add_gear(r1, r6, r=1, v0=-0.5)

sys.pilot(r1)

sys.compile()

#sys.signs = {(1, 2, 3): 1, (5, 6, 7): 1, (10, 8, 9): -1, (11, 15, 13): -1, (12, 16, 14): -1, (19, 17, 18) : -1}
sys.signs = {(1, 2, 3): 1, (5, 6, 7): 1, (10, 8, 9): -1, (11, 15, 13): -1, (12, 16, 14): -1, (19, 17, 18): -1, (22, 20, 21): 1}

a = np.linspace(0, 2*np.pi, 101)

#%%

sys.solve_kinematics(a)
P = s4.get_point((270, 0))

#%%

anim = t.animate([[r1.point, r3.point, r4.point, r2.point], [r4.point, r19.point, P],
                  [r6.point, r8.point, r9.point, r7.point],
                  [r8.point, r10.point, r11.point, r12.point],
                  [r13.point, r15.point, r17.point, r16.point, r14.point],
                  [r21.point, r22.point, r23.point]], anim_time=2)
display(True)

#%%

anim = t.animate([[r19.point, P],
                  [r10.point, r11.point],
                  [r13.point, r15.point, r17.point, r16.point, r14.point],
                  [r21.point, r22.point, r23.point]], anim_time=1)
display(False)

