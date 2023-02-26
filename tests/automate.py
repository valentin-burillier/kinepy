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
s16 = sys.add_solid(name='Bûche') # à enlever



R1, R2 = 43.6, 15.
#R1, R2 = 50, 31 # max
#R1, R2 = 14, 14 # min

r0 = sys.add_revolute(0, 1, p1=(197.4, 68.9))
r1 = sys.add_revolute(0, 3, p1=(78.2, 45.))
r2 = sys.add_revolute(1, 2, p1=(R1, 0.))
r3 = sys.add_revolute(2, 3, p1=(182., 0.), p2=(195., 0.))

r5 = sys.add_revolute(0, 5, p1=(250., 68.9))
r6 = sys.add_revolute(0, 7, p1=(70., 50.))
r7 = sys.add_revolute(5, 6, p1=(R2, 0))
r8 = sys.add_revolute(6, 7, p1=(200., 0), p2=(212., 0))

r9 = sys.add_revolute(6, 8, p1=(200. + 80., 0))
r10 = sys.add_revolute(8, 9, p1=(82.1, 0))
r11 = sys.add_revolute(0, 9, p1=(50, 125), p2=(130, 0))

cuisse, tibia = 64., 67.
r12 = pied_g = sys.add_revolute(0, 11, p1=(40., 150.))
r13 = pied_d = sys.add_revolute(0, 13, p1=(185., 150.))
r14 = genou_g = sys.add_revolute(11, 10, p1=(tibia, 0))
r15 = genou_d = sys.add_revolute(13, 12, p1=(tibia, 0))
r16 = hanche_g = sys.add_revolute(10, 8, p1=(cuisse, 0), p2=(82.1, 0))
r17 = hanche_d = sys.add_revolute(12, 8, p1=(cuisse, 0), p2=(82.1, 0))

r18 = sys.add_revolute(2, 4, p1=(182 + 42.7, 0))
# ps = sys.add_pin_slot(4, 0, d1=-23.4, p2=(262., 237. - 30)) # à mettre 
p = sys.add_prismatic(16, 4, d1=23.4) # à enlever
r19 = sys.add_revolute(0, 16, p1=(262., 237. - 30)) # à enlever

r20 = sys.add_revolute(8, 14, p1=(19, 0))
r21 = sys.add_revolute(14, 15, p1=(45, 0))
r22 = sys.add_revolute(4, 15, p2=(43, 0))

sys.add_gear(r0, r5, r=1, v0=-0.5)

sys.pilot(r0)
sys.compile()

sys._object.signs = {'2 RRR': -1, '3 RRR': -1, '4 RRR': 1, '5 RRR': -1, '6 RRR': 1, '7 RRP': -1, '8 RRR': -1}

a = t.sinusoidal_input(0, 2*np.pi, 2, 101, v_max=4)

#%%

sys.solve_kinematics(a)
P = s4.get_point((270, 0))

#%%

# print(np.sum((genou_d.point - pied_d.point) ** 2, axis=0) ** .5) # sais tu que tu un outils norm dans tools ?
# je supp que non, voici un tuto :
# print(t.norm(genou_d.point - pied_d.point))


anim = t.animate([
    [r0.point, r2.point, r3.point, r1.point], [r3.point, r18.point, P],  # saw motion
    [r5.point, r7.point, r8.point, r6.point], [r7.point, r9.point, r10.point, r11.point],  # body motion
    [pied_g.point, genou_g.point, hanche_g.point, genou_d.point, pied_d.point],  # buggy part
    [r20.point, r21.point, r22.point]  # bras
], anim_time=2)

plt.show()
#%%

anim = t.animate([[r18.point, P],
                  [r9.point, r10.point],
                  [pied_g.point, genou_g.point, hanche_g.point, genou_d.point, pied_d.point],
                  [r20.point, r21.point, r22.point]], anim_time=1)

