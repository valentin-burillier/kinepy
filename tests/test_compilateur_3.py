import kinepy.system as k
import random as rd

#%%

# test méca en arbre (que des Lp)

S = k.System()

sols = [S.add_solid() for _ in range(7)]

S.add_revolute(0, 1)
S.add_pin_slot(1, 2)
S.add_revolute(2, 3)
S.add_pin_slot(3, 4)
S.add_prismatic(4, 5)
S.add_pin_slot(3, 6)
S.add_prismatic(2, 7)

Lp = list(range(7))
rd.shuffle(Lp)
print(Lp)
S.pilot(Lp)

S.compile()

print()
for instr in S.kin_instr:
    print(instr)
print()
for instr in S.dyn_instr:
    print(instr)