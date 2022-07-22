import kinepy.system as k

#%%

# test méca complexe

S = k.System()

sols = [S.add_solid() for _ in range(9)]

S.add_prismatic(0, 1)
S.add_revolute(1, 2)
S.add_revolute(2, 3)
S.add_prismatic(3, 6)
S.add_revolute(6, 8)
S.add_revolute(9, 8)
S.add_prismatic(8, 7)
S.add_revolute(5, 7)
S.add_revolute(5, 4)
S.add_prismatic(6, 4)
S.add_revolute(0, 5)
S.add_prismatic(1, 4)

S.pilot([0, 3, 5])

S.compile()

print()
for instr in S.kin_instr:
    print(instr)
print()
for instr in S.dyn_instr:
    print(instr)