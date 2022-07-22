import kinepy.system as k

#%%

# test un cycle

S = k.System()

sols = [S.add_solid() for _ in range(3)]
#S.add_solid()

S.add_revolute(0, 1)
S.add_pin_slot(1, 2)
S.add_pin_slot(2, 0)
S.add_prismatic(1, 3)

S.pilot([1, 3])

S.compile()

print(S.kin_instr)

