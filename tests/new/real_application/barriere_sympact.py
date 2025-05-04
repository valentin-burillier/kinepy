import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

barriere = kp.System()

bati = barriere.ground
lisse = barriere.add_solid(g=(1, 0), mass=5, moment_of_inertia=3.4)
manivelle = barriere.add_solid()

r1 = barriere.add_revolute(bati, lisse)
r2 = barriere.add_revolute(bati, manivelle, p1=(0, 0.11))
ps = barriere.add_pin_slot(lisse, manivelle, p2=(0.08, 0), alpha1=np.pi/4)

r2.pilot()
r2.work()

#barriere.add_gravity()
barriere.add_inertia() # donne nan
#barriere.add_twisting_spring(r2, 10) # donne que des 0
#barriere.add_linear_spring(bati, lisse, p1=(-1, 1), p2=(1, 0), k=10) # donne que des 0 lorsque r1.work()

barriere.determine_computation_order()
n = 101
barriere.set_sim_parameters(n, 5)

a, = barriere.get_steps_with_multiple_solutions()
a.solution_index = 1

#%%

angle = np.linspace(-np.pi/4, 5/4*np.pi, n)
#angle = np.linspace(0, 1, n)**2
r2.set_input(angle)

barriere.solve_kinematics()

#%%

kd = barriere.kinematic_diagram()

kd.add_solid_point(lisse, (1, 0))

kd.show()

#%%

plt.plot(r2.get_value(), r1.get_value(), '.-')

#%%

barriere.solve_dynamics()

#%%

plt.plot(r1.get_value(), r2.get_torque())
