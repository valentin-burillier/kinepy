import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

L = 3 # m
R = 0.03 # m
pho = 1 # kg/m

M = pho*L # kg
j = M*R**2/2 + M*L**2/12 # kg.m^2

#%%

Mc = Me = 2.8 # kg
Ml = 0.85 # kg
Mtot = Mc + Me + Ml

Ge = 0.8 # m
Gl = 0.425 # m
Gc = (M*L/2 - Ge*Me - Gl*Ml)/Mc # m
Gtot = (Gc*Mc + Ge*Me + Gl*Ml)/Mtot

#%%

barriere = kp.System()

bati = barriere.ground
lisse = barriere.add_solid(g=(L/2, 0), mass=M, moment_of_inertia=j) # sys réel
#lisse = barriere.add_solid(g=(Gtot, 0), mass=Mtot, moment_of_inertia=0) # maquette

manivelle = barriere.add_solid()

r1 = barriere.add_revolute(bati, lisse)
r2 = barriere.add_revolute(bati, manivelle, p1=(0, 0.11))
ps = barriere.add_pin_slot(lisse, manivelle, p2=(0.08, 0), alpha1=np.pi/4)

r2.pilot()
r1.work()

barriere.add_gravity()
barriere.add_inertia()
barriere.add_twisting_spring(r1, k=25.8, a0=2.1)

barriere.determine_computation_order()
n = 1001
barriere.set_sim_parameters(n, 5)

a, = barriere.get_steps_with_multiple_solutions()
a.solution_index = 1

angle = np.linspace(-0.55, np.pi + 0.55, n)
r2.set_input(angle)

#%%

barriere.solve_kinematics()

#%%

kd = barriere.kinematic_diagram()

kd.add_solid_point(lisse, (L, 0))

kd.show()

#%%

r = r1.get_value().derivative()/r2.get_value().derivative() # rapport de réduction

plt.plot(r2.get_value(), r)

#%%

r_gear = np.nanmax(r)

barriere.solve_dynamics()

gear_torque = np.copy(r1.get_torque()*r)

plt.plot(r1.get_value(), gear_torque)

#%%

#r1.work(False)
#r2.work()

barriere.solve_kinematics()
barriere.solve_dynamics()

plt.plot(r1.get_value(), r2.get_torque())
plt.plot(r1.get_value(), gear_torque)
