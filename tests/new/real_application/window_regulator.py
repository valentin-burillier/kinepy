import kinepy as kp
import numpy as np
import matplotlib.pyplot as plt

#%%

L = 0.34 # m
r_gear_1 = 0.01 # m
r_gear_2 = 0.12 # m

m_arm = 0.4
j_arm = m_arm*L**2/12

m_gear = 0.1
j_gear = m_gear*r_gear_1**2/2

m_window = 4

#%%

window_regulator = kp.System()

door = window_regulator.ground
motor_shaft = window_regulator.add_solid(mass=m_gear, moment_of_inertia=j_gear)
main_arm = window_regulator.add_solid(g=(L/2, 0), mass=m_arm, moment_of_inertia=j_arm)
secondary_arm = window_regulator.add_solid(g=(L/2, 0), mass=m_arm, moment_of_inertia=j_arm)
glass = window_regulator.add_solid(g=(-L/2, 0), mass=m_window)

r1 = window_regulator.add_revolute(door, motor_shaft, p1=(r_gear_1 + r_gear_2, 0))
r2 = window_regulator.add_revolute(door, main_arm)
gp = window_regulator.add_gear_pair(r1, r2, -r_gear_1/r_gear_2)
r3 = window_regulator.add_revolute(main_arm, secondary_arm, p1=(-L/2, 0), p2=(L/2, 0))
r4 = window_regulator.add_revolute(secondary_arm, glass, p1=(L, 0))
ps1 = window_regulator.add_pin_slot(door, secondary_arm)
ps2 = window_regulator.add_pin_slot(glass, main_arm, p2=(-L, 0))

r2.pilot()
r1.work()

window_regulator.add_gravity()
window_regulator.add_inertia()
window_regulator.add_twisting_spring(r2, k=1, a0=-14)

window_regulator.determine_computation_order()
n = 1001
window_regulator.set_sim_parameters(n, 3)

#a, = window_regulator.get_steps_with_multiple_solutions()
#a.solution_index = 1

angle = np.linspace(-np.pi/4, np.pi/4, n)
r2.set_input(angle)

#%%

window_regulator.declare_chose_lowest_value(ps1.sliding)
window_regulator.declare_chose_lowest_value(ps2.sliding)

window_regulator.solve_kinematics()

#%%

kd = window_regulator.kinematic_diagram()

kd.add_solid_point(glass, (0, 0.1), False)
kd.add_solid_point(glass, (-0.4, 0.2), False)

kd.show()

#%%

window_regulator.solve_dynamics()

plt.plot(r2.get_value(), r1.get_torque())
