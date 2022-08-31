# Mechanism representation
## Adding a solid

Ground is the only solid existing in a system by default.

- `add_solid(j=0., m=0., g=0., name='')` : Adds a solid to the system et returns the corresponding `Solid` object. 
For further information about the parameters and the features of this class, please check on [this](https://github.com/valentin-burillier/kinepy/blob/main/docs/Solid.md) (in french).

## Adding a joint

- `add_revolute(s1, s2, p1=(0., 0.), p2=(0., 0.))` : Adds a revolute joint between `s1` and `s2`, it returns the corresponding `RevoluteJoint` object. 
For further information about the parameters and the features of this class, please check on [this](https://github.com/valentin-burillier/kinepy/blob/main/doc_EN/Revolute.md).
- `add_prismatic(s1, s2, a1=0., d1=0., a2=0., d2=0.)` : Adds a prismatic joint between `s1` and `s2`, it returns the corresponding `PrismaticJoint` object. 
For further information about the parameters and the features of this class, please check on [this](https://github.com/valentin-burillier/kinepy/blob/main/docs/Prismatic.md) (in french).
- `add_pin_slot(s1, s2, a1=0., d1=0., p2=(0., 0.))` : Adds a pin-slot joint between `s1` and `s2`, it returns the corresponding `PinSlotJoint` object. 
For further information about the parameters and the features of this class, please check on [this](https://github.com/valentin-burillier/kinepy/blob/main/docs/Pin_slot.md) (in french).

*Note that `s1` is always the reference solid for kinematic and dynamic values of a joint.*

# Joint piloting and blocking

- `pilot(joints)` : Specifies which joints are piloted, you impose how these joits behave. 
The `joints` argument can be either a joint (of type `Joint`) or an iterable of joints. 
For the resolution, the order of entries must correspond to the specified order. It will be displayed in the console.
- `show_input()` : Shows the entry order.
- `block(joints)` : Joint is "blocked" when it can transfer mechanical actions through its degrees of freedom. 
By default, the piloted joints are the blocked joints. It is useful too make them different for inverse kinematics.
The `joints` argument can be either a joint (of type `Joint`) or an iterable of joints. 
Effort values are accessible via joint attributes (check each joint documentation for further information).

# Mechanical actions

- `add_spring(k, l0, s1, s2, p1=(0, 0), p2=(0, 0))` : Add a spirong of constant `k`, of unloaded length `l0` attached to `p1` in solid `s1` and to `p2` in solid `s2`.
By default, it is attached to the origins of the solids.
- `add_gravity(g=(0, -9.81))` : Add a constant gravitational field `g`. By default, `g` is Earth's gravitationnal field.
Other mechanical actions can be imposed on joints and solids.

# Compilation

- `compile()` : Once the mechanism is modeled, this method will establish the solving strategy for both statics et dynanics. 
It detects if the mechanism can't be solved. 
Some cycle may depend on a "sign" as a mechanism could be built in defferent ways with the same parrameters. 
You have to determine the signs corresponding to your system. 
It can be done by visualising the systme or by vérifing some kinematic outputs. 
`1` is the default sign. They are changed in the attribute `signs` of the system. 
Adding new mechanical actions or changing parameters of joints don't require a new compilation. 

[Example coming soon]

# Resolution

In each of the following resolution mode, `inputs` is the piloted joints entry. 
If a signle joint is piloted, `inputs` is a 1darra, if several joints are piloted `inputs` can be list/tuple of 1darrays or a 2darray. 
The order corresponds to the one specified by `show_input()`.

## Kinematics

- `solve_kinematics(inputs=None)` : Solves the kinematics of the system. Giving the postions of solids and the values of the joints.
If `inputs` is `None`, it uses last input.

## Dynamics

- `solve_dynamics(t, compute_kine=True, inputs=None)` :Solves the dynamics of the system. It determines the efforts transmitted by joints.
They are accessible as attributes of the concerned joitns. `t` is the total duration of the simulation. 
The argument `compute_kine` specifies if it solves kinematics, if `inputs` is `None`, it uses last input. 
Efforts are undefined at the first and the last value as the computation of acceleration has a too weak order of convergence.

## Statique/Quasi-statique

- `solve_statics(compute_kine=True, inputs=None)` : Quite the same as `solve_dynamics` but without inertia. As opposed to `solve_dynamics`, efforts are well defined.
