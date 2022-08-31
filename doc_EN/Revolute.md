# Parameters

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180985073-b8f29ecb-9536-444b-bb1d-f5c63a8198e6.svg">
</p>

There are 4 parameters to initialize a revolute joint using the `System.add_revolute` method:

- `s1`, int : Index of the first solid
- `s2`, int : Index of the first solid
- `p1`, tuple : Coordinates of the joint in the frame of reference of `s1`
- `p2`, tuple : Coordinates of the joint in the frame of reference of `s2`

It returns a `RevoluteJoint` object.
The values of `p1` and `p2` can be changed at any time, even after the compilation. 

# Kinematics

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180985253-4236026d-37ac-4d80-8c5a-075d7e914bbe.svg">
</p>

- `angle`, 1darray : Successive values of the angle between `s1` and `s2`
- `point`, 2darray : Coordinates of joint in the global frame of reference

`s1` est la référence : c'est par rapport à lui que l'angle de pivotement est exprimé. Le pilotage d'une liaison pivot permet de fixer l'attribut `angle`.

# Internal mechanical actions

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/181909316-8e097b1c-a33f-4740-a54a-1d95736e96df.svg">
</p>

## Input

- `set_torque(t)` : Defines an additional torque `t` apllied by `s2` on `s1` at the joint. `t` can of type int/float for a constant torque, 1darray for a time dependent torque or fonction (no arguments) for any dependence you want, it has to return one of the previous types.
For example, functions can depend on geometric/kinematic parameters of the mechanism even if it is not simulated yet.
It can be accessed after a dynamic/static simulation of the mechanism using the `input_torque` attribute.

## Outputs

- `force`, 2darray : Successive values of the force applied by `s2` on `s1` at the joint, it is described in the global frame of reference
- `torque`, 1darray : For blocked revolute joints only (and some related joints). Successive values of the torque`s2` needs to apply on `s1` at the joint to make the mechanism move like in the kinematic simulation. 
It is `None` when the joint is not blocked