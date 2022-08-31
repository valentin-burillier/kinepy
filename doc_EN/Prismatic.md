# Parameters

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180998717-29c55798-3157-4605-a70c-5fc98c37d6d7.svg">
</p>

There are 6 parameters to initialize a prismatic joint using the `System.add_prismatic` method:

- `s1`, int : Index of the first solid
- `s2`, int : Index of the second solid
- `a1`, float : Angle between the direction of the joint and the `x` axis in the frame of `s1`
- `a2`, float : Angle between the direction of the joint and the `x` axis in the frame of `s2`
- `d1`, float : Algebraic distance between the origin of `s1` and the line of joint 
- `d2`, float : Algebraic distance between the origin of `s2` and the line of joint

In this example: `d1` is positive and `d2` is negative
It returns a `PrismaticJoint` object. The values of `a1`, `a2`, `d1` and `d2` can be changed at any time, event after the compilation of the mechanism. 

# Kinematics

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180998713-5b020132-4f86-447d-b9d5-7e28a1be30cd.svg">
</p>

- `delta`, 1darray : Values of the algebraic distance between the origin of `s2` and the origin of `s1` along the direction of the joint

Piloting the joint sets `delta`.

# Internal mechanical actions

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/181913881-2f462197-1df0-4826-bf42-ef6c753074ef.svg">
</p>

## Inputs

- `set_tangent(t)` : Defines a force `t` applied by `s2` on `s1` which is tangent to the axis of the joint. `t` can be of type int/float for a constant force, 1darray for a time dependent force or fonction (no arguments) for any dependence you want, it has to return one of the previous types.
For example, functions can depend on geometric/kinematic parameters of the mechanism even if it is not simulated yet.
It can be accessed after a dynamic/static simulation of the mechanism using the `input_tangent` attribute.

## Outputs

- `normal`, 1darray : Forces applied by `s2` on `s1` perpendicularly to the direction of the joint
- `torque`, 1darray : Torques applied by `s2` on `s1` on the perpendicular projction of the origin of `s1` on the axis of the joint. 
- `tangent`, 1darray : For blocked prismatic joints only (and some related joints). Values of the tangent force `s2` needs to apply on `s1` to make the mechanism move like in the kinematic simulation, or not move if you are simulating statics. 
It is `None` when the joint is not blocked