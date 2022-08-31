# Parameters

*These representations show a sphere making contact with a plane, but in reality the sphere has a null radius*

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180995202-8aa4efe1-f6f4-4875-af6d-c4638de9d920.svg">
</p>

There are 5 parameters to initialize a pin-slot joint using the `System.add_pin_slot` method :

- `s1`, int : Index of the first solid
- `s2`, int : Index of the second solid
- `a1`, float : Angle between the direction of the joint and the `x` axis in the frame of `s1`
- `d1`, float : Algebraic distance from the origin of `s1` to the line of joint
- `p2`, tuple : Coordinates of the contact point in the frame of `s2`

It returns a `PinSlotJoint` object.
The values of `a1`, `d1` and `p2` can be changed at any time, even after the compilation 

# Cinématique

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/180994983-7e22c490-3204-499d-95a2-d96de7a14656.svg">
</p>

- `point`, 2darray :Coordinates of the contact point in the global frame
- `angle`, 1darray : Values of the angle between `s1` and `s2`
- `delta`, 1darray : Values of the algebraic distance between the origin of `s1` and the contact point along the direction of the joint

`s1` is the reference: the main angle is positive if `s2` is on the left of `s1` . Piloting the joint sets both `angle` and `delta`.
This joint is a combination of a revolute joint and a prismatic joint

# internal mechanical actions

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/181915275-bbe2af79-ad1d-4ce4-8bca-cdbdda294592.svg">
</p>

# Inputs

- `set_torque(t)` :  Defines an additional torque `t` apllied by `s2` on `s1` at the joint. `t` can be of type int/float for a constant torque, 1darray for a time dependent torque or fonction (no arguments) for any dependence you want, it has to return one of the previous types.
- `set_tangent(t)` : Defines a force `t` applied by `s2` on `s1` which is tangent to the axis of the joint. `t` can be of type int/float for a constant force, 1darray for a time dependent force or fonction (no arguments) for any dependence you want, it has to return one of the previous types.
For example, functions can depend on geometric/kinematic parameters of the mechanism even if it is not simulated yet.
It can be accessed after a dynamic/static simulation of the mechanism using the `input_tangent` or `input_torque` attribute.


# Outputs

- `normal`, 1darray : Forces applied by `s2` on `s1` perpendicularly to the direction of the joint, on the contact point
- `tangent`, 1darray : For blocked pin-slot joints only (and some related joints). Values of the tangent force `s2` needs to apply on `s1` to make the mechanism move like in the kinematic simulation, or not move if you are simulating statics. 
It is `None` when the joint is not blocked
- `torque`, 1darray : For blocked pin-slot joints only (and some related joints). Values of the torque `s2` needs to apply on `s1` to make the mechanism move like in the kinematic simulation, or not move if you are simulating statics. 
It is `None` when the joint is not blocked