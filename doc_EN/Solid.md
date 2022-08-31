# Introduction

Each solid is associated with a unique coordinates system which use a direct orthonormal base to define the relative positions of the joints.
All lengths are described in millimeters and angles in radians. 

# Parameters
## Naming

- `name`, str : Name of the solid. A generic name is given by default.
- `rep`, int : Number identifying the solid, it is given automatically. Ground is number 0. `rep` increases by 1 for each new solid added to the system.

## Physical parameters

- `m`, float : Mass of the solid (in kg). 0 by default.
- `j`, float : Moment of intertia of the solid (in kg.m²). 0 by default.
- `g`, tuple : Coordinates of the center of intertia/gravity of the solid. (0, 0) by default.

These attributes can be changed at any time.

# Kinematics

<p align="center">
    <img width="50%" src="https://user-images.githubusercontent.com/93446869/181019881-8785a058-e1c5-452c-afc3-cc42e11b8e5d.svg">
</p>

The coordinates system is described by :

- `origin`, 2darray : Coordinates of the origin of frame of the solid in the global coordinate system.
- `angle`, 1darray : Values of the angle between both the x axes of the solid's frame and the global frame.

A point can be attached to the solid to get its positions.

- `get_point(p)`, 2darray : Returns the oordinates of the point `p` if the global frame (associated with Ground). Coordinates of `p` are given in the solid's frame. 
It can be a tuple or a ndarray of shape like (2, ...)

# External mechanical actions

- `add_force(f, p)` : Adds a force `f` apllied on the solid at `p`. `f` can be of type tuple for a constant force, 2darray for a time dependent force or fonction (no arguments) for any dependence you want, it has to return one of the previous types.
- `add_torque(t)` : Ajoute un couple `t` appliquée au solide. `t` can be of type int/float for a constant torque, 1darray for a time dependent torque or fonction (no arguments) for any dependence you want, it has to return one of the previous types.

For example, functions can depend on geometric/kinematic parameters of the mechanism even if it is not simulated yet.