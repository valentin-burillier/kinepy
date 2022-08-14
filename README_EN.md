# Kinepy
This python library is dedicated to planar mechanism simulations.

It enables to analytically simulate complex mechanisms made of multiple kinematic cycles. 
It can easily compute position, velocity and acceleration of solids as well as mechanical actions of joints.
This tool can be used for early sizing purposes.
It supports optimisation algorithms and inverse kinematics.
Kinepy is built on the following hypotheses:

 - Mechanism is planar
 - Solids are rigid
 - Solids have constant mass
 - Joints are perfect (no friction, no slipping)
 - Systems are isostatic
 - System is made of dyads (order 1 Assur groups)
 - Ground is assiciated with an intertial frame of reference
 - Gravity is a uniform force field

You need to check on this [file](https://github.com/valentin-burillier/kinepy/blob/main/docs/utiliser_kinepy.md) (still in French) to know how simulate a mechanism.
Library's [documentation](https://github.com/valentin-burillier/kinepy/blob/main/docs) (still in French) is available too.

# Installation
Install the "kinepy" library like that:
```bash
pip install kinepy
```

# Examples

Very cool stuff

# TODO

 - Tests for dynamics/statics
 - Gears

# Links
 - https://cdfg.mit.edu/assets/files/CDMC_0.pdf, https://n.ethz.ch/~bthomasz/assets/PDF/Megaro17Compliant.pdf - What inspired this repository
 - http://faculty.tamucc.edu/psimionescu/PDFs/ASME%20DETC%202016%20-%2059086.pdf, https://blog.rectorsquid.com/linkage-mechanism-designer-and-simulator/ - Existing kinematics simulators
