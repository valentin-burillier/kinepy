from kinepy.units import kinepy_class, Physics
import numpy as np


@kinepy_class
class Solid:
    name: str
    index: int
    mass: Physics.MASS
    moment_of_inertia: Physics.MOMENT_OF_INERTIA
    g: Physics.POINT

    def __init__(self, system, name, index, mass=0., moment_of_inertia=0., g=(0., 0.)):
        self._system = system
        self.name = name
        self._index = index
        self._mass = mass
        self._moment_of_inertia = moment_of_inertia
        self._g = g

    @property
    def index(self):
        return self._index

    @property
    def system(self):
        return self._system


class Joint:
    s1: int
    s2: int
    index: int

    def __init__(self, system, index, s1, s2):
        self._system = system
        self._index = index
        self.s1, self.s2 = s1, s2

    @property
    def index(self):
        return self._index

    @property
    def system(self):
        return self._system

    def get_all_joints(self):
        return self,


@kinepy_class
class Revolute(Joint):
    p1: Physics.POINT
    p2: Physics.POINT

    def __init__(self, system, index, s1, s2, p1=(0.0, 0.0), p2=(0.0, 0.0)):
        Joint.__init__(self, system, index, s1, s2)
        self._p1 = p1
        self._p2 = p2


@kinepy_class
class Prismatic(Joint):
    alpha1: Physics.ANGLE
    distance1: Physics.LENGTH
    alpha2: Physics.ANGLE
    distance2: Physics.LENGTH

    def __init__(self, system, index, s1, s2, alpha1=0.0, distance1=0.0, alpha2=0.0, distance2=0.0):
        Joint.__init__(self, system, index, s1, s2)
        self._alpha1 = alpha1
        self._distance1 = distance1
        self._alpha2 = alpha2
        self._distance2 = distance2


class GhostHolder(Joint):
    ghost_joints: tuple[Revolute | Prismatic, ...]

    def get_all_joints(self):
        return self.ghost_joints


def ghost_property(*args):
    def getter(self):
        return getattr(self.ghost_joints[args[0]], args[1])

    def setter(self, value):
        for index, attr_name in zip(args[::2], args[1::2]):
            setattr(self.ghost_joints[index], attr_name, value)

    return property(getter, setter)


class PinSlot(GhostHolder):
    p1: Physics.POINT = ghost_property(0, 'p1')
    alpha2: Physics.ANGLE = ghost_property(1, 'alpha1', 1, 'alpha2')
    distance2: Physics.LENGTH = ghost_property(1, 'distance2')

    def __init__(self, system, index, ghost_solid, ghost_joints, s1, s2):
        Joint.__init__(self, system, index, s1, s2)
        self.ghost_joints: tuple[Revolute, Prismatic] = ghost_joints
        self.ghost_solid: Solid = ghost_solid


class Translation(GhostHolder):
    alpha1: Physics.ANGLE = ghost_property(0, 'alpha1', 0, 'alpha2')
    distance1: Physics.LENGTH = ghost_property(0, 'distance1')
    alpha2: Physics.ANGLE = ghost_property(1, 'alpha1', 1, 'alpha2')
    distance2: Physics.LENGTH = ghost_property(0, 'distance1')

    def __init__(self, system, index, ghost_solid, ghost_joints, s1, s2):
        Joint.__init__(self, system, index, s1, s2)
        self.ghost_joints: tuple[Prismatic, Prismatic] = ghost_joints
        self.ghost_solid: Solid = ghost_solid


class ThreeDOF(GhostHolder):
    p: Physics.POINT = ghost_property(2, 'p')

    def __init__(self, system, index, ghost_solids, ghost_joints, s1, s2):
        Joint.__init__(self, system, index, s1, s2)
        self.ghost_joints: tuple[Prismatic, Prismatic, Revolute] = ghost_joints
        self.ghost_solids: tuple[Solid, Solid] = ghost_solids


class Relation:
    j1: int
    j2: int


@kinepy_class
class System:
    def __init__(self):
        self._solids: list[Solid] = [Solid(self, "Ground", 0)]
        self._joints: list[Joint] = []
        self._relations: list[Relation] = []
        self._blocked = []
        self._piloted = []

        self._dynamic_strategy = []
        self._kinematic_strategy = []

    def get_ground(self) -> Solid:
        return self._solids[0]

    def _check_solids(self, *solids: Solid):
        for solid in solids:
            if solid.system is not self:
                raise ValueError(f"Solid: {solid.name} does not belong to this system")

    def _check_joints(self, *joints: Joint):
        for joint in joints:
            if joint.system is not self:
                raise ValueError(f"Joint {joint} does not belong to this system")

    def add_solid(self, name='', mass: Physics.MASS = 0., moment_of_inertia: Physics.MOMENT_OF_INERTIA = 0., g: Physics.POINT = (0., 0.)) -> Solid:
        name = name or f'Solid {len(self._solids)}'
        self._solids.append(Solid(self, name, len(self._solids), mass, moment_of_inertia, g))
        return self._solids[-1]

    def add_prismatic(self, s1: Solid, s2: Solid, alpha1: Physics.ANGLE = 0.0, distance1: Physics.LENGTH = 0.0, alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> Prismatic:
        self._check_solids(s1, s2)
        result = Prismatic(self, len(self._joints), s1.index, s2.index, alpha1, distance1, alpha2, distance2)
        self._joints.append(result)
        return result

    def add_revolute(self, s1: Solid, s2: Solid, p1: Physics.POINT = (0.0, 0.0), p2: Physics.POINT = (0.0, 0.0)) -> Revolute:
        self._check_solids(s1, s2)
        result = Revolute(self, len(self._joints), s1.index, s2.index, p1, p2)
        self._joints.append(result)
        return result

    def add_pin_slot(self, s1: Solid, s2: Solid, p1: Physics.POINT = (0.0, 0.0), alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> PinSlot:
        self._check_solids(s1, s2)
        ghost_solid = Solid(self, f'GhostSolid {len(self._solids)}', len(self._solids))
        self._solids.append(ghost_solid)
        ghost_joints = (
            Revolute(self, len(self._joints), s1.index, ghost_solid.index, p1),
            Prismatic(self, len(self._joints)+1, ghost_solid.index, s2.index, alpha2, 0.0, alpha2, distance2)
        )
        self._joints.extend(ghost_joints)
        return PinSlot(self, -1, ghost_solid, ghost_joints, s1.index, s2.index)

    def add_translation(self, s1: Solid, s2: Solid, alpha1: Physics.ANGLE = 0.0, distance1: Physics.LENGTH = 0.0, alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> Translation:
        self._check_solids(s1, s2)
        ghost_solid = Solid(f'GhostSolid {len(self._solids)}', len(self._solids))
        self._solids.append(ghost_solid)
        ghost_joints = (
            Prismatic(self, len(self._joints), s1.index, ghost_solid.index, alpha1, distance1, alpha1, 0.0),
            Prismatic(self, len(self._joints)+1, ghost_solid.index, s2.index, alpha2, 0.0, alpha2, distance2)
        )
        self._joints.extend(ghost_joints)
        return Translation(self, -1, ghost_solid, ghost_joints, s1, s2)

    def add_3dof_to_ground(self, s: Solid, p: Physics.POINT = (0., 0.)) -> ThreeDOF:
        self._check_solids(s)
        ghost_solids = (
            Solid(self, f'GhostSolid {len(self._solids)}', len(self._solids)),
            Solid(self, f'GhostSolid {len(self._solids)+1}', len(self._solids)+1)
        )
        self._solids.extend(ghost_solids)
        ghost_joints = (
            Prismatic(self, len(self._joints), 0, ghost_solids[0].index),
            Prismatic(self, len(self._joints)+1, ghost_solids[0].index, ghost_solids[1].index, alpha1=np.pi * 0.5, alpha2=np.pi * 0.5),
            Revolute(self, len(self._joints)+2, ghost_solids[1].index, s.index, p2=p)
        )
        self._joints.extend(ghost_joints)
        return ThreeDOF(self, -1, ghost_solids, ghost_joints, 0, s.index)

    def pilot(self, *joints: Joint):
        self._check_joints(*joints)
        self._piloted.extend(joints)

    def block(self, *joints: Joint):
        self._check_joints(*joints)
        self._blocked.extend(joints)

    def determine_computation_order(self):
        pass

    def solve_kinematics(self):
        pass

    def solve_dynamics(self):
        pass
