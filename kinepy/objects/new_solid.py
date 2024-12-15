from kinepy.units import kinepy_class, Physics
import numpy as np

@kinepy_class
class Solid:
    name: str
    index: int
    mass: Physics.MASS
    moment_of_inertia: Physics.MOMENT_OF_INERTIA
    g: Physics.POINT

    def __init__(self, name, index, mass=0., moment_of_inertia=0., g=(0., 0.)):
        self.name = name
        self.index = index
        self._mass = mass
        self._moment_of_inertia = moment_of_inertia
        self._g = g


class Joint:
    s1: int
    s2: int
    index: int

    def __init__(self, index, s1, s2):
        self.index = index
        self.s1, self.s2 = s1, s2


@kinepy_class
class Revolute(Joint):
    p1: Physics.POINT
    p2: Physics.POINT

    def __init__(self, index, s1, s2, p1=(0.0, 0.0), p2=(00, 0.0)):
        Joint.__init__(self, index, s1, s2)
        self._p1 = p1
        self._p2 = p2


@kinepy_class
class Prismatic(Joint):
    alpha1: Physics.ANGLE
    distance1: Physics.LENGTH
    alpha2: Physics.ANGLE
    distance2: Physics.LENGTH

    def __init__(self, index, s1, s2, alpha1=0.0, distance1=0.0, alpha2=0.0, distance2=0.0):
        Joint.__init__(self, index, s1, s2)
        self._alpha1 = alpha1
        self._distance1 = distance1
        self._alpha2 = alpha2
        self._distance2 = distance2


@kinepy_class
class PinSlot(Joint):
    p1: Physics.POINT
    alpha2: Physics.ANGLE
    distance2: Physics.LENGTH

    def __init__(self, index, ghost_solid, ghost_joints, s1, s2):
        Joint.__init__(self, index, s1, s2)
        self.ghost_joints: tuple[Revolute, Prismatic] = ghost_joints
        self.ghost_solid: Solid = ghost_solid


@kinepy_class
class Translation(Joint):
    alpha1: Physics.ANGLE
    distance1: Physics.LENGTH
    alpha2: Physics.ANGLE
    distance2: Physics.LENGTH

    def __init__(self, index, ghost_solid, ghost_joints, s1, s2):
        Joint.__init__(self, index, s1, s2)
        self.ghost_joints: tuple[Prismatic, Prismatic] = ghost_joints
        self.ghost_solid: Solid = ghost_solid


@kinepy_class
class ThreeDOF(Joint):
    p: Physics.POINT

    def __init__(self, index, ghost_solids, ghost_joints, s1, s2):
        Joint.__init__(self, index, s1, s2)
        self.ghost_joints: tuple[Prismatic, Prismatic, Revolute] = ghost_joints
        self.ghost_solids: tuple[Solid, Solid] = ghost_solids


class Relation:
    j1: int
    j2: int


class System:
    def __init__(self):
        self._solids: list[Solid] = [Solid()]
        self._joints: list[Joint] = []
        self._relations: list[Relation] = []

    def _add_solid(self, name, mass, moment_of_inertia, g) -> Solid:
        name = name if name else f'Solid {len(self._solids)}'
        self._solids.append(Solid(name, len(self._solids), mass, moment_of_inertia, g))
        return self._solids[-1]

    def add_solid(self, name='', mass: Physics.MASS = 0., moment_of_inertia: Physics.MOMENT_OF_INERTIA = 0., g: Physics.POINT = (0., 0.)) -> Solid:
        name = name if name else f'Solid {len(self._solids)}'
        self._solids.append(Solid(name, len(self._solids), mass, moment_of_inertia, g))
        return self._solids[-1]

    def add_prismatic(self, s1, s2, alpha1: Physics.ANGLE = 0.0, distance1: Physics.LENGTH = 0.0, alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> Prismatic:
        result = Prismatic(len(self._joints), s1, s2, alpha1, distance1, alpha2, distance2)
        self._joints.append(result)
        return result

    def add_revolute(self, s1, s2, p1: Physics.POINT = (0.0, 0.0), p2: Physics.POINT = (0.0, 0.0)) -> Revolute:
        result = Revolute(len(self._joints), s1, s2, p1, p2)
        self._joints.append(result)
        return result

    def add_pin_slot(self, s1, s2, p1: Physics.POINT = (0.0, 0.0), alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> PinSlot:
        ghost_solid = Solid(f'GhostSolid {len(self._solids)}', len(self._solids))
        self._solids.append(ghost_solid)
        ghost_joints = (
            Revolute(len(self._joints), s1, ghost_solid.index, p1),
            Prismatic(len(self._joints)+1, ghost_solid.index, s2, alpha2, distance2, alpha2, distance2)
        )
        self._joints.extend(ghost_joints)
        return PinSlot(-1, ghost_solid, ghost_joints)

    def add_translation(self, s1, s2, alpha1: Physics.ANGLE = 0.0, distance1: Physics.LENGTH = 0.0, alpha2: Physics.ANGLE = 0.0, distance2: Physics.LENGTH = 0.0) -> Translation:
        ghost_solid = Solid(f'GhostSolid {len(self._solids)}', len(self._solids))
        self._solids.append(ghost_solid)
        ghost_joints = (
            Prismatic(len(self._joints), s1, ghost_solid.index, alpha1, distance1, alpha1, distance1),
            Prismatic(len(self._joints)+1, ghost_solid.index, s2, alpha2, distance2, alpha2, distance2)
        )
        self._joints.extend(ghost_joints)
        return Translation(-1, ghost_solid, ghost_joints)

    def add_3dof_to_ground(self, s, p: Physics.POINT = (0., 0.)) -> ThreeDOF:
        ghost_solids = (
            Solid(f'GhostSolid {len(self._solids)}', len(self._solids)),
            Solid(f'GhostSolid {len(self._solids)+1}', len(self._solids)+1)
        )
        self._solids.extend(ghost_solids)
        ghost_joints = (
            Prismatic(len(self._joints), 0, ghost_solids[0].index),
            Prismatic(len(self._joints)+1, ghost_solids[0].index, ghost_solids[1].index, alpha1=np.pi * 0.5, alpha2=np.pi * 0.5),
            Revolute(len(self._joints)+2, ghost_solids[1].index, s, p2=p)
        )
        self._joints.extend(ghost_joints)
        return ThreeDOF(-1, ghost_solids, ghost_joints, 0, s)
