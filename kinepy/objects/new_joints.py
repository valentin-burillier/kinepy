from kinepy.units import Physics, kinepy_class
from kinepy.objects.new_solid import Solid

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
