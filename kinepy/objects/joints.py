import numpy as np

from kinepy.units import Physics, _PhysicsEnum
from kinepy.objects.system_element import SystemElement
from kinepy.objects.solid import Solid


class Joint(SystemElement):
    s1: Solid
    s2: Solid
    index: int

    def __init__(self, system, index: int, s1: Solid, s2: Solid):
        SystemElement.__init__(self, system, index)
        self.s1, self.s2 = s1, s2

    def get_all_joints(self):
        return self,


@Physics.class_
class Revolute(Joint):
    p1: Physics.POINT
    p2: Physics.POINT

    def __init__(self, system, index: int, s1: Solid, s2: Solid, p1=(0.0, 0.0), p2=(0.0, 0.0)):
        Joint.__init__(self, system, index, s1, s2)
        self._p1 = p1
        self._p2 = p2

    @staticmethod
    def get_input_physics() -> _PhysicsEnum:
        return _PhysicsEnum.ANGLE

    def _set_points(self):
        pass


@Physics.class_
class Prismatic(Joint):
    alpha1: Physics.ANGLE
    distance1: Physics.LENGTH
    alpha2: Physics.ANGLE
    distance2: Physics.LENGTH

    def __init__(self, system, index: int, s1: Solid, s2: Solid, alpha1=0.0, distance1=0.0, alpha2=0.0, distance2=0.0):
        Joint.__init__(self, system, index, s1, s2)
        self._alpha1 = alpha1
        self._distance1 = distance1
        self._alpha2 = alpha2
        self._distance2 = distance2
        self._p1 = 0., 0.
        self._p2 = 0., 0.

    @staticmethod
    def get_input_physics() -> _PhysicsEnum:
        return _PhysicsEnum.LENGTH

    def _set_points(self):
        self._p1 = (np.cos(self._alpha1), np.sin(self._alpha1)) * np.array(self._distance1)
        self._p2 = (np.cos(self._alpha2), np.sin(self._alpha2)) * np.array(self._distance2)


PrimitiveJoint = Revolute | Prismatic


class GhostHolder(Joint):
    _ghost_joints: tuple[PrimitiveJoint, ...]

    def get_all_joints(self) -> tuple[PrimitiveJoint, ...]:
        return self._ghost_joints


def _ghost_property(*args):
    def getter(self):
        return getattr(self._ghost_joints[args[0]], args[1])

    def setter(self, value):
        for index, attr_name in zip(args[::2], args[1::2]):
            setattr(self._ghost_joints[index], attr_name, value)

    return property(getter, setter)


class PinSlot(GhostHolder):
    p1: Physics.POINT = _ghost_property(0, 'p1')
    alpha2: Physics.ANGLE = _ghost_property(1, 'alpha1', 1, 'alpha2')
    distance2: Physics.LENGTH = _ghost_property(1, 'distance2')

    def __init__(self, system, index: int, ghost_solid, ghost_joints, s1: Solid, s2: Solid):
        Joint.__init__(self, system, index, s1, s2)
        self._ghost_joints: tuple[Revolute, Prismatic] = ghost_joints
        self._ghost_solid: Solid = ghost_solid


class Translation(GhostHolder):
    alpha1: Physics.ANGLE = _ghost_property(0, 'alpha1', 0, 'alpha2')
    distance1: Physics.LENGTH = _ghost_property(0, 'distance1')
    alpha2: Physics.ANGLE = _ghost_property(1, 'alpha1', 1, 'alpha2')
    distance2: Physics.LENGTH = _ghost_property(0, 'distance1')

    def __init__(self, system, index: int, ghost_solid, ghost_joints, s1: Solid, s2: Solid):
        Joint.__init__(self, system, index, s1, s2)
        self._ghost_joints: tuple[Prismatic, Prismatic] = ghost_joints
        self._ghost_solid: Solid = ghost_solid


class ThreeDOF(GhostHolder):
    p: Physics.POINT = _ghost_property(2, 'p2')

    def __init__(self, system, index: int, ghost_solids, ghost_joints, s1: Solid, s2: Solid):
        Joint.__init__(self, system, index, s1, s2)
        self._ghost_joints: tuple[Prismatic, Prismatic, Revolute] = ghost_joints
        self._ghost_solids: tuple[Solid, Solid] = ghost_solids
