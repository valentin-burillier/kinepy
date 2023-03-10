import kinepy.objects as obj
from kinepy.interface.metaclass import *


class EffortlessRelation(obj.EffortlessRelation, metaclass=MetaUnit):
    _object: obj.EffortlessRelation

    def __init__(self): # noqa
        self.v0_phy = (ANGLE, LENGTH)[self._object.j2.id_ - 1]
        self.r_phy = (ANGLE, LENGTH)[self._object.j1.id_ - 1], (ANGLE, LENGTH)[self._object.j2.id_ - 1]
        self.v0 = self._object.v0
        self.r = self._object.r

    v0 = property(
        (lambda self: self.v0_ / self._unit_system[self.v0_phy][0]),
        lambda self, v: setattr(self, 'v0', v * self._unit_system[self.v0_phy][0])
    )

    r = property(
        (lambda self: self._object.r_ * self._unit_system[self.r_phy[1]][0] / self._unit_system[self.r_phy[0]][0]),
        lambda self, v: setattr(self._object, 'r', v * self._unit_system[self.r_phy[0]][0] /
                                self._unit_system[self.r_phys[1]][0])
    )


class DistantRelation(obj.DistantRelation, EffortlessRelation):
    pass


class Gear(obj.Gear, metaclass=MetaUnit):
    read_write = ('pressure_angle', ANGLE), ('v0', ANGLE), ('r', ADIMENSIONNED)


class GearRack(obj.GearRack, metaclass=MetaUnit):
    read_write = ('v0', LENGTH), ('r', LENGTH)
