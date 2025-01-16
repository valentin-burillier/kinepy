import kinepy.units as u
from kinepy.objects.system_element import SystemElement
from kinepy.objects.joints import PrimitiveJoint
from kinepy.objects.solid import Solid


class Relation(SystemElement):
    """
    j2.value = j1.value * r + v0
    """
    j1: PrimitiveJoint
    j2: PrimitiveJoint
    
    def __init__(self, system, index, j1: PrimitiveJoint, j2: PrimitiveJoint, r=0.0, v0=0.0):
        SystemElement.__init__(self, system, index)
        self.j1, self.j2 = j1, j2

        self._r = r
        self._v0 = v0


@u.UnitSystem.class_
class GearRelation(Relation):
    _g1: None | Solid = None
    _g2: None | Solid = None
    pressure_angle: u.Angle.phy

    def __index__(self, system, index, j1: PrimitiveJoint, j2: PrimitiveJoint, r=0.0, v0=0.0, pa=0.0, g1=None, g2=None):
        Relation.__init__(self, system, index, j1, j2, r, v0)
        self._g1 = g1
        self._g1 = g2

        self._pressure_angle = pa


@u.UnitSystem.class_
class GearPair(GearRelation):
    r: u.Dimensionless.phy
    v0: u.Angle.phy


@u.UnitSystem.class_
class GearRack(GearRelation):
    r: u.Length.phy
    v0: u.Length.phy


class _NonGearRelation(Relation):
    def _get_r_unit(self) -> u.scalar_type:
        u1, u2 = self.j1.get_input_physics(), self.j2.get_input_physics()
        return u.UnitSystem._get_unit_value(u._PhysicsEnum.DIMENSIONLESS) if u1 == u2 else u.UnitSystem._get_unit_value(u2) / u.UnitSystem._get_unit_value(u1)

    @property
    def r(self):
        return self._r / self._get_r_unit()

    @r.setter
    def r(self, value):
        self._r = value * self._get_r_unit()

    @property
    def v0(self):
        return self._v0 / u.UnitSystem._get_unit_value(self.j2.get_input_physics())

    @v0.setter
    def v0(self, value):
        self._v0 = value * u.UnitSystem._get_unit_value(self.j2.get_input_physics())


class DistantRelation(_NonGearRelation):
    pass


class EffortlessRelation(_NonGearRelation):
    pass
