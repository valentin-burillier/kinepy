from kinepy.units import Physics, _PhysicsEnum
from kinepy.objects.joints import Prismatic, PrimitiveJoint
from kinepy.objects.solid import Solid


class Relation:
    """
    j2.value = j1.value * r + v0
    """

    j1: PrimitiveJoint
    j2: PrimitiveJoint

    def __init__(self, system, index, j1: PrimitiveJoint, j2: PrimitiveJoint, r=0.0, v0=0.0):
        self._system = system
        self._index = index
        self.j1, self.j2 = j1, j2

        self._r = r
        self._v0 = v0


@Physics.class_
class _GearRelation(Relation):
    _g1: None | Solid = None
    _g2: None | Solid = None
    pressure_angle: Physics.ANGLE

    def __index__(self, system, index, j1: PrimitiveJoint, j2: PrimitiveJoint, r=0.0, v0=0.0, pa=0.0, g1=None, g2=None):
        Relation.__init__(self, system, index, j1, j2, r, v0)
        self._g1 = g1
        self._g1 = g2

        self._pressure_angle = pa


@Physics.class_
class Gear(_GearRelation):
    r: Physics.DIMENSIONLESS
    v0: Physics.ANGLE


@Physics.class_
class GearRack(_GearRelation):
    r: Physics.LENGTH
    v0: Physics.LENGTH


class _NonGearRelation(Relation):
    # TODO: find a better name

    def _get_j1_physics(self) -> _PhysicsEnum:
        return _PhysicsEnum.LENGTH if isinstance(self.j1, Prismatic) else _PhysicsEnum.ANGLE

    def _get_j2_physics(self) -> _PhysicsEnum:
        return _PhysicsEnum.LENGTH if isinstance(self.j2, Prismatic) else _PhysicsEnum.ANGLE

    def _get_r_unit(self) -> Physics.scalar_type:
        u1, u2 = self._get_j1_physics(), self._get_j2_physics()
        return Physics._get_unit_value(_PhysicsEnum.DIMENSIONLESS) if u1 == u2 else Physics._get_unit_value(u2) / Physics._get_unit_value(u1)

    @property
    def r(self):
        return self._r / self._get_r_unit()

    @r.setter
    def r(self, value):
        self._r = value * self._get_r_unit()

    @property
    def v0(self):
        return self._v0 / Physics._get_unit_value(self._get_j2_physics())

    @v0.setter
    def v0(self, value):
        self._v0 = value * Physics._get_unit_value(self._get_j2_physics())


class DistantRelation(_NonGearRelation):
    pass


class EffortlessRelation(_NonGearRelation):
    pass
