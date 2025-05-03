import numpy as np

from kinepy.objects.joints_solid import PrimitiveJoint, Prismatic, Revolute, Solid
from kinepy.objects.config import Config, ConfigView
import kinepy.units as u


class Relation(ConfigView):
    def _config_arr(self) -> np.ndarray[int]:
        return self._config.relation_config

    def _physics(self) -> np.ndarray[float]:
        return self._config.relation_physics

    v0 = ConfigView._physics_view(Config.RELATION_V0, u.Dimensionless.phy)

    _j1 = ConfigView._config_view(Config.RELATION_J1)
    _j2 = ConfigView._config_view(Config.RELATION_J2)

    @property
    def j1(self) -> PrimitiveJoint:
        return PrimitiveJoint(self._config, self._j1)

    @property
    def j2(self) -> PrimitiveJoint:
        return PrimitiveJoint(self._config, self._j2)


class _Gear(Relation):
    _g1 = ConfigView._config_view(Config.RELATION_G1)
    _g2 = ConfigView._config_view(Config.RELATION_G2)
    _g_name = 'g', 'g'

    @classmethod
    def g_property(cls, int_prop: property, joint_prop: property, index):
        def getter(self: cls):
            if (_g := int_prop.__get__(self)) < 0:
                raise ValueError(f'{cls._g_name[index]}{index} was never set')
            joint: PrimitiveJoint = joint_prop.__get__(self)
            if _g == joint._s1:
                return joint.s1
            if _g == joint._s2:
                return joint.s2
            raise ValueError(f'{cls._g_name[index]}{index} is ill-formed')

        def setter(self: cls, value: Solid):
            _g = value._index
            joint: PrimitiveJoint = joint_prop.__get__(self)
            if _g != joint._s1 and _g != joint._s2:
                raise ValueError(f'{cls._g_name[index]}{index} must be one of j{index}\'s solids, (j{index} is {joint.name})')
            int_prop.__set__(self, _g)

        return property(getter, setter)


class GearPair(_Gear):
    _g_name = 'gear', 'gear'
    r = ConfigView._physics_view(Config.RELATION_R, u.Dimensionless.phy)
    pressure_angle = ConfigView._physics_view(Config.RELATION_PRESSURE_ANGLE, u.Angle.phy)

    gear1 = _Gear.g_property(_Gear._g1, Relation.j1, 0)
    gear2 = _Gear.g_property(_Gear._g2, Relation.j2, 1)


class GearRack(_Gear):
    _g_name = 'gear', 'rack'

    r = ConfigView._physics_view(Config.RELATION_R, u.Length.phy)
    pressure_angle = ConfigView._physics_view(Config.RELATION_PRESSURE_ANGLE, u.Angle.phy)

    gear1 = _Gear.g_property(_Gear._g1, Relation.j1, 0)
    rack2 = _Gear.g_property(_Gear._g2, Relation.j2, 1)


class Belt(_Gear):
    _g_name = 'shaft', 'shat'
    r1 = ConfigView._physics_view(Config.RELATION_R1, u.Length.phy)
    r2 = ConfigView._physics_view(Config.RELATION_R2, u.Length.phy)
    t0 = ConfigView._physics_view(Config.RELATION_T0, u.Force.phy)

    shaft1 = _Gear.g_property(_Gear._g1, Relation.j1, 0)
    shaft2 = _Gear.g_property(_Gear._g2, Relation.j2, 1)


class Distant(Relation):
    r = ConfigView._physics_view(Config.RELATION_R, u.Dimensionless.phy)


class Effortless(Relation):
    r = ConfigView._physics_view(Config.RELATION_R, u.Dimensionless.phy)
