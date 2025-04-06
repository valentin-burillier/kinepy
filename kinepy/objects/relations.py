from kinepy.objects.joints_solid import PrimitiveJoint, Prismatic, Revolute, Solid
from kinepy.objects.config import Config, ConfigView
import kinepy.units as u


class Relation(ConfigView):
    v0 = ConfigView.physics_view(Config.RELATION, Config.RELATION_V0, u.Dimensionless.phy)
    __slots__ = 'j1', 'j2'

    def __init__(self, config: Config, index: int, j1: PrimitiveJoint, j2: PrimitiveJoint):
        self.j1, self.j2 = j1, j2
        ConfigView.__init__(self, config, index)


class _Gear(Relation):
    __slots__ = 'g1', 'g2'
    
    def __init__(self, config: Config, index: int, j1: PrimitiveJoint, j2: PrimitiveJoint, g1: Solid, g2: Solid):
        self.g1, self.g2 = g1, g2
        Relation.__init__(self, config, index, j1, j2)


class GearPair(_Gear):
    r = ConfigView.physics_view(Config.RELATION, Config.RELATION_R, u.Dimensionless.phy)
    pressure_angle = ConfigView.physics_view(Config.RELATION, Config.RELATION_PRESSURE_ANGLE, u.Angle.phy)


class GearRack(_Gear):
    r = ConfigView.physics_view(Config.RELATION, Config.RELATION_R, u.Length.phy)
    pressure_angle = ConfigView.physics_view(Config.RELATION, Config.RELATION_PRESSURE_ANGLE, u.Angle.phy)


class Belt(_Gear):
    r1 = ConfigView.physics_view(Config.RELATION, Config.RELATION_R1, u.Length.phy)
    r2 = ConfigView.physics_view(Config.RELATION, Config.RELATION_R2, u.Length.phy)
    t0 = ConfigView.physics_view(Config.RELATION, Config.RELATION_T0, u.Force.phy)


class Distant(Relation):
    r = ConfigView.physics_view(Config.RELATION, Config.RELATION_R, u.Dimensionless.phy)


class Effortless(Relation):
    r = ConfigView.physics_view(Config.RELATION, Config.RELATION_R, u.Dimensionless.phy)
