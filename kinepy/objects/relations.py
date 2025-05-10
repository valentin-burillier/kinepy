import kinepy.objects.config as cfg
import kinepy.objects.joints_solid as jo_so


class Relation(cfg.ConfigView):
    def _array(self) -> cfg.Relations:
        return self._config.relations

    def __new__(cls, config: cfg.Config, index: int):
        _dict = {
            cfg.Relations.Type.GEAR_RACK: GearRack,
            cfg.Relations.Type.GEAR_PAIR: GearPair,
            cfg.Relations.Type.BELT: Belt,
            cfg.Relations.Type.DISTANT: Distant,
            cfg.Relations.Type.EFFORTLESS: Effortless
        }
        final_type = _dict[cfg.Relations.Type(config.relations.type_[index])]
        cls._create_subclass(final_type)

    v0 = cfg.Relations.v0()

    _j1 = cfg.Relations.j1()
    _j2 = cfg.Relations.j2()

    @property
    def j1(self) -> jo_so.Joint:
        return jo_so.Joint(self._config, self._j1)

    @property
    def j2(self) -> jo_so.Joint:
        return jo_so.Joint(self._config, self._j2)


class _Gear(Relation):
    _g1 = cfg.Relations.g1()
    _g2 = cfg.Relations.g2()
    _g_name = 'g', 'g'

    @classmethod
    def g_property(cls, int_prop: property, joint_prop: property, index):
        def getter(self: cls):
            if (_g := int_prop.__get__(self)) < 0:
                raise ValueError(f'{cls._g_name[index]}{index} was never set')
            joint: int = joint_prop.__get__(self)
            if _g == (s1 := self._config.joints.s1[joint]):
                return jo_so.SolidBase(self._config, s1)
            if _g == (s2 := self._config.joints.s2[joint]):
                return jo_so.SolidBase(self._config, s2)
            raise ValueError(f'{cls._g_name[index]}{index} is ill-formed')

        def setter(self: cls, value: jo_so.SolidBase):
            _g = value._index
            joint: int = joint_prop.__get__(self)
            if _g != self._config.joints.s1[joint] and _g != self._config.joints.s2[joint]:
                raise ValueError(f'{cls._g_name[index]}{index} must be one of j{index}\'s solids, (j{index} is {self._config.joints.names[joint]})')
            int_prop.__set__(self, _g)

        return property(getter, setter)


class GearPair(_Gear):
    _g_name = 'gear', 'gear'
    r = cfg.Relations.r()
    pressure_angle = cfg.Relations.gear_pressure_angle()

    gear1 = _Gear.g_property(_Gear._g1, Relation._j1, 0)
    gear2 = _Gear.g_property(_Gear._g2, Relation._j2, 1)


class GearRack(_Gear):
    _g_name = 'gear', 'rack'

    r = cfg.Relations.r()
    pressure_angle = cfg.Relations.gear_pressure_angle()

    gear1 = _Gear.g_property(_Gear._g1, Relation._j1, 0)
    rack2 = _Gear.g_property(_Gear._g2, Relation._j2, 1)


class Belt(_Gear):
    _g_name = 'pulley', 'pulley'
    r1 = cfg.Relations.belt_r1
    r2 = cfg.Relations.belt_r2
    t0 = cfg.Relations.belt_t0

    pulley1 = _Gear.g_property(_Gear._g1, Relation._j1, 0)
    pulley2 = _Gear.g_property(_Gear._g2, Relation._j2, 1)


class Distant(Relation):
    r = cfg.Relations.r()


class Effortless(Relation):
    r = cfg.Relations.r()
