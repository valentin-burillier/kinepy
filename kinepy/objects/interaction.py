import kinepy.objects.config as cfg
import kinepy.objects.joints_solid as jo_so
import kinepy.objects.action as act


class Interaction(cfg.ConfigView):
    _first_action = cfg.Interactions.first_action()

    def _array(self) -> cfg.Interactions:
        return self._config.interactions

    def __new__(cls, config: cfg.Config, index: int):
        _dict = {
            cfg.Interactions.Type.GRAVITY: Gravity,
            cfg.Interactions.Type.INERTIA: Inertia,
            cfg.Interactions.Type.LINEAR_SPRING: LinearSpring,
            cfg.Interactions.Type.TWISTING_SPRING: TwistingSpring
        }
        final_type = _dict[cfg.Interactions.Type(config.interactions.type_[index])]
        return cls._create_subclass(final_type)


class UniversalInteraction(Interaction):
    @cfg.ConfigView.assert_resources
    def __getitem__(self, item: jo_so.SolidBase) -> act.Action:
        return act.Action(self._config, self._first_action + item._index)


class Gravity(UniversalInteraction):
    g = cfg.Interactions.g_fields()


class Inertia(UniversalInteraction):
    pass


class LinearSpring(Interaction):
    k = cfg.Interactions.spring_stiffness()
    l0 = cfg.Interactions.spring_equilibrium_position()

    p1 = cfg.Interactions.linear_spring_p1
    p2 = cfg.Interactions.linear_spring_p2

    _s1 = cfg.Interactions.linear_spring_s1
    _s2 = cfg.Interactions.linear_spring_s2

    @property
    def s1(self) -> jo_so.SolidBase:
        return jo_so.SolidBase(self._config, self._s1)

    @property
    def s2(self) -> jo_so.SolidBase:
        return jo_so.SolidBase(self._config, self._s2)


class TwistingSpring(Interaction):
    k = cfg.Interactions.spring_stiffness()
    a0 = cfg.Interactions.spring_equilibrium_position()

    _r = cfg.Interactions.twisting_spring_revolute()

    @property
    def revolute(self) -> jo_so.Joint:
        return jo_so.Joint(self._config, self._r)
