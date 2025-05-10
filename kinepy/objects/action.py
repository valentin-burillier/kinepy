import kinepy.objects.config as cfg
import kinepy.objects.joints_solid as jo_so
import kinepy.math.geometry as geo


class Action(cfg.ConfigView):
    def _array(self) -> cfg.Actions:
        return self._config.actions

    def __new__(cls, config: cfg.Config, index: int):
        _dict: dict[cfg.Actions.Type, type[Action]] = {
            cfg.Actions.Type.INTERNAL_INTERACTION: InternalAction,
            cfg.Actions.Type.INTERNAL_OUTPUT: InternalAction,
            cfg.Actions.Type.USER: UserAction
        }
        final_type: type[Action] = _dict[cfg.Composite.Type(config.composite_joints.type_[index])]
        return cls._create_subclass(final_type)

    _ap = cfg.Actions.application_point()
    _force = cfg.Actions.force()
    _torque = cfg.Actions.torque()

    @cfg.ConfigView.assert_resources
    def get_application_point(self):
        return self._kp_array(self._ap.swapaxes(0, 1))

    @cfg.ConfigView.assert_resources
    def get_force(self):
        return self._kp_array(self._force.swapaxes(0, 1))

    @cfg.ConfigView.assert_resources
    def get_torque(self):
        return self._kp_array(self._torque)


class InternalAction(Action):
    """
    ReadOnly class
    """


class UserAction(Action):
    ap = cfg.Actions.user_point()
    _s = cfg.Actions.user_solid()

    @property
    def solid(self) -> jo_so.SolidBase:
        return jo_so.SolidBase(self._config, self._s)

    @cfg.ConfigView.assert_resources
    def set_force(self, value):
        """
        Force value is described in the global frame of reference
        """
        self._force = value

    @cfg.ConfigView.assert_resources
    def set_force_locally(self, value):
        """
        Force value is described in action's solid's frame of reference
        """
        self._force = geo.Position.vector(self._config, self._s, value)

    @cfg.ConfigView.assert_resources
    def set_torque(self, value):
        self._torque = value
