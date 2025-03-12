import numpy as np

from kinepy.objects.config import Config
import kinepy.math.dynamics as dyn
from kinepy.objects.joints_solid import Solid
import kinepy.math.geometry as geo
import kinepy.units as u


@u.UnitSystem.class_
class Interaction:
    def __init__(self):
        self._config: None | Config = None

    def add_action(self, solid: Solid, point: u.Length.point, force: u.Force.point, torque: u.Torque.phy):
        if self._config is None:
            raise ValueError('Add me to a system')
        if not solid.check_against(self._config, Config.SOLID):
            raise ValueError('This solid is not from the same system')
        dyn.Solid.add_action(self._config, solid._index, force, torque, point)

    def register_actions(self):
        pass


@u.UnitSystem.class_
class Gravity(Interaction):
    g: u.Acceleration.point

    def __init__(self, g: u.Acceleration.point = (0.0, -u.Acceleration.G.value)):
        Interaction.__init__(self)
        self._g = g

    def register_actions(self):
        dyn.Solid.add_action(self._config, slice(None), np.einsum('m,i->mi', self._config.solid_physics[:, 0], self._g)[..., np.newaxis], 0.0, self._config.results.solid_dynamics[:, 2:4, :])