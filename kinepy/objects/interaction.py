import numpy as np

from kinepy.objects.config import Config
import kinepy.math.dynamics as dyn
from kinepy.objects.joints_solid import Solid
import kinepy.math.geometry as geo
import kinepy.units as u


@u.UnitSystem.class_
class Interaction:
    _config: None | Config

    def __init__(self):
        self._config: None | Config = None

    def add_action(self, solid: Solid, point: u.Length.point, force: u.Force.point, torque: u.Torque.phy):
        if self._config is None:
            raise ValueError('Add me to a system')
        if not solid.check_against(self._config, Config.SOLID):
            raise ValueError('This solid is not from the same system')
        dyn.Solid.add_action(self._config, solid._index, force, torque, point)

    def register_actions(self):
        """Override this method to apply your actions"""""


@u.UnitSystem.class_
class Gravity(Interaction):
    g: u.Acceleration.point

    def __init__(self, g: u.Acceleration.point = (0.0, -u.Acceleration.G.value)):
        Interaction.__init__(self)
        self._g = g

    def register_actions(self):
        self._config.results.solid_dynamics[:, Config.SOLID_DYN_FORCE, :] += np.einsum('m,i->mi', self._config.solid_physics[:, Config.SOLID_MASS], self._g)[..., np.newaxis]


class Inertia(Interaction):
    def register_actions(self):
        # shape (m, 2, n)
        solid_ori = geo.Orientation.get(self._config, slice(None))
        # shape (m, n)
        solid_angles = np.arctan2(solid_ori[:, 1, :], solid_ori[:, 0, :])
        # shape (m, n)
        inertia = self._config.solid_physics[:, (Config.SOLID_MOMENT_OF_INERTIA,)] * np.diff(solid_angles, n=2, axis=-1, prepend=float('NaN'), append=float('NaN')) * self._config.frame_time ** -2
        self._config.results.solid_dynamics[:, Config.JOINT_DYN_TORQUE] -= inertia

        # shape (m, 2, n)
        solid_g = self._config.results.solid_dynamics[:, Config.SOLID_DYN_G, :]
        inertia = self._config.solid_physics[:, (Config.SOLID_MASS,)] * np.diff(solid_g, n=2, axis=-1, prepend=float('NaN'), append=float('NaN')) * self._config.frame_time ** -2
        self._config.results.solid_dynamics[:, Config.SOLID_DYN_FORCE] -= inertia