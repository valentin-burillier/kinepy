import numpy as np

from kinepy.objects.config import Config
import kinepy.math.dynamics as dyn
from kinepy.objects.joints_solid import Solid, Revolute
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
        if not solid.check_against(self._config, self._config.solid_physics):
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
        if self._config.frame_time == 0.0:
            return
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


@u.UnitSystem.class_
class LinearSpring(Interaction):
    k: u.SpringConstant.phy
    l0: u.Length.phy

    p1: u.Length.point
    p2: u.Length.point

    def __init__(self, s1: Solid, s2: Solid, p1: u.Length.point = (0.0, 0.0), p2: u.Length.point = (0.0, 0.0), k: u.SpringConstant.phy = 0.0, l0: u.Length.phy = 0.0):
        Interaction.__init__(self)
        self._k = k
        self._l0 = l0
        self.s1 = s1
        self.s2 = s2
        self._p1 = p1
        self._p2 = p2

    def register_actions(self):
        # shape (2, n)
        p1, p2 = self.s1.get_point(self.p1), self.s2.get_point(self.p2)
        vector = p2 - p1
        length = np.sum(vector * vector, axis=0) ** 0.5
        unit = vector / length

        force = (length - self.l0) * self.k * unit
        self.add_action(self.s2, p2, -force, 0)
        self.add_action(self.s1, p1, force, 0)

        print("End spring")


@u.UnitSystem.class_
class TwistingSpring(Interaction):
    k: u.Torque.phy
    a0: u.Angle.phy

    def __init__(self, r: Revolute, k: u.Torque.phy = 0.0, a0: u.Angle.phy = 0.0):
        Interaction.__init__(self)
        self._k = k
        self._a0 = a0
        self.r = r

    def register_actions(self):
        torque = (self.r.get_value() - self.a0) * self.k

        self.add_action(self.r.s2, np.array([[0], [0]]), np.array([[0], [0]]), -torque)
        self.add_action(self.r.s1, np.array([[0], [0]]), np.array([[0], [0]]), torque)
