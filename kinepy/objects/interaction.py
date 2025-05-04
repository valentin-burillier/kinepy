import numpy as np

from kinepy.objects.config import Config, ConfigState, ActionMode
from kinepy.objects.action import Action, InternalAction
import kinepy.math.dynamics as dyn
from kinepy.objects.joints_solid import Solid, Revolute
import kinepy.math.geometry as geo
import kinepy.units as u


@u.UnitSystem.class_
class Interaction:
    def __init__(self, config: Config, action_mapping: dict):
        self._config = config
        self._action_mapping: dict[int, int] = action_mapping

    def _claim_resources(self):
        pass

    def _set_actions(self):
        pass


class SystemInteraction(Interaction):
    def __getitem__(self, item: Solid) -> Action:
        assert self._config.state >= ConfigState.ALLOCATED_RESOURCES, "Call `System.set_sim_parameters` accessing InternalActions"
        if not item.check_against(self._config):
            raise KeyError(f'This solid {item} does not belong to the same system')
        try:
            return InternalAction(self._config, self._action_mapping[item._index])
        except KeyError as e:
            e.args = f"This interaction is not doing anything to {item}",
            raise

    def _claim_resources(self):
        if len(self._action_mapping) < len(self._config.solid_names):
            diff = len(self._config.solid_names) - len(self._action_mapping)
            _n_solid_indices = np.arange(len(self._action_mapping), len(self._config.solid_names))
            _action_index = self._config.action_config.shape[0]
            self._config.add_actions(
                np.r_['-1', _n_solid_indices[:, np.newaxis], diff * [[ActionMode.SOLID_G.value]], _n_solid_indices[:, np.newaxis]],
                np.zeros((diff, 2))
            )
            self._action_mapping.update(zip(_n_solid_indices, np.arange(_action_index, _action_index + diff)))


@u.UnitSystem.class_
class Gravity(SystemInteraction):
    g: u.Acceleration.point

    def __init__(self,  config: Config, action_mapping: np.array, g: u.Acceleration.point = (0.0, -u.Acceleration.G.value)):
        Interaction.__init__(self, config, action_mapping)
        self._g = g

    def _set_actions(self):
        g = self._config.solid_physics[:, Config.SOLID_MASS, np.newaxis, np.newaxis] * self._g
        self._config.results.action_values[list(self._action_mapping.values()), :, Config.ACTION_DYN_FORCE] = g
        self._config.results.solid_dynamics[...,  Config.SOLID_DYN_FORCE] += g


class Inertia(SystemInteraction):
    def _set_actions(self):
        if self._config.frame_time == 0.0:
            return
        # shape (m, n, 2)
        solid_ori = geo.Orientation.get(self._config, slice(None))
        # shape (m, n)
        solid_angles = np.arctan2(solid_ori[..., 1], solid_ori[..., 0])
        # shape (m, n)
        inertia = self._config.solid_physics[..., Config.SOLID_MOMENT_OF_INERTIA, np.newaxis] * np.diff(solid_angles, n=2, axis=-1,  prepend=float('NaN'), append=float('NaN')) * self._config.frame_time ** -2
        self._config.results.action_values[list(self._action_mapping.values()), :, Config.ACTION_DYN_TORQUE] = inertia
        self._config.results.solid_dynamics[..., Config.SOLID_DYN_TORQUE] -= inertia

        # shape (m, n, 2)
        solid_g = self._config.results.solid_dynamics[..., Config.SOLID_DYN_G]
        inertia = self._config.solid_physics[..., Config.SOLID_MASS, np.newaxis, np.newaxis] * np.diff(solid_g, n=2, axis=-1, prepend=float('NaN'), append=float('NaN')) * self._config.frame_time ** -2
        self._config.results.action_values[list(self._action_mapping.values()), :, Config.ACTION_DYN_FORCE] = inertia
        self._config.results.solid_dynamics[..., Config.SOLID_DYN_FORCE] -= inertia


@u.UnitSystem.class_
class LinearSpring(Interaction):
    k: u.SpringConstant.phy
    l0: u.Length.phy

    p1: u.Length.point
    p2: u.Length.point

    def __init__(self, config: Config, action_mapping: np.array, s1: Solid, s2: Solid, p1: u.Length.point = (0.0, 0.0), p2: u.Length.point = (0.0, 0.0), k: u.SpringConstant.phy = 0.0, l0: u.Length.phy = 0.0):
        Interaction.__init__(self, config, action_mapping)
        self._k = k
        self._l0 = l0
        self.s1 = s1
        self.s2 = s2
        self._s1 = s1._index
        self._s2 = s2._index
        self._p1 = np.array(p1)
        self._p2 = np.array(p2)

    def _set_actions(self):
        # shape (n, 2)
        p1, p2 = geo.Position.point(self._config, self._s1, self._p1), geo.Position.point(self._config, self._s2, self._p2)

        vector = p2 - p1
        length = geo.Geometry.mag(vector)
        unit = vector / length
        force = (length - self.l0) * self.k * unit
        self._config.results.action_values[self._action_mapping[self._s1], :, Config.ACTION_DYN_FORCE] = force
        self._config.results.solid_dynamics[self._s1, :, Config.SOLID_DYN_FORCE] += force
        self._config.results.action_values[self._action_mapping[self._s2], :, Config.ACTION_DYN_FORCE] = -force
        self._config.results.solid_dynamics[self._s2, :, Config.SOLID_DYN_FORCE] -= force


@u.UnitSystem.class_
class TwistingSpring(Interaction):
    k: u.Torque.phy
    a0: u.Angle.phy

    def __init__(self, config: Config, action_mapping: np.array, r: Revolute, k: u.Torque.phy = 0.0, a0: u.Angle.phy = 0.0):
        Interaction.__init__(self, config, action_mapping)
        self._k = k
        self._a0 = a0
        self.r = r

    def register_actions(self):
        torque = (self.r.get_value() - self._a0) * self._k

        self._config.results.action_values[self._action_mapping[self.r._s1], :, Config.ACTION_DYN_TORQUE] = torque
        self._config.results.solid_dynamics[self.r._s1, :, Config.SOLID_DYN_TORQUE] += torque
        self._config.results.action_values[self._action_mapping[self.r._s2], :, Config.ACTION_DYN_TORQUE] = -torque
        self._config.results.solid_dynamics[self.r._s2, :, Config.SOLID_DYN_TORQUE] -= torque
