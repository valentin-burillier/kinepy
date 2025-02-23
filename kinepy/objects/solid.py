import numpy as np

import kinepy.units as u
from kinepy.objects.system_element import SystemElement
import kinepy.math.geometry as geo


@u.UnitSystem.class_
class Solid(SystemElement):
    name: str
    mass: u.Mass.phy
    moment_of_inertia: u.MomentOfInertia.phy
    g: u.Length.point

    def __init__(self, system, name, index, mass=0., moment_of_inertia=0., g=(0., 0.)):
        SystemElement.__init__(self, system, index)
        self.name = name
        self._mass = mass
        self._moment_of_inertia = moment_of_inertia
        self._g = g
        self._continuous_angle = None
        self._3dof = None

    def get_origin(self) -> u.Length.point:
        return geo.Position.get(self._system._solid_values, self._index)

    def get_point(self, point: u.Length.point) -> u.Length.point:
        geo.Position.get(self._system._solid_values, self._index) + geo.Orientation.add(geo.Orientation.get(self._system._solid_values, self._index), point[:, np.newaxis])

    def get_angle(self) -> u.Angle.phy:
        if not self._system._solid_states[self._index]:
            self._system._solid_states[self._index] = True
            x, y = geo.Orientation.get(self._system._solid_values, self._index)
            self._continuous_angle = np.arctan2(y, x)
            geo.Orientation.make_angle_continuous(self._continuous_angle)
        return self._continuous_angle

    @property
    def x_joint(self):
        if self._3dof is None:
            self._3dof = self._system.add_3dof_to_ground(self)
        return self._3dof._ghost_joints[0]

    @property
    def y_joint(self):
        if self._3dof is None:
            self._3dof = self._system.add_3dof_to_ground(self)
        return self._3dof._ghost_joints[1]

    @property
    def angle_joint(self):
        if self._3dof is None:
            self._3dof = self._system.add_3dof_to_ground(self)
        return self._3dof._ghost_joints[2]

