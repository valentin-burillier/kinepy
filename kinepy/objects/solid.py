import kinepy.units as u
from kinepy.objects.system_element import SystemElement


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
