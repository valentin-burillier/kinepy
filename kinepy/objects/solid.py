from kinepy.units import Physics
from kinepy.objects.system_element import SystemElement


@Physics.class_
class Solid(SystemElement):
    name: str
    mass: Physics.MASS
    moment_of_inertia: Physics.MOMENT_OF_INERTIA
    g: Physics.POINT

    def __init__(self, system, name, index, mass=0., moment_of_inertia=0., g=(0., 0.)):
        SystemElement.__init__(self, system, index)
        self.name = name
        self._mass = mass
        self._moment_of_inertia = moment_of_inertia
        self._g = g
