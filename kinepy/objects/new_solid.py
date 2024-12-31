from kinepy.units import Physics


@Physics.class_
class Solid:
    name: str
    index: int
    mass: Physics.MASS
    moment_of_inertia: Physics.MOMENT_OF_INERTIA
    g: Physics.POINT

    def __init__(self, system, name, index, mass=0., moment_of_inertia=0., g=(0., 0.)):
        self._system = system
        self.name = name
        self._index = index
        self._mass = mass
        self._moment_of_inertia = moment_of_inertia
        self._g = g

    @property
    def index(self):
        return self._index

    @property
    def system(self):
        return self._system

