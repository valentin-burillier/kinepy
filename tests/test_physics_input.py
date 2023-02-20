from kinepy.units import *
from kinepy.interface.decorators import physics_input


class Test:
    def __init__(self):
        self._unit_system = UnitSystem()
        self._unit_system.set(ANGLE, DEGREE)

    @physics_input(LENGTH, ANGLE)
    def test0(self, length, angle):
        return length, angle

    @physics_input(LENGTH, ANGLE)
    def test1(self, length, angle=np.math.pi):
        return length, angle

    @physics_input(LENGTH, ANGLE)
    def test2(self, length=1., angle=np.math.pi):
        return length, angle

    @physics_input(LENGTH, ANGLE, '', LENGTH, ACCELERATION)
    def test3(self, length, angle, brandon, point=(1., 1.), gravity=(0., -G)):
        return length, angle, brandon, tuple(point), tuple(gravity)


TEST = Test()

assert TEST.test0(1., 90) == (.001, np.math.pi * .5)
assert TEST.test1(1.) == (.001, np.math.pi)
assert TEST.test1(length=1.) == (.001, np.math.pi)
assert TEST.test2() == (1., np.math.pi)
assert TEST.test2(angle=45) == (1., np.math.pi * .25)
assert TEST.test3(1., 180, 'Brandon') == (.001, np.math.pi, 'Brandon', (1., 1.), (0., -G))
assert TEST.test3(1., 180, brandon='Brandou', gravity=(1., 0.)) == (.001, np.math.pi, 'Brandou', (1., 1.), (1., 0))
