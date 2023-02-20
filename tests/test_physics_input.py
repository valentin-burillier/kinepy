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


TEST = Test()

assert TEST.test0(1., 90) == (.001, np.math.pi * .5)
assert TEST.test1(1.) == (.001, np.math.pi)
assert TEST.test1(length=1.) == (.001, np.math.pi)
assert TEST.test2() == (1., np.math.pi)
assert TEST.test2(angle=45) == (1., np.math.pi * .25)
