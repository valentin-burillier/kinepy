from kinepy.units import *
import matplotlib.pyplot as plt
from kinepy.tools import direct_input, trapezoidal_input, sinusoidal_input
from kinepy.interface.decorators import physics_input_method, physics_input_function, physics_input_function_variable

set_unit(ANGLE, DEGREE)
set_unit(FORCE, DECANEWTON)


class Test:
    @physics_input_method(LENGTH, ANGLE)
    def test0(self, length, angle):
        return length, angle

    @physics_input_method(LENGTH, ANGLE)
    def test1(self, length, angle=np.pi):
        return length, angle

    @physics_input_method(LENGTH, ANGLE)
    def test2(self, length=1., angle=np.pi):
        return length, angle

    @physics_input_method(LENGTH, ANGLE, '', LENGTH, ACCELERATION)
    def test3(self, length, angle, brandon, point=(1., 1.), gravity=(0., -G[0])):
        return length, angle, brandon, tuple(point), tuple(gravity)


@physics_input_function(LENGTH, ANGLE)
def test0(length, angle):
    return length, angle


@physics_input_function(LENGTH, ANGLE)
def test1(length, angle=np.pi):
    return length, angle


@physics_input_function(LENGTH, ANGLE)
def test2(length=1., angle=np.pi):
    return length, angle


@physics_input_function(LENGTH, ANGLE, '', LENGTH, ACCELERATION)
def test3(length, angle, brandon, point=(1., 1.), gravity=(0., -G[0])):
    return length, angle, brandon, tuple(point), tuple(gravity)


@physics_input_function_variable(VARIABLE_UNIT, ANGLE, output=None)
def test4(u, angle):
    return u, angle


@physics_input_function_variable(VARIABLE_UNIT, VARIABLE_DERIVATIVE, output=ANGLE)
def test5(u, du):
    return u, du


TEST = Test()
assert TEST.test0(1., 90) == (.001, np.pi * .5)
assert TEST.test1(1.) == (.001, np.pi)
assert TEST.test1(length=1.) == (.001, np.pi)
assert TEST.test2() == (1., np.pi)
assert TEST.test2(angle=45) == (1., np.pi * .25)
assert TEST.test3(1., 180, 'Brandon') == (.001, np.pi, 'Brandon', (1., 1.), (0., -G[0]))
assert TEST.test3(1., 180, brandon='Brandou', gravity=(1., 0.)) == (.001, np.pi, 'Brandou', (1., 1.), (1., 0))

assert tuple(test4(1., 90, phy=LENGTH)) == (.001, np.pi * .5)
assert tuple(test4(90, 90, phy=ANGLE)) == (np.pi * .5, np.pi * .5)
assert tuple(test4(9, 90, phy=FORCE)) == (90., np.pi * .5)


plt.plot(sinusoidal_input(0, 100, 1, 1000, v_max=120, phy=LENGTH))
plt.show()
