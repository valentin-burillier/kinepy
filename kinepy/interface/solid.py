import numpy as np

from kinepy.interface.metaclass import *
import kinepy.units as units
import kinepy.objects as obj
from kinepy.math.geometry import rvec, ZERO_F, ZERO_ARRAY_F
from kinepy.interface.decorators import physics_input_method, physics_output, FUNCTION_TYPE
from kinepy.interface.interactions import SolidExternal


class Solid(obj.Solid, metaclass=MetaUnit):
    _object: obj.Solid
    read_only = ('origin', LENGTH), ('angle', ANGLE)
    read_write = ('m', MASS), ('g', LENGTH), ('j', INERTIA)

    def __init__(self): # noqa
        self.external_actions = SolidExternal(self)

    def __repr__(self):
        return f'{self.rep} | {self._object.name}'

    @physics_output(LENGTH)
    @physics_input_method(LENGTH)
    def get_point(self, p):
        return self._object.origin + rvec(self._object.angle, p)

    @physics_input_method('', LENGTH)
    def add_force(self, f, p):
        if isinstance(f, (np.ndarray, tuple, list)) and len(f) == 2:
            f = np.array(f)
            if f.shape == (2,):
                f = f.reshape((2, 1))
            self.external_actions.append((lambda: f * units.SYSTEM[FORCE]), ZERO_F, p)
        elif isinstance(f, FUNCTION_TYPE):
            self.external_actions.append((lambda: f() * units.SYSTEM[FORCE]), ZERO_F, p)

    def add_torque(self, t):
        if isinstance(t, (int, float, np.ndarray)):
            self.external_actions.append(ZERO_ARRAY_F, (lambda: t * units.SYSTEM[TORQUE]), self._object.g)
        elif isinstance(t, FUNCTION_TYPE):
            self.external_actions.append(ZERO_ARRAY_F, (lambda: t() * units.SYSTEM[TORQUE]), self._object.g)
