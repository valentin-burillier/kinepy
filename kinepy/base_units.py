import numpy as np

FUNCTION_TYPE = type(lambda: None)


decor_divide = (lambda x, y: None if x is None else x / y), (lambda x, y: None if x is None else (lambda: x() / y))
decor_multiply = (lambda x, y: None if x is None else x * y), (lambda x, y: None if x is None else (lambda: x() * y))


class MetaUnit(type):
    def __new__(mcs, name, bases, dct):

        if 'read_only' in dct:
            for tag, phy, def_ in dct['read_only']:
                dct[tag] = MetaUnit.read_only(tag, phy, isinstance(def_, FUNCTION_TYPE))
                dct[f'{tag}_'] = def_
        if 'read_write' in dct:
            for tag, phy, def_ in dct['read_write']:
                dct[tag] = MetaUnit.read_write(tag, phy, isinstance(def_, FUNCTION_TYPE))
                dct[f'{tag}_'] = def_

        return type.__new__(mcs, name, bases, dct)

    @staticmethod
    def read_only(tag, phy, t):
        return property(
            (lambda self: decor_divide[t](self.__getattribute__(f'{tag}_'), self._unit_system[phy]))
        )

    @staticmethod
    def read_write(tag, phy, t):
        return property(
            (lambda self: decor_divide[t](self.__getattribute__(f'{tag}_'), self._unit_system[phy])),
            (lambda self, value: self.__setattr__(f'{tag}_', decor_multiply[t](value, self._unit_system[phy]))
                if value is not None else None)
        )


def physics_input(*args):
    def decor(f):
        def g(self, *args2):
            return f(
                self,
                *(arg if not t else decor_multiply[isinstance(arg, FUNCTION_TYPE)](arg, self._unit_system[t])
                    for t, arg in zip(args, args2))
            )
        return g
    return decor


def physics_output(phy):
    def decor(f):
        def g(self, *args):
            return decor_divide[0](f(self, *args), self._unit_system[phy])
        return g
    return decor


#  ------------------------------------------------ Length units -------------------------------------------------------

LENGTH = 'Length'
MILLIMETER = 1e-3
METER = 1.
CENTIMETER = 1e-2
INCH = 2.54e-2

#  ----------------------------------------------- Time units ----------------------------------------------------------

TIME = 'Time'
SECOND = 1.
MILLISECOND = 1e-3
MINUTE = 60.

# ------------------------------------------------ Angle units ---------------------------------------------------------

ANGLE = 'Angle'
RADIAN = 1.
DEGREE = np.pi / 180

# ----------------------------------------------- Acceleration units ---------------------------------------------------

ACCELERATION = 'Acceleration'
METER_PER_SQUARE_SECOND = 1.
G = 9.8067

# ---------------------------------------------- Force units -----------------------------------------------------------

FORCE = 'Force'
NEWTON = 1.
DECANEWTON = 10.
MILLINEWTON = 1e-3

# --------------------------------------------- Mass units -------------------------------------------------------------

MASS = 'Mass'
KILOGRAM = 1.
GRAM = 1e-3
POUND = 2.20462

# -------------------------------------------- Torque units ------------------------------------------------------------

TORQUE = 'Torque'
NEWTON_METER = 1.
MILLINEWTON_METER = 1e-3

# -------------------------------------------- Spring constant units ---------------------------------------------------

SPRING_CONSTANT = 'SpingConstant'
NEWTON_PER_METER = 1.,

# --------------------------------------------- Inertia units ----------------------------------------------------------

INERTIA = 'Inertia'
KILOGRAM_METER_SQUARED = 1.

ADIMENSIONNED = 'Adimensionned'


class UnitSystem:
    def __init__(self):
        self.dct = {
            LENGTH: np.array(1e-3),
            ANGLE: np.array(1.),
            TIME: np.array(1.),
            ACCELERATION: np.array(1.),
            FORCE: np.array(1.),
            MASS: np.array(1.),
            TORQUE: np.array(1.),
            SPRING_CONSTANT: np.array(1.),
            INERTIA: np.array(1.),
            ADIMENSIONNED: np.array(1)
        }

    def __getitem__(self, item):
        return self.dct[item]

    def set(self, phy, unit):
        self.dct[phy] = np.array(unit)


P1, P2 = ('p1', LENGTH, (0., 0.)), ('p2', LENGTH, (0., 0.))
A1, A2 = ('a1', ANGLE, 0.), ('a2', ANGLE, 0.)
D1, D2 = ('d1', LENGTH, 0.), ('d2', LENGTH, 0.)
ANGLE1_ = ('angle', ANGLE, 0.)

NORMAL, TANGENT = ('normal', FORCE, None), ('tangent', FORCE, None)
FORCE_, TORQUE_ = ('force', FORCE, None), ('torque', TORQUE, None)

SLIDING, ANGLE2_ = ('sliding', LENGTH, None), ('angle', ANGLE, None)
