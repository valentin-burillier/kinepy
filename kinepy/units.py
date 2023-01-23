import numpy as np

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

