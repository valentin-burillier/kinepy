import numpy as np

#  ------------------------------------------------ Length units -------------------------------------------------------

LENGTH = 'Length'
MILLIMETER = np.array(1e-3), 'mm'
METER = np.array(1.), 'm'
CENTIMETER = np.array(1e-2), 'cm'
INCH = np.array(2.54e-2), 'in'

#  ----------------------------------------------- Time units ----------------------------------------------------------

TIME = 'Time'
SECOND = np.array(1.), 's'
MILLISECOND = np.array(1e-3), 'ms'
MINUTE = np.array(60.), 'min'

# ------------------------------------------------ Angle units ---------------------------------------------------------

ANGLE = 'Angle'
RADIAN = np.array(1.), 'rad'
DEGREE = np.array(np.pi / 180), '°'

# ----------------------------------------------- Acceleration units ---------------------------------------------------

ACCELERATION = 'Acceleration'
METER_PER_SQUARE_SECOND = np.array(1.), 'm/s²'
G = np.array(9.8067), 'G'

# ----------------------------------------------- Speed units ---------------------------------------------------

SPEED = 'Speed'
METER_PER_SECOND = np.array(1.), 'm/s'
KILOMETRE_PER_HOUR = np.array(1/3.6), 'km/h'

# ---------------------------------------------- Force units -----------------------------------------------------------

FORCE = 'Force'
NEWTON = np.array(1.), 'N'
DECANEWTON = np.array(10.), 'daN'
MILLINEWTON = np.array(1e-3), 'mN'
KILONEWTON = np.array(1e3), 'kN'

# --------------------------------------------- Mass units -------------------------------------------------------------

MASS = 'Mass'
KILOGRAM = np.array(1.), 'kg'
GRAM = np.array(1e-3), 'g'
POUND = np.array(2.20462), 'lb'

# -------------------------------------------- Torque units ------------------------------------------------------------

TORQUE = 'Torque'
NEWTON_METER = np.array(1.), 'Nm'
MILLINEWTON_METER = np.array(1e-3), 'mNm'

# -------------------------------------------- Spring constant units ---------------------------------------------------

SPRING_CONSTANT = 'SpringConstant'
NEWTON_PER_METER = np.array(1.), 'N/m'

# --------------------------------------------- Inertia units ----------------------------------------------------------

INERTIA = 'Inertia'
KILOGRAM_METER_SQUARED = np.array(1.), 'kg.m²'

# --------------------------------------------- Adimensionned ----------------------------------------------------------

ADIMENSIONNED = 'Adimensionned'
NO_UNIT = np.array(1.), 'No Unit'
PERCENT = np.array(.01), '%'


class UnitSystem:
    def __init__(self):
        self.dct = {
            LENGTH: MILLIMETER,
            ANGLE: RADIAN,
            TIME: SECOND,
            ACCELERATION: KILOGRAM_METER_SQUARED,
            SPEED: METER_PER_SECOND,
            FORCE: NEWTON,
            MASS: KILOGRAM,
            TORQUE: NEWTON_METER,
            SPRING_CONSTANT: NEWTON_PER_METER,
            INERTIA: KILOGRAM_METER_SQUARED,
            ADIMENSIONNED: NO_UNIT
        }

    def __getitem__(self, item):
        return self.dct[item][0]

    def set(self, phy, value, unit: str):
        self.dct[phy] = np.array(value), unit

    def __repr__(self):
        return '\n'.join('{phy:14} : {unit}'.format(phy=phy, unit=unit) for phy, (value, unit) in self.dct.items())


PHYSICAL_QUANTITIES = LENGTH, TIME, ANGLE, ACCELERATION, FORCE, MASS, TORQUE, SPRING_CONSTANT, INERTIA, ADIMENSIONNED
