import numpy as np

# every goat related data is from http://www.chevre-poitevine.org/le-standard-de-race/


#  ------------------------------------------------ Length units -------------------------------------------------------

LENGTH = 'Length'
MILLIMETER = np.array(1e-3), 'mm'
METER = np.array(1.), 'm'
CENTIMETER = np.array(1e-2), 'cm'
INCH = np.array(2.54e-2), 'in'
GOAT_HEIGHT = np.array(.673), 'goats'

# ----------------------------------------------- Speed units ---------------------------------------------------

SPEED = 'Speed'
METER_PER_SECOND = np.array(1.), 'm/s'
KILOMETRE_PER_HOUR = np.array(1/3.6), 'km/h'
GOAT_AVG_SPEED = np.array(16 / 3.6), 'avg goat speed'
GOAT_MAX_SPEED = np.array(80 / 3.6), 'max goat speed'


# ----------------------------------------------- Acceleration units ---------------------------------------------------

ACCELERATION = 'Acceleration'
METER_PER_SQUARE_SECOND = np.array(1.), 'm/s²'
G = np.array(9.8067), 'G'


#  ----------------------------------------------- Time units ----------------------------------------------------------

TIME = 'Time'
SECOND = np.array(1.), 's'
MILLISECOND = np.array(1e-3), 'ms'
MINUTE = np.array(60.), 'min'

# ------------------------------------------------ Angle units ---------------------------------------------------------

ANGLE = 'Angle'
RADIAN = np.array(1.), 'rad'
DEGREE = np.array(np.pi / 180), '°'

# --------------------------------------------- Angular velocity units -------------------------------------------------

ANGULAR_VELOCITY = 'Angular velocity'
RADIAN_PER_SECOND = np.array(1.), 'rad/s'

# ----------------------------------------- Angular velocity acceleration ----------------------------------------------

ANGULAR_ACCELERATION = 'Angular acceleration'
RADIAN_PER_SQUARE_SECOND = np.array(1.), 'rad/s²'


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
GOAT_MASS = np.array(60.), 'goats'

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

# -------------------------------------------- dimensionless units -----------------------------------------------------

DIMENSIONLESS = 'dimensionless'
NO_UNIT = np.array(1.), 'No Unit'
PERCENT = np.array(.01), '%'


class UnitSystem:
    def __init__(self):
        self.dct = {
            LENGTH: MILLIMETER,
            SPEED: METER_PER_SECOND,
            ACCELERATION: METER_PER_SQUARE_SECOND,
            ANGLE: RADIAN,
            ANGULAR_VELOCITY: RADIAN_PER_SECOND,
            ANGULAR_ACCELERATION: RADIAN_PER_SQUARE_SECOND,
            TIME: SECOND,
            FORCE: NEWTON,
            MASS: KILOGRAM,
            TORQUE: NEWTON_METER,
            SPRING_CONSTANT: NEWTON_PER_METER,
            INERTIA: KILOGRAM_METER_SQUARED,
            DIMENSIONLESS: NO_UNIT
        }

    def __getitem__(self, item):
        return self.dct[item][0]

    def set(self, phy, value, unit: str):
        self.dct[phy] = np.array(value), unit

    def __repr__(self):
        return '\n'.join('{phy:14} : {unit}'.format(phy=phy, unit=unit) for phy, (value, unit) in self.dct.items())


PHYSICAL_QUANTITIES = (
    LENGTH, SPEED, ACCELERATION, TIME, ANGLE, ANGULAR_VELOCITY, ANGULAR_ACCELERATION, FORCE, MASS, TORQUE,
    SPRING_CONSTANT, INERTIA, DIMENSIONLESS
)
SYSTEM = UnitSystem()


def set_unit(phy, value, unit='Unnamed unit'):
    if isinstance(value, tuple):
        value, unit = value
    SYSTEM.set(phy, value, unit)


set_unit.__doc__ = f"""Changes the unit
phy is the physical quantity among {', '.join(PHYSICAL_QUANTITIES)}
value is hom much of the SI unit your unit is: ex. 1 mm is 0.001 m so value is 0.001
name is the name of your unit.
You can use units imported from units.py"""


def show_units():
    print("Currently used units:", *(f'{key}: {name}' for key, (_, name) in SYSTEM.dct.items()), sep='\n')
