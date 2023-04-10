import numpy as np

# every goat related data is from http://www.chevre-poitevine.org/le-standard-de-race/

#  ----------------------------------------------- Time units ----------------------------------------------------------

TIME = 'Time'
SECOND = np.array(1.), 's'
MILLISECOND = np.array(1e-3), 'ms'
MINUTE = np.array(60.), 'min'
_PINS = np.array(2.2), "pin's"

#  ------------------------------------------------ Length units -------------------------------------------------------

LENGTH = 'Length'
MILLIMETER = np.array(1e-3), 'mm'
METER = np.array(1.), 'm'
CENTIMETER = np.array(1e-2), 'cm'
INCH = np.array(2.54e-2), 'in'
GOAT_HEIGHT = np.array(.673), 'goats'
_ALLOUFS = np.array(6e-2), "allouf's"

# ----------------------------------------------- Speed units ---------------------------------------------------

SPEED = 'Speed'
METER_PER_SECOND = np.array(1.), 'm/s'
KILOMETRE_PER_HOUR = np.array(1/3.6), 'km/h'
GOAT_SPEED = np.array(16 / 3.6), 'goat speed'
_ALLOUFS_PER_PINS = np.array(2.73e-2), "allouf's/pin's"

# ----------------------------------------------- Acceleration units ---------------------------------------------------

ACCELERATION = 'Acceleration'
METER_PER_SECOND_SQUARED = np.array(1.), 'm/s²'
G = np.array(9.8067), 'G'
_ALLOUFS_PER_PINS_SQUARED = np.array(1.24e-2), "allouf's/pin's²"


# ------------------------------------------------ Angle units ---------------------------------------------------------

ANGLE = 'Angle'
RADIAN = np.array(1.), 'rad'
DEGREE = np.array(np.pi / 180), '°'

# --------------------------------------------- Angular velocity units -------------------------------------------------

ANGULAR_VELOCITY = 'Angular velocity'
RADIAN_PER_SECOND = np.array(1.), 'rad/s'
RPM = np.array(np.pi/30), 'rpm'
HERTZ = np.array(2*np.pi), 'Hz'

# ----------------------------------------- Angular velocity acceleration ----------------------------------------------

ANGULAR_ACCELERATION = 'Angular acceleration'
RADIAN_PER_SECOND_SQUARED = np.array(1.), 'rad/s²'

# --------------------------------------------- Mass units -------------------------------------------------------------

MASS = 'Mass'
KILOGRAM = np.array(1.), 'kg'
GRAM = np.array(1e-3), 'g'
POUND = np.array(2.20462), 'lb'
GOAT_MASS = np.array(60.), 'goats'
_CAISSEMS = np.array(50.), "caissem's"

# ---------------------------------------------- Force units -----------------------------------------------------------

FORCE = 'Force'
NEWTON = np.array(1.), 'N'
DECANEWTON = np.array(10.), 'daN'
MILLINEWTON = np.array(1e-3), 'mN'
KILONEWTON = np.array(1e3), 'kN'
KILOGRAM_FORCE = np.array(9.8067), 'kgf'

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
            TIME: SECOND,
            LENGTH: MILLIMETER,
            SPEED: METER_PER_SECOND,
            ACCELERATION: METER_PER_SECOND_SQUARED,
            ANGLE: RADIAN,
            ANGULAR_VELOCITY: RADIAN_PER_SECOND,
            ANGULAR_ACCELERATION: RADIAN_PER_SECOND_SQUARED,
            MASS: KILOGRAM,
            FORCE: NEWTON,
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
        return '\n'.join('{phy:20} : {unit}'.format(phy=phy, unit=unit) for phy, (value, unit) in self.dct.items())


PHYSICAL_QUANTITIES = (
    LENGTH, SPEED, ACCELERATION, TIME, ANGLE, ANGULAR_VELOCITY, ANGULAR_ACCELERATION, FORCE, MASS, TORQUE,
    SPRING_CONSTANT, INERTIA, DIMENSIONLESS
)
SYSTEM = UnitSystem()

show_units = lambda : print(SYSTEM)    

def set_unit(phy, value, unit='Unnamed unit'):
    if isinstance(value, tuple):
        value, unit = value
    SYSTEM.set(phy, value, unit)

def set_unit_system(unit_system='Default'):
    """
    ['Default', 'SI', 'American', 'Goat']
    """ # CGS
    if unit_system == 'Default':
        SYSTEM.dct = {TIME: SECOND,
                      LENGTH: MILLIMETER,
                      SPEED: METER_PER_SECOND,
                      ACCELERATION: METER_PER_SECOND_SQUARED,
                      ANGLE: RADIAN,
                      ANGULAR_VELOCITY: RADIAN_PER_SECOND,
                      ANGULAR_ACCELERATION: RADIAN_PER_SECOND_SQUARED,
                      MASS: KILOGRAM,
                      FORCE: NEWTON,
                      TORQUE: NEWTON_METER,
                      SPRING_CONSTANT: NEWTON_PER_METER,
                      INERTIA: KILOGRAM_METER_SQUARED,
                      DIMENSIONLESS: NO_UNIT}
    elif unit_system == 'SI':
        SYSTEM.dct = {TIME: SECOND,
                      LENGTH: METER,
                      SPEED: METER_PER_SECOND,
                      ACCELERATION: METER_PER_SECOND_SQUARED,
                      ANGLE: RADIAN,
                      ANGULAR_VELOCITY: RADIAN_PER_SECOND,
                      ANGULAR_ACCELERATION: RADIAN_PER_SECOND_SQUARED,
                      MASS: KILOGRAM,
                      FORCE: NEWTON,
                      TORQUE: NEWTON_METER,
                      SPRING_CONSTANT: NEWTON_PER_METER,
                      INERTIA: KILOGRAM_METER_SQUARED,
                      DIMENSIONLESS: NO_UNIT}
    elif unit_system == 'American':
        SYSTEM.dct = {TIME: SECOND,
                      LENGTH: METER,
                      SPEED: METER_PER_SECOND,
                      ACCELERATION: METER_PER_SECOND_SQUARED,
                      ANGLE: RADIAN,
                      ANGULAR_VELOCITY: RADIAN_PER_SECOND,
                      ANGULAR_ACCELERATION: RADIAN_PER_SECOND_SQUARED,
                      MASS: KILOGRAM,
                      FORCE: NEWTON,
                      TORQUE: NEWTON_METER,
                      SPRING_CONSTANT: NEWTON_PER_METER,
                      INERTIA: KILOGRAM_METER_SQUARED,
                      DIMENSIONLESS: NO_UNIT}
    elif unit_system == 'Goat':
        SYSTEM.dct = {TIME: None,
                      LENGTH: GOAT_HEIGHT,
                      SPEED: GOAT_SPEED,
                      ACCELERATION: None,
                      ANGLE: None,
                      ANGULAR_VELOCITY: None,
                      ANGULAR_ACCELERATION: None,
                      MASS: GOAT_MASS,
                      FORCE: None,
                      TORQUE: None,
                      SPRING_CONSTANT: None,
                      INERTIA: None,
                      DIMENSIONLESS: None}        
    elif unit_system == 'CGS':
        SYSTEM.dct = {TIME: _PINS,
                      LENGTH: _ALLOUFS,
                      SPEED: _ALLOUFS_PER_PINS,
                      ACCELERATION: _ALLOUFS_PER_PINS_SQUARED,
                      ANGLE: RADIAN,
                      ANGULAR_VELOCITY: RADIAN_PER_SECOND,
                      ANGULAR_ACCELERATION: RADIAN_PER_SECOND_SQUARED,
                      MASS: _CAISSEMS,
                      FORCE: NEWTON,
                      TORQUE: NEWTON_METER,
                      SPRING_CONSTANT: NEWTON_PER_METER,
                      INERTIA: KILOGRAM_METER_SQUARED,
                      DIMENSIONLESS: NO_UNIT}

set_unit.__doc__ = f"""Changes the unit
phy is the physical quantity among {', '.join(PHYSICAL_QUANTITIES)}
value is hom much of the SI unit your unit is: ex. 1 mm is 0.001 m so value is 0.001
name is the name of your unit.
You can use units imported from units.py"""
