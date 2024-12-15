import numpy as np

# every goat related data is from http://www.chevre-poitevine.org/le-standard-de-race/

#  ----------------------------------------------- Time units ----------------------------------------------------------

TIME = 'Time'
SECOND = np.array(1.), 's'
MILLISECOND = np.array(1e-3), 'ms'
MINUTE = np.array(60.), 'min'

#  ------------------------------------------------ Length units -------------------------------------------------------

LENGTH = 'Length'
MILLIMETER = np.array(1e-3), 'mm'
METER = np.array(1.), 'm'
CENTIMETER = np.array(1e-2), 'cm'
INCH = np.array(2.54e-2), 'in'
FOOT = np.array(0.3048), 'ft'
YARD = np.array(0.9144), 'yd'
GOAT_HEIGHT = np.array(.673), 'goats'

# ----------------------------------------------- Speed units ---------------------------------------------------

SPEED = 'Speed'
METER_PER_SECOND = np.array(1.), 'm/s'
MILLIMETER_PER_SECOND = np.array(1e-3), 'mm/s'
KILOMETRE_PER_HOUR = np.array(1/3.6), 'km/h'
FOOT_PER_SECOND = np.array(0.3048), 'ft/s'
MILES_PER_HOUR = np.array(0.44704), 'mph'
GOAT_SPEED = np.array(16 / 3.6), 'goat speed'

# ----------------------------------------------- Acceleration units ---------------------------------------------------

ACCELERATION = 'Acceleration'
METER_PER_SECOND_SQUARED = np.array(1.), 'm/s²'
MILLIMETER_PER_SECOND_SQUARED = np.array(1e-3), 'mm/s²'
FOOT_PER_SECOND_SQUARED = np.array(0.3048), 'ft/s²'
G = np.array(9.8067), 'G'


# ------------------------------------------------ Angle units ---------------------------------------------------------

ANGLE = 'Angle'
RADIAN = np.array(1.), 'rad'
DEGREE = np.array(np.pi / 180), '°'
REVOLUTION = np.array(2*np.pi), 'r'

# --------------------------------------------- Angular velocity units -------------------------------------------------

ANGULAR_VELOCITY = 'Angular velocity'
RADIAN_PER_SECOND = np.array(1.), 'rad/s'
REVOLUTION_PER_MINUTE = np.array(np.pi/30), 'rpm'
HERTZ = np.array(2*np.pi), 'Hz'
DEGREE_PER_SECOND = np.array(np.pi / 180), '°/s'


# ----------------------------------------- Angular acceleration units -------------------------------------------------

ANGULAR_ACCELERATION = 'Angular acceleration'
RADIAN_PER_SECOND_SQUARED = np.array(1.), 'rad/s²'
DEGREE_PER_SECOND_SQUARED = np.array(np.pi / 180), '°/s²'

# --------------------------------------------- Mass units -------------------------------------------------------------

MASS = 'Mass'
KILOGRAM = np.array(1.), 'kg'
GRAM = np.array(1e-3), 'g'
POUND = np.array(0.453592), 'lb'
GOAT_MASS = np.array(60.), 'goats'

# --------------------------------------------- Density units ----------------------------------------------------------

DENSITY = 'Density'
KILOGRAM_PER_CUBIC_METER = np.array(1.), 'kg/m³'
GRAM_PER_CUBIC_CENTIMETER = np.array(1e3), 'g/cm³'
POUND_PER_CUBIC_INCH = np.array(27679.9), 'lb/in³'
POUND_PER_CUBIC_FOOT = np.array(16.0185), 'lb/ft³'

# ---------------------------------------------- Force units -----------------------------------------------------------

FORCE = 'Force'
NEWTON = np.array(1.), 'N'
DECANEWTON = np.array(10.), 'daN'
MILLINEWTON = np.array(1e-3), 'mN'
KILONEWTON = np.array(1e3), 'kN'
POUND_FORCE = np.array(4.44822), 'lbf'
KILOGRAM_FORCE = np.array(9.8067), 'kgf'

# -------------------------------------------- Torque units ------------------------------------------------------------

TORQUE = 'Torque'
NEWTON_METER = np.array(1.), 'N.m'
NEWTON_MILLIMETER = np.array(1e-3), 'N.mm'
MILLINEWTON_METER = np.array(1e-3), 'mN.m'
POUND_FOOT = np.array(1.355818), 'lb.ft'

# -------------------------------------------- Spring constant units ---------------------------------------------------

SPRING_CONSTANT = 'SpringConstant'
NEWTON_PER_METER = np.array(1.), 'N/m'
NEWTON_PER_MILLIMETER = np.array(1e3), 'N/mm'
POUND_FORCE_PER_INCH = np.array(0.112985), 'lbf/in'

# --------------------------------------------- Inertia units ----------------------------------------------------------

INERTIA = 'Inertia'
KILOGRAM_METER_SQUARED = np.array(1.), 'kg.m²'
POUND_FOOT_SECOND_SQUARED = np.array(1.3423), 'lbf.ft.s2'

# -------------------------------------------- dimensionless units -----------------------------------------------------

DIMENSIONLESS = 'dimensionless'
NO_UNIT = np.array(1.), 'No Unit'
PERCENT = np.array(.01), '%'


# ----------------------------------------------- Variable units -------------------------------------------------------

VARIABLE_UNIT = 'X'
VARIABLE_DERIVATIVE = 'dX/dt'
VARIABLE_SECOND_DERIVATIVE = 'd²X/dt²'

# ---------------------------------------------- Preset systems --------------------------------------------------------

DEFAULT_SYSTEM = {
    TIME: SECOND,
    LENGTH: MILLIMETER,
    SPEED: METER_PER_SECOND,
    ACCELERATION: METER_PER_SECOND_SQUARED,
    ANGLE: RADIAN,
    ANGULAR_VELOCITY: RADIAN_PER_SECOND,
    ANGULAR_ACCELERATION: RADIAN_PER_SECOND_SQUARED,
    DENSITY: GRAM_PER_CUBIC_CENTIMETER,
    MASS: KILOGRAM,
    FORCE: NEWTON,
    TORQUE: NEWTON_METER,
    SPRING_CONSTANT: NEWTON_PER_METER,
    INERTIA: KILOGRAM_METER_SQUARED,
    DIMENSIONLESS: NO_UNIT
}

SI = {
    TIME: SECOND,
    LENGTH: METER,
    SPEED: METER_PER_SECOND,
    ACCELERATION: METER_PER_SECOND_SQUARED,
    ANGLE: RADIAN,
    ANGULAR_VELOCITY: RADIAN_PER_SECOND,
    ANGULAR_ACCELERATION: RADIAN_PER_SECOND_SQUARED,
    DENSITY: KILOGRAM_PER_CUBIC_METER,
    MASS: KILOGRAM,
    FORCE: NEWTON,
    TORQUE: NEWTON_METER,
    SPRING_CONSTANT: NEWTON_PER_METER,
    INERTIA: KILOGRAM_METER_SQUARED,
    DIMENSIONLESS: NO_UNIT
}

AMERICAN_SYSTEM = {
    TIME: SECOND,
    LENGTH: INCH,
    SPEED: FOOT_PER_SECOND,
    ACCELERATION: FOOT_PER_SECOND_SQUARED,
    ANGLE: RADIAN,
    ANGULAR_VELOCITY: RADIAN_PER_SECOND,
    ANGULAR_ACCELERATION: RADIAN_PER_SECOND_SQUARED,
    DENSITY: POUND_PER_CUBIC_INCH,
    MASS: POUND,
    FORCE: POUND_FORCE,
    TORQUE: POUND_FOOT,
    SPRING_CONSTANT: POUND_FORCE_PER_INCH,
    INERTIA: POUND_FOOT_SECOND_SQUARED,
    DIMENSIONLESS: NO_UNIT
}

GOAT_SYSTEM = {
    LENGTH: GOAT_HEIGHT,
    SPEED: GOAT_SPEED,
    MASS: GOAT_MASS,
}

DERIVATIVES = {
    LENGTH: SPEED,
    SPEED: ACCELERATION,
    ANGLE: ANGULAR_VELOCITY,
    ANGULAR_VELOCITY: ANGULAR_ACCELERATION
}


class UnitSystem:
    def __init__(self):
        self.dct = dict(DEFAULT_SYSTEM)

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


def show_units():
    print(SYSTEM)


def set_unit(phy, value, unit='Unnamed unit'):
    if isinstance(value, tuple):
        value, unit = value
    SYSTEM.set(phy, value, unit)
    

set_unit.__doc__ = f"""Changes the unit
phy is the physical quantity among {', '.join(PHYSICAL_QUANTITIES)}
value is hom much of the SI unit your unit is: ex. 1 mm is 0.001 m so value is 0.001
name is the name of your unit.
You can use units imported from units.py"""


def get_unit(phy):
    return SYSTEM.dct[phy][1]


def get_value(phy):
    return SYSTEM.dct[phy][0]


def set_unit_system(unit_system: dict):
    """
    Replaces every unit of the global unit system with the units described in unit_system.
    Missing units will be unchanged.
    
    Available systems:
    DEFAULT_SYSTEM, SI, AMERICAN_SYSTEM, GOAT_SYSTEM 
    """
    for key in SYSTEM.dct:
        SYSTEM.dct[key] = unit_system.get(key, SYSTEM[key])


import types
from typing import Annotated
import numpy as np
import enum
from functools import wraps


class _PhysicsEnum(enum.Enum):
    LENGTH, MASS, MOMENT_OF_INERTIA, ANGLE = range(4)


class _IdEnum(enum.Enum):
    SOLID, JOINT, RELATION = range(3)


class Physics:
    _scalar_type = float | np.ndarray
    LENGTH = Annotated[_scalar_type, _PhysicsEnum.LENGTH]
    MASS = Annotated[_scalar_type, _PhysicsEnum.MASS]
    MOMENT_OF_INERTIA = Annotated[_scalar_type, _PhysicsEnum.MOMENT_OF_INERTIA]
    ANGLE = Annotated[_scalar_type, _PhysicsEnum.ANGLE]

    POINT = Annotated[tuple[float, float] | list[float, float] | np.ndarray, _PhysicsEnum.LENGTH]

    _unit_values: dict[_PhysicsEnum, tuple[np.ndarray, str]] = {
        _PhysicsEnum.LENGTH: (np.array(1.0), 'm'),
        _PhysicsEnum.MASS: (np.array(1.0), 'kg'),
        _PhysicsEnum.MOMENT_OF_INERTIA: (np.array(1.0), 'kg.m²'),
        _PhysicsEnum.ANGLE: (np.array(1.0), 'rad')
    }

    @classmethod
    def get_unit_value(cls, phy: _PhysicsEnum) -> np.ndarray:
        return cls._unit_values[phy][0]

    @classmethod
    def set_unit(cls, phy: _PhysicsEnum, value: _scalar_type, symbol: str) -> None:
        cls._unit_values[phy] = np.array(value), symbol


def kinepy_input(physics: _PhysicsEnum):
    return lambda value: value * Physics.get_unit_value(physics)


def identity(x):
    return x


def kinepy_output(physics):
    if not hasattr(physics, '__metadata__'):
        return identity
    physics = physics.__metadata__[0]
    return lambda value: value / Physics.get_unit_value(physics)


def kinepy_function(func: types.FunctionType):
    arguments = func.__code__.co_varnames[:func.__code__.co_argcount]
    defaults = dict(zip(arguments[::-1], (func.__defaults__ or ())[::-1]))

    physical_transforms = {
        name: kinepy_input(annotation.__metadata__[0])
        for name, annotation in func.__annotations__.items() if hasattr(annotation, '__metadata__') and isinstance(annotation.__metadata__[0], _PhysicsEnum) and name != 'return'
    }

    return_transform = kinepy_output(func.__annotations__.get('return', None))

    @wraps(func)
    def new_function(*args, **kwargs):
        arg_dict = kwargs | dict(zip(arguments, args))
        arg_dict = {name: physical_transforms.get(name, identity)(value) for name, value in arg_dict.items()}
        return return_transform(func(**(defaults | arg_dict)))

    return new_function


def kinepy_class(cls: type):
    # Retrieve all method definitions
    for method_name, method in cls.__dict__.items():
        if method_name != '__init__' and str.startswith(method_name, '__'):
            continue
        setattr(cls, method_name, kinepy_function(method))

    # Retrieve all attributes that are annotated with physical quantities to place getters and setters
    for attr, unit in cls.__annotations__.items():
        setattr(cls, attr, property(
            lambda self: getattr(self, f'_{attr}'),
            lambda self, value: setattr(self, f'_{attr}', value)
        ))
    return cls
