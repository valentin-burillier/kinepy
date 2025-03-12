import types
import typing
import numpy as np
import enum
from functools import wraps
from dataclasses import dataclass


PhysicalQuantity = typing._AnnotatedAlias


class _PhysicsEnum(enum.Enum):
    LENGTH, MASS, MOMENT_OF_INERTIA, ANGLE, DIMENSIONLESS, TIME, VELOCITY, ACCELERATION, ANGULAR_VELOCITY, ANGULAR_ACCELERATION, DENSITY, FORCE, TORQUE, SPRING_CONSTANT = range(14)


scalar_type = float | np.ndarray
point_type = tuple[float, float] | list[float, float] | np.ndarray


@dataclass(frozen=True)
class Unit:
    _physics: _PhysicsEnum
    fullname: str
    symbol: str
    value: np.ndarray


class UnitSet:
    _PHYSICS: _PhysicsEnum
    phy: PhysicalQuantity
    SI_UNIT: Unit

    @classmethod
    def new_unit(cls, fullname: str, symbol: str, value: scalar_type) -> Unit:
        return Unit(cls._PHYSICS, fullname, symbol, np.ndarray(value))


class Length(UnitSet):
    # region auto-fill LENGTH
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.LENGTH
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill LENGTH
    point: PhysicalQuantity = typing.Annotated[point_type, _PHYSICS]

    METRE: Unit = Unit(_PHYSICS, 'metre', 'm', np.array(1.0))
    MILLIMETRE: Unit = Unit(_PHYSICS, 'millimetre', 'mm', np.array(1e-3))
    CENTIMETRE: Unit = Unit(_PHYSICS, 'centimetre', 'cm', np.array(1e-2))
    INCH: Unit = Unit(_PHYSICS, 'inch', 'in', np.array(2.54e-2))
    FOOT: Unit = Unit(_PHYSICS, 'foot', 'ft', np.array(0.3048))
    YARD: Unit = Unit(_PHYSICS, 'yard', 'yd', np.array(0.9144))
    GOAT: Unit = Unit(_PHYSICS, 'goat height', 'gt', np.array(.673))

    SI_UNIT = METRE


class Mass(UnitSet):
    # region auto-fill MASS
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.MASS
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill MASS

    KILOGRAM: Unit = Unit(_PHYSICS, 'kilograms', 'kg', np.array(1.))
    GRAM: Unit = Unit(_PHYSICS, 'grams', 'g', np.array(1e-3))
    POUND: Unit = Unit(_PHYSICS, 'pounds', 'lb', np.array(0.453592))
    GOAT: Unit = Unit(_PHYSICS, 'goats', 'gt', np.array(60.))

    SI_UNIT = KILOGRAM


class MomentOfInertia(UnitSet):
    # region auto-fill MOMENT_OF_INERTIA
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.MOMENT_OF_INERTIA
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill MOMENT_OF_INERTIA

    KILOGRAM_METRE_SQUARED: Unit = Unit(_PHYSICS, 'kilogram metre squared', 'kg.m²', np.array(1.))
    POUND_FORCE_FOOT_SECOND_SQUARED: Unit = Unit(_PHYSICS, 'pound-force foot second squared', 'lbf.ft.s2', np.array(1.3423))

    SI_UNIT = KILOGRAM_METRE_SQUARED


class Angle(UnitSet):
    # region auto-fill ANGLE
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.ANGLE
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill ANGLE

    RADIAN: Unit = Unit(_PHYSICS, 'radians', 'rad', np.array(1.))
    DEGREE: Unit = Unit(_PHYSICS, 'degrees', '°', np.array(np.pi / 180))
    REVOLUTION: Unit = Unit(_PHYSICS, 'revolutions', 'r', np.array(2*np.pi))

    SI_UNIT = RADIAN


class Dimensionless(UnitSet):
    # region auto-fill DIMENSIONLESS
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.DIMENSIONLESS
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill DIMENSIONLESS

    NO_UNIT: Unit = Unit(_PHYSICS, 'no unit', '', np.array(1.))
    PERCENT: Unit = Unit(_PHYSICS, 'percents', '%', np.array(.01))
    GOAT: Unit = Unit(_PHYSICS, 'goats', 'gt', np.array(1.))  # simply count your goats

    SI_UNIT = NO_UNIT


class Time(UnitSet):
    # region auto-fill TIME
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.TIME
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill TIME

    SECOND: Unit = Unit(_PHYSICS, 'seconds', 's', np.array(1.))
    MILLISECOND: Unit = Unit(_PHYSICS, 'milliseconds', 'ms', np.array(1e-3))
    MINUTE: Unit = Unit(_PHYSICS, 'minutes', 'min', np.array(60.))
    BLEAT: Unit = Unit(_PHYSICS, 'goat bleats', 'baa', np.array(1.534))  # measured one bleat that took this long
    SI_UNIT = SECOND


class Velocity(UnitSet):
    # region auto-fill VELOCITY
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.VELOCITY
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]
    point: PhysicalQuantity = typing.Annotated[point_type, _PHYSICS]

    # endregion auto-fill VELOCITY

    METRE_PER_SECOND: Unit = Unit(_PHYSICS, 'metres per second', 'm/s', np.array(1.))
    MILLIMETRE_PER_SECOND: Unit = Unit(_PHYSICS, 'millimetres per second', 'mm/s', np.array(1e-3))
    KILOMETRE_PER_HOUR: Unit = Unit(_PHYSICS, 'kilometres per hour', 'km/h', np.array(1/3.6))
    FOOT_PER_SECOND: Unit = Unit(_PHYSICS, 'feet per second', 'ft/s', np.array(0.3048))
    MILE_PER_HOUR: Unit = Unit(_PHYSICS, 'miles per hour', 'mph', np.array(0.44704))
    GOAT: Unit = Unit(_PHYSICS, 'goat speed', 'gt', np.array(16 / 3.6))

    SI_UNIT = METRE_PER_SECOND


class Acceleration(UnitSet):
    # region auto-fill ACCELERATION
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.ACCELERATION
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]
    point: PhysicalQuantity = typing.Annotated[point_type, _PHYSICS]

    # endregion auto-fill ACCELERATION

    METRE_PER_SECOND_SQUARED: Unit = Unit(_PHYSICS, 'metres per second squared', 'm/s²', np.array(1.))
    MILLIMETRE_PER_SECOND_SQUARED: Unit = Unit(_PHYSICS, 'millimetres per second squared', 'mm/s²', np.array(1e-3))
    FOOT_PER_SECOND_SQUARED: Unit = Unit(_PHYSICS, 'feet per second squared', 'ft/s²', np.array(0.3048))
    G: Unit = Unit(_PHYSICS, 'earth gravitation', 'G', np.array(9.8067))

    SI_UNIT = METRE_PER_SECOND_SQUARED


class AngularVelocity(UnitSet):
    # region auto-fill ANGULAR_VELOCITY
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.ANGULAR_VELOCITY
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill ANGULAR_VELOCITY

    RADIAN_PER_SECOND: Unit = Unit(_PHYSICS, 'radians per second', 'rad/s', np.array(1.))
    REVOLUTION_PER_MINUTE: Unit = Unit(_PHYSICS, 'revolutions pêr minute', 'rpm', np.array(np.pi/30))
    HERTZ: Unit = Unit(_PHYSICS, 'hertz', 'Hz', np.array(2*np.pi))
    DEGREE_PER_SECOND: Unit = Unit(_PHYSICS, 'degrees per second', '°/s', np.array(np.pi / 180))

    SI_UNIT = RADIAN_PER_SECOND


class AngularAcceleration(UnitSet):
    # region auto-fill ANGULAR_ACCELERATION
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.ANGULAR_ACCELERATION
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill ANGULAR_ACCELERATION

    RADIAN_PER_SECOND_SQUARED: Unit = Unit(_PHYSICS, 'radians per second squared', 'rad/s²', np.array(1.))
    DEGREE_PER_SECOND_SQUARED: Unit = Unit(_PHYSICS, 'degrees per second squared', '°/s²', np.array(np.pi / 180))

    SI_UNIT = RADIAN_PER_SECOND_SQUARED


class Density(UnitSet):
    # region auto-fill DENSITY
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.DENSITY
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill DENSITY

    KILOGRAM_PER_CUBIC_METRE: Unit = Unit(_PHYSICS, 'kilogram per cubic metre', 'kg/m³', np.array(1.))
    GRAM_PER_CUBIC_CENTIMETRE: Unit = Unit(_PHYSICS, 'gram per cubic centimetre', 'g/cm³', np.array(1e3))
    POUND_PER_CUBIC_INCH: Unit = Unit(_PHYSICS, 'pound per cubic inch', 'lb/in³', np.array(27679.9))
    POUND_PER_CUBIC_FOOT: Unit = Unit(_PHYSICS, 'pound per cubic foot', 'lb/ft³', np.array(16.0185))

    SI_UNIT = KILOGRAM_PER_CUBIC_METRE


class Force(UnitSet):
    # region auto-fill FORCE
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.FORCE
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]
    point: PhysicalQuantity = typing.Annotated[point_type, _PHYSICS]

    # endregion auto-fill FORCE

    NEWTON: Unit = Unit(_PHYSICS, 'newton', 'N', np.array(1.))
    DECANEWTON: Unit = Unit(_PHYSICS, 'deca-newton', 'daN', np.array(10.))
    MILLINEWTON: Unit = Unit(_PHYSICS, 'millinewton', 'mN', np.array(1e-3))
    KILONEWTON: Unit = Unit(_PHYSICS, 'kilonewton', 'kN', np.array(1e3))
    POUND_FORCE: Unit = Unit(_PHYSICS, 'pound force', 'lbf', np.array(4.44822))
    KILOGRAM_FORCE: Unit = Unit(_PHYSICS, 'kilogram force', 'kgf', np.array(9.8067))

    SI_UNIT = NEWTON


class Torque(UnitSet):
    # region auto-fill TORQUE
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.TORQUE
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill TORQUE

    NEWTON_METRE: Unit = Unit(_PHYSICS, 'newton metre', 'N.m', np.array(1.))
    NEWTON_MILLIMETRE: Unit = Unit(_PHYSICS, 'newton millimetre', 'N.mm', np.array(1e-3))
    MILLINEWTON_METRE: Unit = Unit(_PHYSICS, 'millinewton metre', 'mN.m', np.array(1e-3))
    POUND_FOOT: Unit = Unit(_PHYSICS, 'pound foot', 'lb.ft', np.array(1.355818))

    SI_UNIT = NEWTON_METRE


class SpringConstant(UnitSet):
    # region auto-fill SPRING_CONSTANT
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.SPRING_CONSTANT
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill SPRING_CONSTANT

    NEWTON_PER_METRE: Unit = Unit(_PHYSICS, 'newton per metre', 'N/m', np.array(1.))
    NEWTON_PER_MILLIMETRE: Unit = Unit(_PHYSICS, 'newton per millimetre', 'N/mm', np.array(1e3))
    POUND_FORCE_PER_INCH: Unit = Unit(_PHYSICS, 'pound force per inch', 'lbf/in', np.array(0.112985))

    SI_UNIT = NEWTON_PER_METRE

# this comment is a separator for automated formating


SI_UNITS = tuple(cls.SI_UNIT for cls in UnitSet.__subclasses__())
DEFAULT_UNITS = SI_UNITS + (
    Length.MILLIMETRE,
)

IMPERIAL_UNITS = (
    Time.SECOND,
    Length.INCH,
    Velocity.FOOT_PER_SECOND,
    Acceleration.FOOT_PER_SECOND_SQUARED,
    Angle.RADIAN,
    AngularVelocity.RADIAN_PER_SECOND,
    AngularAcceleration.RADIAN_PER_SECOND_SQUARED,
    Density.POUND_PER_CUBIC_INCH,
    Mass.POUND,
    Force.POUND_FORCE,
    Torque.POUND_FOOT,
    SpringConstant.POUND_FORCE_PER_INCH,
    MomentOfInertia.POUND_FORCE_FOOT_SECOND_SQUARED,
    Dimensionless.NO_UNIT
)

GOAT_UNITS = SI_UNITS + (
    Time.BLEAT,
    Velocity.GOAT,
    Dimensionless.GOAT,
    Mass.GOAT,
    Length.GOAT
)


def _identity(x):
    return x


def screaming_snake_to_pascal(name: str):
    return ''.join(map(str.capitalize, name.split('_')))


def screaming_snake_to_words(name: str):
    return ' '.join(name.split('_')).capitalize()


class UnitSystem:
    _unit_values: dict[_PhysicsEnum, Unit] = {}

    @classmethod
    def _get_unit_value(cls, phy: _PhysicsEnum) -> np.ndarray:
        return cls._unit_values[phy].value

    @classmethod
    def use(cls, *units: Unit, kw_units: tuple[Unit, ...] = ()) -> None:
        """
        Select units to use for every unit-dependant interface. Duplicates are overwritten, last matters.
        """
        for unit in units + kw_units:
            cls._unit_values[unit._physics] = unit

    @classmethod
    def show(cls) -> str:
        phys, full_names, symbols, values = zip(*cls._unit_values.values())

        mn = len(max(max(phys, key=lambda x: len(x.name)).name, 'Physical quantity', key=len))
        mf = len(max(max(full_names, key=len), 'Unit name', key=len))
        ms = len(max(max(symbols, key=len), 'Symbol', key=len))

        titles = f'\n{'Physical quantity':<{mn}} | {'Unit name':<{mf}} | {'Symbol':<{ms}} | 1 unit in SI unit\n'
        table = '\n'.join(f'{screaming_snake_to_words(phy.name):<{mn}} | {full_name:<{mf}} | {symbol:>{ms}} | {value:>12.3e} {globals().get(screaming_snake_to_pascal(phy.name), Dimensionless).SI_UNIT[2]}' for phy, full_name, symbol, value in cls._unit_values.values())
        return titles + table

    @classmethod
    def get_unit(cls, unit_set: type[UnitSet]):
        return cls._unit_values[unit_set._PHYSICS]

    @classmethod
    def _input(cls, phy: _PhysicsEnum):
        return lambda value: value * cls._get_unit_value(phy)

    @classmethod
    def _output(cls, phy: _PhysicsEnum):
        return lambda value: value / cls._get_unit_value(phy)

    @classmethod
    def function(cls, func: types.FunctionType) -> types.FunctionType:
        """
        Function decorator that manages all arguments annotated with a PhysicalQuantity and the return value
        """
        phy_annotations = {
            name: phy for name, annotation in func.__annotations__.items() if (phy := cls._physical_quantity_annotation(annotation)) is not None
        }
        if not phy_annotations:
            return func

        arguments = func.__code__.co_varnames[:func.__code__.co_argcount + func.__code__.co_kwonlyargcount]

        # all parameters that are annotated with a physical quantity including *args
        named_transforms = {
            name: cls._input(phy) for name, phy in phy_annotations.items() if name != 'return'
        }
        # *args
        variadic_transform = named_transforms.pop(func.__code__.co_varnames[len(arguments)], _identity) if len(arguments) < len(func.__code__.co_varnames) else _identity
        return_transform = cls._output(phy) if (phy := phy_annotations.get('return', None)) is not None else _identity

        @wraps(func)
        def new_function(*args, **kwargs):
            new_args = (named_transforms.get(name, _identity)(value) for name, value in zip(arguments, args))
            vari_args = (variadic_transform(value) for value in args[len(arguments):])
            arg_dict = {name: named_transforms.get(name, _identity)(value) for name, value in kwargs.items()}
            return return_transform(func(*new_args, *vari_args, **arg_dict))

        return new_function

    @classmethod
    def class_(cls, target_class: type) -> type:
        """
        Class decorator that manages all methods with the `Physics.function` decorator and creates properties to manage attributes annotated with a PhysicalQuantity
        """
        # Retrieve all method definitions
        for method_name, method in target_class.__dict__.items():
            if isinstance(method, property):
                fget = None if method.fget is None else cls.function(method.fget)
                fset = None if method.fset is None else cls.function(method.fset)
                setattr(target_class, method_name, property(fget, fset))
                continue
            if not isinstance(method, types.FunctionType):
                continue
            setattr(target_class, method_name, cls.function(method))

        # Retrieve all attributes that are annotated with physical quantities to place getters and setters on them
        for attr, annotation in target_class.__annotations__.items():
            if attr in target_class.__dict__ or (phy := cls._physical_quantity_annotation(annotation)) is None:
                continue
            setattr(target_class, attr, property(
                lambda self: getattr(self, f'_{attr}') / cls._get_unit_value(phy),
                lambda self, value: setattr(self, f'_{attr}', value * cls._get_unit_value(phy))
            ))
        return target_class

    @staticmethod
    def _physical_quantity_annotation(annotation: typing.Any) -> None | _PhysicsEnum:
        if isinstance(annotation, PhysicalQuantity) and isinstance(annotation.__metadata__[0], _PhysicsEnum):
            return annotation.__metadata__[0]
        return None


UnitSystem.use(kw_units=DEFAULT_UNITS)


if __name__ == '__main__':
    """
    Automatically create new UnitSet inheritors when a new _PhysicsEnum is created
    """
    with open(__file__, 'r') as _this:
        _whole_file = _this.read()

    # exclude this part
    _separator = "# this comment is a separator for automated formating\n"
    _module, _rest = _whole_file.split(_separator, maxsplit=1)
    _new_class_names = tuple(phy.name for phy in _PhysicsEnum if globals().get(screaming_snake_to_pascal(phy.name), None) is None)
    _template = """class {Name}(UnitSet):
    # region auto-fill {NAME}
    
    _PHYSICS: _PhysicsEnum = _PhysicsEnum.{NAME}
    phy: PhysicalQuantity = typing.Annotated[scalar_type, _PHYSICS]

    # endregion auto-fill {NAME}


    """

    _new_classes = ''.join(_template.format(NAME=name, Name=screaming_snake_to_pascal(name)) for name in _new_class_names)
    _new_file = _module + _new_classes + _separator + _rest

    with open(__file__, 'w') as this:
        this.write(_new_file)
