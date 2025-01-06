import types
import typing
import numpy as np
import enum
from functools import wraps


PhysicalQuantity = typing._AnnotatedAlias


class _PhysicsEnum(enum.Enum):
    LENGTH, MASS, MOMENT_OF_INERTIA, ANGLE, DIMENSIONLESS = range(5)


class Physics:
    scalar_type = float | np.ndarray
    POINT: PhysicalQuantity = typing.Annotated[tuple[float, float] | list[float, float] | np.ndarray, _PhysicsEnum.LENGTH]

    # Just run this file for this region to be updated
    # region PhysicsTypes

    LENGTH: PhysicalQuantity = typing.Annotated[scalar_type, _PhysicsEnum.LENGTH]
    MASS: PhysicalQuantity = typing.Annotated[scalar_type, _PhysicsEnum.MASS]
    MOMENT_OF_INERTIA: PhysicalQuantity = typing.Annotated[scalar_type, _PhysicsEnum.MOMENT_OF_INERTIA]
    ANGLE: PhysicalQuantity = typing.Annotated[scalar_type, _PhysicsEnum.ANGLE]
    DIMENSIONLESS: PhysicalQuantity = typing.Annotated[scalar_type, _PhysicsEnum.DIMENSIONLESS]

    # endregion PhysicsTypes

    _unit_values: dict[_PhysicsEnum, tuple[np.ndarray, str]] = {
        _PhysicsEnum.LENGTH: (np.array(1e-3), 'mm'),
        _PhysicsEnum.MASS: (np.array(1.0), 'kg'),
        _PhysicsEnum.MOMENT_OF_INERTIA: (np.array(1.0), 'kg.m²'),
        _PhysicsEnum.ANGLE: (np.array(1.0), 'rad'),
        _PhysicsEnum.DIMENSIONLESS: (np.array(1.0), '(no unit)')
    }

    @classmethod
    def get_unit_value(cls, phy: PhysicalQuantity) -> np.ndarray:
        if not phy.__metadata__[0] or not isinstance(phy.__metadata__[0], _PhysicsEnum):
            raise ValueError("Invalid physical quantity type")
        return cls._get_unit_value(phy.__metadata__[0])

    @classmethod
    def _get_unit_value(cls, phy: _PhysicsEnum) -> np.ndarray:
        return cls._unit_values[phy][0]

    @classmethod
    def set_unit(cls, phy: PhysicalQuantity, value: scalar_type, symbol: str) -> None:
        if not phy.__metadata__[0] or not isinstance(phy.__metadata__[0], _PhysicsEnum):
            raise ValueError("Invalid physical quantity type")
        cls._unit_values[phy.__metadata__[0]] = np.array(value), symbol

    @classmethod
    def _input(cls, physics: PhysicalQuantity):
        phy = physics.__metadata__[0]
        return lambda value: value * cls._get_unit_value(phy)

    @classmethod
    def _output(cls, physics: None | PhysicalQuantity):
        if physics is None or not isinstance(physics, PhysicalQuantity) or not physics.__metadata__ or not isinstance(physics.__metadata__[0], _PhysicsEnum):
            return _identity
        phy = getattr(physics, '__metadata__')[0]
        return lambda value: value / cls._get_unit_value(phy)

    @classmethod
    def function(cls, func: types.FunctionType):
        """
        Function decorator that manages all arguments annotated with a PhysicalQuantity and the return value
        """
        arguments = func.__code__.co_varnames[:func.__code__.co_argcount]
        defaults = dict(zip(arguments[::-1], (func.__defaults__ or ())[::-1]))

        # all parameters that are annotated with a physical quantity
        physical_transforms = {
            name: cls._input(annotation)
            for name, annotation in func.__annotations__.items() if isinstance(annotation, PhysicalQuantity) and getattr(annotation, '__metadata__') and isinstance(annotation.__metadata__[0], _PhysicsEnum) and name != 'return'
        }
        return_transform = cls._output(func.__annotations__.get('return', None))

        @wraps(func)
        def new_function(*args, **kwargs):
            arg_dict = kwargs | dict(zip(arguments, args))
            arg_dict = {name: physical_transforms.get(name, _identity)(value) for name, value in arg_dict.items()}
            return return_transform(func(**(defaults | arg_dict)))

        return new_function

    @classmethod
    def class_(cls, target_class: type) -> type:
        """
        Class decorator that manages all methods with the `Physics.function` decorator and creates properties to manage attributes annotated with a PhysicalQuantity"""
        # Retrieve all method definitions
        for method_name, method in target_class.__dict__.items():
            if not isinstance(method, types.FunctionType) or (method_name != '__init__' and str.startswith(method_name, '__')):
                continue
            setattr(target_class, method_name, cls.function(method))

        # Retrieve all attributes that are annotated with physical quantities to place getters and setters on them
        for attr, unit in target_class.__annotations__.items():
            if not isinstance(unit, PhysicalQuantity) or not getattr(unit, '__metadata__') or not isinstance(unit.__metadata__[0], _PhysicsEnum):
                continue
            phy: _PhysicsEnum = getattr(unit, '__metadata__')[0]
            setattr(target_class, attr, property(
                lambda self: getattr(self, f'_{attr}') / cls._get_unit_value(phy),
                lambda self, value: setattr(self, f'_{attr}', value * cls._get_unit_value(phy))
            ))
        return target_class


def _identity(x):
    return x


if __name__ == '__main__':
    """
    Automatically writes Physical quantity types from _PhysicsEnum in Physics
    """

    with open(__file__, 'r') as this:
        whole_file = this.read()

    top_separator = "# region PhysicsTypes\n"
    bottom_separator = "# endregion PhysicsTypes\n"

    top, _ = whole_file.split(top_separator, maxsplit=1)
    _, bottom = whole_file.split(bottom_separator, maxsplit=1)

    center = '\n    '.join(f'{phy.name}: PhysicalQuantity = typing.Annotated[scalar_type, _PhysicsEnum.{phy.name}]' for phy in _PhysicsEnum)

    new_file = f'{top}{top_separator}\n    {center}\n\n    {bottom_separator}{bottom}'

    with open(__file__, 'w') as this:
        this.write(new_file)
