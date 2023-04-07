import kinepy.units as units
from kinepy.units import *


P1, P2 = ('p1', LENGTH), ('p2', LENGTH)
A1, A2 = ('a1', ANGLE), ('a2', ANGLE)
D1, D2 = ('d1', LENGTH), ('d2', LENGTH)
ANGLE_ = 'angle', ANGLE

NORMAL, TANGENT = ('normal', FORCE), ('tangent', FORCE)
FORCE_, TORQUE_ = ('force', FORCE), ('torque', TORQUE)

SLIDING = 'sliding', LENGTH


class MetaUnit(type):
    def __new__(mcs, name, bases, dct: dict):

        if 'read_only' in dct:
            for tag, phy in dct['read_only']:
                dct[tag] = MetaUnit.read_only(tag, phy)
        if 'read_write' in dct:
            for tag, phy in dct['read_write']:
                dct[tag] = MetaUnit.read_write(tag, phy)

        old__init__ = dct.get("__init__", lambda self: None)

        def __init__(self, *args):
            self._object = bases[0](*args)
            old__init__(self)

        dct["__init__"] = __init__
        dct["__repr__"] = lambda self: self._object.name

        return type.__new__(mcs, name, bases[1:], dct)

    @staticmethod
    def read_only(tag, phy):
        return property(
            (lambda self: self._object.__getattribute__(f'{tag}') / units.SYSTEM[phy])
        )

    @staticmethod
    def read_write(tag, phy):
        return property(
            (lambda self: self._object.__getattribute__(f'{tag}') / units.SYSTEM[phy]),
            (lambda self, value: self._object.__setattr__(f'{tag}', value * units.SYSTEM[phy])
             if value is not None else None)
        )
