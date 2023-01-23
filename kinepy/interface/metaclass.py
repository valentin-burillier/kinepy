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
            for tag, phy, def_ in dct['read_only']:
                dct[tag] = MetaUnit.read_only(tag, phy)
        if 'read_write' in dct:
            for tag, phy in dct['read_write']:
                dct[tag] = MetaUnit.read_write(tag, phy)

        old__init__ = dct.get("__init__", lambda self: None)

        def __init__(self, unit_system, *args):
            self._unit_system = unit_system
            self._object = bases[0](*args)
            old__init__(self)

        dct["__init__"] = __init__
        dct["__repr__"] = lambda self: self._object.name

        return type.__new__(mcs, name, bases[1:], dct)

    @staticmethod
    def read_only(tag, phy):
        return property(
            (lambda self: self._object.__getattribute__(f'{tag}') / self._unit_system[phy])
        )

    @staticmethod
    def read_write(tag, phy):
        return property(
            (lambda self: self._object.__getattribute__(f'{tag}') / self._unit_system[phy]),
            (lambda self, value: self.objct.__setattr__(f'{tag}', value * self._unit_system[phy])
             if value is not None else None)
        )
