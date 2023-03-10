import numpy as np


def get_object(self):
    return self._object


def solid_checker(f):
    def g(self, s1, s2, *args, **kwargs):
        if isinstance(s1, str):
            s1 = get_object(self.named_sols[s1])
        if isinstance(s1, int):
            s1 = self._object.sols[s1]
        if hasattr(s1, '_object'):
            s1 = get_object(s1)
        if isinstance(s2, str):
            s2 = get_object(self.named_sols[s2])
        if isinstance(s2, int):
            s2 = self._object.sols[s2]
        if hasattr(s2, '_object'):
            s2 = get_object(s2)
        return f(self, s1, s2, *args, **kwargs)
    g.__doc__ = f.__doc__
    return g


def joint_checker(f):
    def g(self, j1, j2, *args, **kwargs):
        if isinstance(j1, str):
            j1 = get_object(self.named_joints[j1])
        if isinstance(j1, int):
            j1 = self._object.joints[j1]
        if hasattr(j1, '_object'):
            j1 = get_object(j1)
        if isinstance(j1, str):
            j2 = get_object(self.named_joints[j2])
        if isinstance(j1, int):
            j2 = self._object.joints[j2]
        if hasattr(j2, '_object'):
            j2 = get_object(j2)
        return f(self, j1, j2, *args, **kwargs)
    g.__doc__ = f.__doc__
    return g


def dfs(a):
    if hasattr(a, '__iter__'):
        for item in a:
            for i in dfs(item):
                yield i
    else:
        yield a


def multiple_joints(f):
    def g(self, *args):
        n_args = []
        for arg in args:
            if isinstance(arg, str):
                arg = get_object(self.named_joints[arg])
            if isinstance(arg, int):
                arg = self._object.joints[arg]
            if hasattr(arg, '_object'):
                arg = get_object(arg)
            n_args.append(arg)
        f(self, *n_args)
    g.__doc__ = f.__doc__
    return g


decor_divide = (lambda x, y: None if x is None else x / y), (lambda x, y: None if x is None else (lambda: x() / y))
decor_multiply = (lambda x, y: None if x is None else x * y), (lambda x, y: None if x is None else (lambda: x() * y))


FUNCTION_TYPE = type(lambda: None)


def physics_output(phy):
    def decor(f):
        def g(self, *args, **kwargs):
            return f(self, *args, **kwargs) / self._unit_system[phy][0]
        return g
    return decor


def to_function(f):
    def g(self, arg):
        if isinstance(arg, (int, float, np.ndarray)):
            return f(self, lambda: arg)
        elif not isinstance(arg, FUNCTION_TYPE):
            raise TypeError(f'Invalid type: {type(arg)}, expected int, float, np.ndarray or function')
        return f(self, arg)
    return g


def physics_input(*phy):
    def decor(method):
        cnt = method.__code__.co_argcount
        f_args = method.__code__.co_varnames[1:cnt]
        defaults = method.__defaults__ if method.__defaults__ is not None else ()
        shift = cnt - 1 - len(defaults)

        def g(self, *args, **kwargs):
            if len(args) + len(kwargs) > cnt:
                raise TypeError(f"Received too many arguments {len(args) + len(kwargs)}, at most {cnt-1} were expected")

            n_args = [value * self._unit_system[unit][0] if unit else value for value, unit in zip(args, phy)]

            for index, arg in enumerate(f_args[len(n_args):], len(n_args)):
                if arg not in kwargs:
                    if index < shift:
                        raise TypeError(f"Value axpected for {arg}, index {index}")
                    n_args.append(defaults[index - shift])
                elif phy[index]:
                    n_args.append(kwargs[arg] * self._unit_system[phy[index]][0])
                else:
                    n_args.append(kwargs[arg])
            return method(self, *n_args)

        g.__doc__ = method.__doc__
        return g
    return decor


def add_joint(self, cls, s1, s2, *args):
    joint = cls(self._unit_system, s1, s2, *args, f'{self._object.sols.index(s2)}/{self._object.sols.index(s1)}')
    self._object.joints.append(get_object(joint))
    self._object.interactions.append(joint.interaction)
    self.named_joints[repr(joint)] = joint
    return joint
