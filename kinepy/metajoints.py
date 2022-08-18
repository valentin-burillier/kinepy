

class LinearRelation:
    def __init__(self, system, j1, j2, r, v0):
        self.system, self.j1, self.j2, self.r, self.v0 = system, j1, j2, r, v0

    def __pilot_joint0(self):
        return self.system.joints[self.j1].set_value((self.system.joints[self.j2].get_value() - self.v0) / self.r)

    def __pilot_joint1(self):
        return self.system.joints[self.j2].set_value(self.system.joints[self.j1].get_value() * self.r + self.v0)

    __pilot_joint = __pilot_joint0, __pilot_joint1

    def rel_pilot(self, index):
        return self.__pilot_joint[index](self)


class Gear(LinearRelation):
    pass


class GearRack(LinearRelation):
    pass


class EffortlessRelation(LinearRelation):
    pass


class DistantRelation(LinearRelation):
    pass
