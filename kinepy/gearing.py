

class LinearRelation:
    def __init__(self, system, j1, j2, r, v0):
        self.system, self.j1, self.j2, self.r, self.v0 = system, j1, j2, r, v0


class Gear(LinearRelation):
    pass


class GearRack(LinearRelation):
    pass


class EffortlessRelation(LinearRelation):
    pass


class DistantRelation(LinearRelation):
    pass
