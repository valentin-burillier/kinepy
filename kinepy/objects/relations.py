from kinepy.math.dynamic import group_tmd, group_trd, set_force
from kinepy.math.geometry import get_point, sq_mag, z_cross, np


class LinearRelationBase:
    v0 = r = j1 = j2 = None

    def rel_pilot(self, j_index, eq1, eq2):
        v = (self.j1.get_value() * self.r + self.v0) if j_index else (self.j2.get_value() - self.v0) / self.r
        (self.j1, self.j2)[j_index].set_value(v, eq1, eq2)

    def rel_block(self, j_index, eq1, eq2, ref):
        (self.j1, self.j2)[j_index].block(eq1, eq2, ref)


class LinearRelation(LinearRelationBase):
    def __init__(self, j1, j2, r, v0):
        self.j1, self.j2, self.r, self.v0 = j1, j2, r, v0


class EffortlessRelation(LinearRelation):
    pass


class DistantRelation(LinearRelation):
    """
    def rel_block(self, j_index, eq1, eq2, ref):
        LinearRelationBase.rel_block(self, j_index, eq1s1, eq2s2)
        add_effort(
            (self.j2, self.j1)[j_index],
            -self.r_ ** (2 * j_index - 1) * ((self.j1, self.j2)[j_index].get_effort())
        )
    """


class GearRelation(LinearRelationBase):
    common_eq = False, False

    def __init__(self, j1, j2, r, v0, pa):
        self.j1, self.j2, self.r, self.v0 = j1, j2, r, v0
        self.pressure_angle = pa


class Gear(GearRelation):
    def rel_block(self, direction, eq0, eq1, ref):
        eff_r = (self.r, -self.r)[self.common_eq[0] ^ self.common_eq[1]]
        p1, p2 = get_point(self.j1, 0), get_point(self.j2, 0)
        vec = p2 - p1
        radius = eff_r ** (1, 0)[direction] / (eff_r - 1)

        # compute force at gear contact
        t_10 = group_tmd((eq0, eq1), (0,), ref, (p1, p2)[direction]) / radius
        n_10 = np.tan(self.pressure_angle) * abs(t_10)
        f_10 = (vec * n_10 * (1, -1)[radius > 0] + z_cross(vec) * t_10) / sq_mag(vec)

        # add the force to gears
        contact_point = p1 + vec * eff_r / (eff_r - 1)
        j1, j2 = ((self.j2, self.j1), (self.j1, self.j2))[direction]
        (j2.s1, j2.s2)[not self.common_eq[direction]].add_mech_action(f_10, contact_point, 0.)
        (j1.s1, j1.s2)[not self.common_eq[not direction]].add_mech_action(-f_10, contact_point, 0.)

        # RevoluteJoint force
        set_force(j2, True, group_trd((eq1, eq0), (0,), ref))


class GearRack(GearRelation):
    def rel_block(self, direction, eq0, eq1, ref):
        pass
