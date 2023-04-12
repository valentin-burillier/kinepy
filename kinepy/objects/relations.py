from kinepy.math.dynamic import group_tmd, group_trd, set_force, set_normal
from kinepy.math.geometry import get_point, sq_mag, z_cross, np, unit, get_angle, dot, get_zero, ZERO_21
from kinepy.objects.joints import RevoluteJoint, PrismaticJoint


class LinearRelationBase:
    v0 = r = j1 = j2 = None

    def rel_pilot(self, j_index, eq1, eq2):
        v = (self.j1.get_value() * self.r + self.v0) if j_index else (self.j2.get_value() - self.v0) / self.r
        (self.j1, self.j2)[j_index].set_value(v, eq1, eq2)

    def rel_block(self, direction, eq0, eq1, ref):
        (self.j1, self.j2)[direction].block(eq0, eq1, ref)


class LinearRelation(LinearRelationBase):
    def __init__(self, j1, j2, r, v0):
        self.j1, self.j2, self.r, self.v0 = j1, j2, r, v0


class EffortlessRelation(LinearRelation):
    pass


class DistantRelation(LinearRelation):
    def rel_block(self, direction, eq0, eq1, ref):
        LinearRelationBase.rel_block(self, direction, eq0, eq1, ref)
        (self.j1, self.j2)[not direction].put_effort((self.j1, self.j2)[direction] * self.r ** (-1, 1)[direction])


class GearRelation(LinearRelationBase):
    common_eq = False, False
    contact_point = contact_force = None
    tmp = None

    def __init__(self, j1, j2, r, v0, pa):
        self.j1, self.j2, self.r, self.v0 = j1, j2, r, v0
        self.pressure_angle = pa

    def _solve_contact(self, direction, radius, vec, contact_point, t_gear):
        n_gear = np.tan(self.pressure_angle) * abs(t_gear) * (1, -1)[(radius > 0)]
        # force applied on the propagated gear
        f_gear = vec * n_gear + z_cross(vec) * t_gear

        # add the force to gears
        j1, j2 = ((self.j2, self.j1), (self.j1, self.j2))[direction]
        (j2.s1, j2.s2)[not self.common_eq[direction]].add_mech_action(f_gear, contact_point, 0.)
        (j1.s1, j1.s2)[not self.common_eq[not direction]].add_mech_action(-f_gear, contact_point, 0.)

        self.contact_force = f_gear * (1, -1)[direction]


class Gear(GearRelation):
    def rel_block(self, direction, eq0, eq1, ref):
        eff_r = (self.r, -self.r)[self.common_eq[0] ^ self.common_eq[1]]
        p1, p2 = get_point(self.j1, 0), get_point(self.j2, 0)
        radius = eff_r ** (1, 0)[direction] / (eff_r - 1)
        t_gear = group_tmd((eq0, eq1), (not self.common_eq[direction],), ref, (p1, p2)[direction]) / radius

        vec = p2 - p1
        self.contact_point = contact_point = p1 + vec * eff_r / (eff_r - 1)
        self._solve_contact(direction, radius, vec / sq_mag(vec), contact_point, t_gear)

        # RevoluteJoint force
        set_force((self.j1, self.j2)[direction], True, group_trd((eq0, eq1), (0,), ref))


class GearRack(GearRelation):
    def rel_block(self, direction, eq0, eq1, ref):
        eff_r = self.r * (1, -1)[self.common_eq[0] ^ self.common_eq[1]]
        v = -z_cross(u := unit(get_angle(self.j2, 0)))
        self.contact_point = contact_point = get_point(self.j1, 0) + v * eff_r
        if not direction:
            t_gear = group_tmd((eq0, eq1), (not self.common_eq[direction],), ref, get_point(self.j1, 0)) / eff_r
            self._solve_contact(direction, eff_r, v, contact_point, t_gear)
            set_force(self.j1, True, group_trd((eq1, eq0), (0,), ref))
        else:
            t_gear = dot(group_trd((eq0, eq1), (not self.common_eq[direction],), ref), u)
            self._solve_contact(direction, -eff_r, v, contact_point, t_gear)
            n_10, m_10 = group_trd((eq0, eq1), (0,), ref), group_tmd((eq0, eq1), (0,), ref, get_zero(self.j2, 0, u))
            set_normal(self.j2, True, n_10, m_10, u)
