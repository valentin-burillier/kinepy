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
    common_eq = None, None

    def __init__(self, j1, j2, r, v0, pa):
        self.j1, self.j2, self.r, self.v0 = j1, j2, r, v0
        self.pressure_angle = pa


class Gear(GearRelation):
    def rel_block(self, direction, eq0, eq1, ref):
        joint = (self.j1, self.j2)[not direction]
        vec = get_point(self.j2, 0) - get_point(self.j1, 0)
        r = self.r / (self.r - 1)
        vec = get_point(self.j1, 0) + r * vec - get_point(joint, 0)
        mag = sq_mag(vec) ** .5
        t10 = group_tmd((eq0, eq1), (0,), ref, get_point(joint, 0)) / mag
        vec /= mag
        f10 = z_cross(vec) * t10 - abs(t10) * np.tan(self.pressure_angle) * vec
        set_force(joint, True, f10)


class GearRack(GearRelation):
    """
    def _rel_block1(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        j1, j2 = self.j1, self.j2

        # indices du solide commun
        i1, i2 = (j1.s2 == j2.s1) or (j1.s2 == j2.s2),  (j1.s1 == j2.s2) or (j1.s2 == j2.s2)
        r = self.r * (2 * (i1 ^ i2) - 1)

        # s2 est-il le solide en commun
        c = s2 == (j2.s1, j2.s2)[i1]

        f_12 = trd(self.system, eq2)
        u = get_unit(j2, 0)
        p = j1.point + r * z_cross(u)
        tangent_12 = dot(f_12, u)
        normal_12 = np.tan(self.pressure_angle_) * np.abs(tangent_12) * (2 * (r > 0) - 1) * (-2 * c + 1)
        f_1_2 = u * tangent_12 + z_cross(u) * normal_12

        self.system.sols[(s1, (j1.s1, j1.s2)[not i2])[not c]].mech_actions.append(MechanicalAction(-f_1_2, p, 0.))
        self.system.sols[((j1.s1, j1.s2)[not i2], s2)[not c]].mech_actions.append(MechanicalAction(f_1_2, p, 0.))

        p = self.system.get_origin(s1)
        mp_12 = tmd(self.system, p, eq2)

        self.system.sols[s1].mech_actions.append(MechanicalAction(-f_12 + f_1_2, p, -mp_12))
        self.system.sols[s2].mech_actions.append(MechanicalAction(f_12 - f_1_2, p, mp_12))
    """

    """
    def _rel_block0(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        j1, j2 = self.j1, self.j2

        # indices du solide commun
        i1, i2 = (j1.s2 == j2.s1) or (j1.s2 == j2.s2),  (j1.s1 == j2.s2) or (j1.s2 == j2.s2)

        # s2 est-il le solide en commun
        c = s2 == (j1.s1, j1.s2)[i2]
        r = self.r * (2 * (i1 ^ i2) - 1)

        u = get_unit(j2, 0)
        p = j1.point
        m_12 = tmd(self.system, p, eq2)
        tangent_12 = -m_12 / r
        normal_12 = np.tan(self.pressure_angle) * np.abs(tangent_12) * (2 * (r > 0) - 1) * (2 * c - 1)
        f_1_2 = u * tangent_12 + z_cross(u) * normal_12

        f_12 = trd(self.system, eq2) - f_1_2
        self.system.sols[s1].mech_actions.append(MechanicalAction(-f_12 + f_1_2, p, 0))
        self.system.sols[s2].mech_actions.append(MechanicalAction(f_12 - f_1_2, p, 0))

        p = p + r * z_cross(u)

        self.system.sols[(s1, (j2.s1, j2.s2)[not i1])[not c]].mech_actions.append(MechanicalAction(-f_1_2, p, 0.))
        self.system.sols[((j2.s1, j2.s2)[not i1], s2)[not c]].mech_actions.append(MechanicalAction(f_1_2, p, 0.))

    def rel_block(self, j_index, eq1s1, eq2s2):
        (self._rel_block0, self._rel_block1)[j_index](eq1s1, eq2s2)
    """
