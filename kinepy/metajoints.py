import numpy as np
from kinepy.geometry import mag, angle2, mat_mul_n, rot, unit, get_unit, dot, z_cross
from kinepy.dynamic import tmd, trd, MechanicalAction, add_effort


class LinearRelation:
    def __init__(self, system, j1, j2, r, v0):
        self.v0_physics = 'Angle' if not system.joints[j2].id_ else 'Length'
        self.system, self.j1, self.j2, self.r, self.v0_ = system, j1, j2, r, v0 * system.units[self.v0_physics]

    @property
    def v0(self):
        return self.v0_ / self.system.units[self.v0_physics]

    @v0.setter
    def v0(self, value):
        self.v0_ = value * self.system.units[self.v0_physics]

    def __pilot_joint0(self, si):
        return self.system.joints[self.j1].set_value((self.system.joints[self.j2].get_value() - self.v0_) / self.r, si)

    def __pilot_joint1(self, si):
        return self.system.joints[self.j2].set_value(self.system.joints[self.j1].get_value() * self.r + self.v0_, si)

    __pilot_joint = __pilot_joint0, __pilot_joint1

    def rel_pilot(self, j_index, si):
        return self.__pilot_joint[j_index](self, si)

    def rel_block(self, j_index, eq1s1, eq2s2):
        self.system.joints[(self.j1, self.j2)[j_index]].block(eq1s1, eq2s2)


class Gear(LinearRelation):
    def __init__(self, system, j1, j2, r, v0, pa=None):
        LinearRelation.__init__(self, system, j1, j2, r, v0)
        self.pressure_angle_ = np.pi/9 if pa is None else pa * system.units['Angle']

    @property
    def pressure_angle(self):
        return self.pressure_angle_ / self.system.units['Angle']

    @pressure_angle.setter
    def pressure_angle(self, value):
        self.pressure_angle_ = value * self.system.units['Angle']

    def rel_block(self, j_index, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        j1, j2 = (self.system.joints[self.j1], self.system.joints[self.j2])[::(1, -1)[j_index]]

        # indices du solide commun
        i1, i2 = (j1.s2 == j2.s1) or (j1.s2 == j2.s2),  (j1.s1 == j2.s2) or (j1.s2 == j2.s2)
        r = (self.r * (-2 * (i1 ^ i2) + 1)) ** (1, -1)[j_index]

        # s2 est-il le solide en commun
        c = s2 == (j1.s1, j1.s2)[i1]

        a, b = j1.point, j2.point
        d = mag(b - a)
        theta = angle2(b - a, 1 / d)
        rad = r * d / (r - 1)

        m_12 = tmd(self.system, a, eq2)
        n_12 = m_12 / rad
        t_12 = np.abs(n_12) * np.tan(self.pressure_angle_) * (2 * ((rad < 0) ^ c) - 1)
        f_12 = mat_mul_n(rot(theta), (t_12, n_12))
        f_s12 = trd(self.system, eq2) - f_12

        self.system.sols[s1].mech_actions.append(MechanicalAction(-f_s12, a, 0.))
        self.system.sols[s2].mech_actions.append(MechanicalAction(f_s12, a, 0.))

        p = a + rad * unit(theta)
        self.system.sols[(s1, (j2.s1, j2.s2)[not i2])[not c]].mech_actions.append(MechanicalAction(-f_12, p, 0.))
        self.system.sols[((j2.s1, j2.s2)[not i2], s2)[not c]].mech_actions.append(MechanicalAction(f_12, p, 0.))


class GearRack(LinearRelation):
    def __init__(self, system, j1, j2, r, v0, pa=None):
        LinearRelation.__init__(self, system, j1, j2, r, v0)
        self.pressure_angle_ = np.pi/9 if pa is None else pa * system.units['Angle']

    @property
    def pressure_angle(self):
        return self.pressure_angle_ / self.system.units['Angle']

    @pressure_angle.setter
    def pressure_angle(self, value):
        self.pressure_angle_ = value * self.system.units['Angle']

    def _rel_block1(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        j1, j2 = self.system.joints[self.j1], self.system.joints[self.j2]

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

    def _rel_block0(self, eq1s1, eq2s2):
        (_, s1), (eq2, s2) = eq1s1, eq2s2
        j1, j2 = self.system.joints[self.j1], self.system.joints[self.j2]

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


class EffortlessRelation(LinearRelation):
    pass


class DistantRelation(LinearRelation):
    def rel_block(self, j_index, eq1s1, eq2s2):
        LinearRelation.rel_block(self, j_index, eq1s1, eq2s2)
        j1, j2 = self.system.joints[self.j1], self.system.joints[self.j2]
        add_effort((j2, j1)[j_index], -self.r ** (2 * j_index - 1) * ((j1, j2)[j_index].get_effort()))
