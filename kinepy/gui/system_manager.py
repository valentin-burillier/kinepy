import numpy as np
from kinepy.gui.drawing_tool import *
from kinepy.math.geometry import unit, z_cross, rvec, sq_mag, get_point
from kinepy.math.calculus import derivative_vec
from kinepy.objects.joints import Joint
from kinepy.objects.relations import LinearRelation


class SystemManager:
    force_scale = speed_scale = torque_scale = 0.

    def __init__(self, system, scale0, points, speeds, forces, torques,  joint_efforts):
        self.scale0 = scale0

        # shapes, mounting points of joints
        self.solid_data_step1 = tuple([] for _ in system.sols)
        self.solid_data_step2 = tuple([] for _ in system.sols)
        self.sols = system.sols

        for relation in system.relations:
            self.get_relation_data(relation)

        for joint in system.joints:
            self.get_joint_data(joint)

        self.traces = []
        for s, p, trace in points:
            self.point_data(s, p)
            if trace:
                self.traces.append((s, s.origin + rvec(s.angle, p)))

        for s, p in speeds:
            self.update_speed_scale(s, p)
        for s, p in speeds:
            self.speed_data(s, p)

        for s, p, f in forces:
            self.update_force_scale(f())
        for s, p, f in forces:
            self.force_data(s, p, f())

        for s, p, t in torques:
            self.update_torque_scale(t())
        for s, p, t in torques:
            self.torque_data(s, t())

        for j, rev in joint_efforts:
            self.effort_scales(j)
        for j, rev in joint_efforts:
            self.effort_data(j, rev)

    def do_nothing(self, *args, **kwargs):
        """Void function"""

    # relation positional data
    # region
    def gears_data(self, gear):
        r1, r2 = gear.get_radii()
        d = np.nanmax(sq_mag(get_point(gear.j1, 0) - get_point(gear.j2, 0)) ** .5)

        s1, p1 = (gear.j1.s1, gear.j1.s2)[not gear.common_eq[0]], (gear.j1.p1, gear.j1.p2)[not gear.common_eq[0]]
        self.solid_data_step1[s1.rep].append(('circle', (p1, r1 * d * self.scale0, False)))
        self.solid_data_step1[s1.rep].append(('lines', np.array((p1, p1 + r1 * d * unit(0)))))

        s2, p2 = (gear.j2.s1, gear.j2.s2)[not gear.common_eq[1]], (gear.j2.p1, gear.j2.p2)[not gear.common_eq[1]]
        self.solid_data_step1[s2.rep].append(('circle', (p2, r2 * d * self.scale0, False)))
        self.solid_data_step1[s2.rep].append(('lines', np.array((p2, p2 + r2 * d * unit(gear.v0)))))

    relation_data = {
        LinearRelation.GEAR: 'gears_data',
    }

    def get_relation_data(self, rel: LinearRelation):
        getattr(self, self.relation_data.get(rel.id_, 'do_nothing'))(rel)

    # endregion

    # joint positional data
    # region
    def revolute_data(self, rev):
        # Regions: GREEN, BLUE, RED, YELLOW
        offset_angles = -np.pi * .5, 0, np.pi, np.pi * .5
        for s, point in (rev.s1, rev.p1), (rev.s2, rev.p2):
            # region matching
            p = np.array(point) * self.scale0 / REVOLUTE_RADIUS
            if not s.rep or p[1] >= 2:
                region = 0
            elif p[1] <= -2:
                region = 3
            elif p[0] > 0:
                region = 2
            else:
                region = 1

            shape = np.einsum('ik,lk->li', rot(offset_angles[region]), REVOLUTE) / self.scale0 + point
            m_point = shape[REVOLUTE_MOUNTING_POINT]
            self.solid_data_step1[s.rep].append(('line', shape))
            if not s.rep:
                self.solid_data_step1[s.rep].append(('ground', m_point))
            elif region in (0, 3):
                self.solid_data_step1[s.rep].append(('lines', np.array(((0., 0.), (m_point[0], 0), m_point))))
            else:
                self.solid_data_step1[s.rep].append(('lines', np.array(((0., 0.), (0., m_point[1]), m_point))))
        self.solid_data_step2[rev.s2.rep].append(('circle', (rev.p2, REVOLUTE_RADIUS, True)))

    def prismatic_data(self, pri):
        u1, u2 = unit(pri.a1), unit(pri.a2)
        v1, v2 = pri.d1 * z_cross(u1), pri.d2 * z_cross(u2)

        shape1 = np.einsum('ik,lk->li', rot(pri.a1 + np.pi * (pri.d1 > 0)), PRISMATIC) / self.scale0 + v1
        m_point = shape1[PRISMATIC_MOUNTING_POINT]
        self.solid_data_step2[pri.s1.rep].append(('polygon', shape1))
        if not pri.s1.rep:
            self.solid_data_step1[pri.s1.rep].append(('ground', m_point))
        else:
            self.solid_data_step1[pri.s1.rep].append(('lines', np.array(((0., 0.), (m_point[0], 0), m_point))))

        offset = 3 * REVOLUTE_RADIUS / self.scale0
        min_sliding, max_sliding = -min(pri.sliding) + offset, -max(pri.sliding) - offset
        min_point, m_point, max_point = (
            u2 * min_sliding + v2, u2 * (min_sliding + max_sliding) * .5 + v2, u2 * max_sliding + v2
        )

        self.solid_data_step1[pri.s2.rep].append(('line', np.array((min_point, max_point))))
        if pri.s2.rep:
            self.solid_data_step1[pri.s2.rep].append(('lines', np.array(((0., 0.), (m_point[0], 0), m_point))))
        else:
            self.solid_data_step1[pri.s2.rep].append(('ground', m_point))

    def pin_slot_data(self, pin):
        # Regions: GREEN, BLUE, RED, YELLOW

        # s1
        offset = REVOLUTE_RADIUS / self.scale0
        u = unit(pin.a1)
        v = z_cross(u) * pin.d1
        min_sliding, max_sliding = min(pin.sliding) - offset, max(pin.sliding) + offset
        min_point, m_point, max_point = (
            u * min_sliding + v, u * (min_sliding + max_sliding) * .5 + v, u * max_sliding + v
        )
        self.solid_data_step1[pin.s1.rep].append(('line', np.array((min_point, max_point))))
        if pin.s1.rep:
            self.solid_data_step1[pin.s1.rep].append(('lines', np.array(((0., 0.), (m_point[0], 0), m_point))))
        else:
            self.solid_data_step1[pin.s1.rep].append(('ground', m_point))

        # s2 region matching
        p2 = np.array(pin.p2) * self.scale0 / REVOLUTE_RADIUS
        if not pin.s2.rep or p2[1] >= 1 + 3 ** .5:
            region2 = 0
        elif p2[1] <= -1 - 3 ** .5:
            region2 = 3
        elif p2[0] > 0:
            region2 = 2
        else:
            region2 = 1
        offset_angles2 = np.pi * .5, np.pi, 0., -np.pi * .5
        shape2 = np.einsum('ik,lk->li', rot(offset_angles2[region2]), PIN_SLOT) / self.scale0 + pin.p2
        m_point = shape2[PIN_SLOT_MOUNTING_POINT]
        self.solid_data_step2[pin.s2.rep].append(('polygon', shape2))
        if not pin.s2.rep:
            self.solid_data_step1[pin.s2.rep].append(('ground', m_point))
        elif region2 in (0, 3):
            self.solid_data_step1[pin.s2.rep].append(('lines', np.array(((0., 0.), (m_point[0], 0), m_point))))
        else:
            self.solid_data_step1[pin.s2.rep].append(('lines', np.array(((0., 0.), (0., m_point[1]), m_point))))

    get_data_dict = {
        Joint.REVOLUTE: 'revolute_data',
        Joint.PRISMATIC: 'prismatic_data',
        Joint.PIN_SLOT: 'pin_slot_data'
    }

    def get_joint_data(self, joint):
        getattr(self, self.get_data_dict.get(joint.id_, 'do_nothing'))(joint)
    # endregion

    # single objects data
    # region
    def point_data(self, solid, point):
        self.solid_data_step2[solid.rep].append(('circle', (point, REVOLUTE_RADIUS * .5, True)))
        self.solid_data_step1[solid.rep].append(('lines', np.array(((0., 0.), (point[0], 0), point))))

    def speed_data(self, solid, point):
        self.solid_data_step1[solid.rep].append(('lines', np.array(((0., 0.), (point[0], 0), point))))
        self.solid_data_step2[solid.rep].append(('circle', (point, REVOLUTE_RADIUS * .5, True)))

        real_point = solid.origin + rvec(solid.angle, point)
        speed = derivative_vec(real_point, 1.)
        mag = sq_mag(speed) ** .5
        angle = np.arccos(speed[0] / mag) * (2 * (speed[1] > 0) - 1)

        shape = np.einsum('ikn,lk->lin', rot(-angle), ARROW / self.scale0) * mag / self.speed_scale
        self.solid_data_step2[solid.rep].append(('arrow', (real_point, shape)))

    def force_data(self, solid, point, force):
        self.solid_data_step1[solid.rep].append(('lines', np.array(((0., 0.), (point[0], 0), point))))
        self.solid_data_step2[solid.rep].append(('circle', (point, REVOLUTE_RADIUS * .5, True)))

        real_point = solid.origin + rvec(solid.angle, point)
        mag = sq_mag(force) ** .5
        angle = np.arccos(force[0] / mag) * (2 * (force[1] > 0) - 1)

        shape = np.einsum('ikn,lk->lin', rot(-angle), ARROW / self.scale0) * mag / self.speed_scale
        self.solid_data_step2[solid.rep].append(('arrow', (real_point, shape)))

    def torque_data(self, solid, torque):
        shape = np.einsum(
            'ikn,lk->lin',
            rot(-solid.angle), CIRCLE_ARROW / self.scale0
        ) * torque / self.torque_scale
        self.solid_data_step2[solid.rep].append(('arrow', (solid.origin, shape)))
    # endregion

    # joint effort data
    # region
    def rev_effort(self, rev, reverse):
        real_point = rev.s1.origin + rvec(rev.s1.angle, rev.p1)
        mag = sq_mag(rev.force) ** .5
        f = (1, -1)[reverse]
        angle = np.arccos(f * rev.force[0] / mag) * (2 * (f * rev.force[1] > 0) - 1)
        shape = np.einsum('ikn,lk->lin', rot(-angle), ARROW / self.scale0) * mag / self.force_scale

        self.solid_data_step2[(rev.s2.rep, rev.s1.rep)[reverse]].append(('arrow', (real_point, shape)))

    def pri_effort(self, pri, reverse):
        real_point = pri.s1.origin + pri.d1 * z_cross(unit(pri.s1.angle + pri.a1))
        mag, angle = np.abs(pri.normal), pri.s1.angle + pri.a1 + np.pi * .5 * (1, -1)[reverse]
        shape = np.einsum('ikn,lk->lin', rot(-angle), ARROW / self.scale0) * mag / self.force_scale
        self.solid_data_step2[(pri.s2.rep, pri.s1.rep)[reverse]].append(('arrow', (real_point, shape)))

        torque = pri.torque * (1, -1)[reverse]
        shape = np.einsum(
            'ikn,lk->lin',
            rot(-(pri.s1.angle + pri.a1)), CIRCLE_ARROW / self.scale0
        ) * torque / self.torque_scale + pri.d1 * z_cross(unit(pri.s1.angle + pri.a1))
        self.solid_data_step2[(pri.s2.rep, pri.s1.rep)[reverse]].append(('arrow', (pri.s1.origin, shape)))

    def pin_effort(self, pin, reverse):
        u1 = unit(pin.s1.angle + pin.a1)
        real_point = pin.s1.origin + pin.d1 * z_cross(u1) + pin.sliding * u1
        mag, angle = np.abs(pin.normal) ** .5, pin.s1.angle + pin.a1 + np.pi * .5 * (1, -1)[reverse]
        shape = np.einsum('ikn,lk->lin', rot(-angle), ARROW / self.scale0) * mag / self.force_scale
        self.solid_data_step2[(pin.s2.rep, pin.s1.rep)[reverse]].append(('arrow', (real_point, shape)))

    effort_data_dict = {
        Joint.REVOLUTE: 'rev_effort',
        Joint.PRISMATIC: 'pri_effort',
        Joint.PIN_SLOT: 'pin_effort'
    }

    def effort_data(self, j, reverse):
        getattr(self, self.effort_data_dict.get(j.id_, 'do_nothing'))(j, reverse)
    # endregion

    # joint effort scales
    # region

    def rev_scales(self, rev):
        mag = sq_mag(rev.force) ** .5
        self.force_scale = max(self.force_scale, np.nanmax(mag))

    def pri_scales(self, pri):
        mag = np.abs(pri.normal)
        self.force_scale = max(self.force_scale, np.nanmax(mag))
        mag = np.abs(pri.torque)
        self.torque_scale = max(self.torque_scale, np.nanmax(mag))

    def pin_scales(self, pin):
        mag = np.abs(pin.normal)
        self.force_scale = max(self.force_scale, np.nanmax(mag))

    effort_scales_dict = {
        Joint.REVOLUTE: 'rev_scales',
        Joint.PRISMATIC: 'pri_scales',
        Joint.PIN_SLOT: 'pin_scales'
    }

    def effort_scales(self, j):
        getattr(self, self.effort_scales_dict.get(j.id_, 'do_nothing'))(j)

    # endregion

    # scale setters
    # region
    def update_speed_scale(self, solid, point):
        real_point = solid.origin + rvec(solid.angle, point)
        speed = derivative_vec(real_point, 1.)
        mag = sq_mag(speed) ** .5
        self.speed_scale = max(self.speed_scale, np.nanmax(mag))

    def update_force_scale(self, force):
        mag = sq_mag(force) ** .5
        self.force_scale = max(self.force_scale, np.nanmax(mag))

    def update_torque_scale(self, torque):
        mag = np.abs(torque)
        self.torque_scale = max(self.torque_scale, np.nanmax(mag))
    # endregion

    # parts drawing
    # region

    @staticmethod
    def arrow(camera, color, solid, data):
        frame = int(camera.animation_state)
        point, shape = data
        shape = camera.real_to_screen(point[:, frame]) + shape[:, :, frame] * camera.scale * camera.zoom
        pg.draw.polygon(
            camera.surface,
            color,
            shape,
            0
        )

    @staticmethod
    def shape_to_screen(camera, solid, shape):
        frame = int(camera.animation_state)
        origin = camera.real_to_screen(solid.origin[:, frame])
        new_shape = np.einsum('ik,lk->li', rot(-solid.angle[frame]), shape * (1, -1)) * camera.scale * camera.zoom
        return origin + new_shape

    @staticmethod
    def line(camera, color, solid, line):
        point1, point2 = SystemManager.shape_to_screen(camera, solid, line)
        pg.draw.line(
            camera.surface,
            color,
            point1,
            point2,
            2
        )

    @staticmethod
    def lines(camera, color, solid, lines):
        shape = SystemManager.shape_to_screen(camera, solid, lines)
        pg.draw.lines(
            camera.surface,
            color,
            False,
            shape,
            2
        )

    @staticmethod
    def circle(camera, color, solid, circle):
        p, rad, hide_bg = circle
        frame = int(camera.animation_state)
        point = camera.real_to_screen(solid.origin[:, frame] + rot(solid.angle[frame]) @ p)
        if hide_bg:
            pg.draw.circle(
                camera.surface,
                camera.background,
                point,
                rad * camera.scale * camera.zoom / camera.scale0,
                0
            )

        pg.draw.circle(
            camera.surface,
            color,
            point,
            rad * camera.scale * camera.zoom / camera.scale0,
            2
        )


    @staticmethod
    def polygon(camera, color, solid, polygon):
        poly = SystemManager.shape_to_screen(camera, solid, polygon)
        pg.draw.polygon(
            camera.surface,
            camera.background,
            poly,
            0
        )
        pg.draw.polygon(
            camera.surface,
            color,
            poly,
            2
        )

    @staticmethod
    def ground(camera, color, solid, point):
        p = camera.real_to_screen(point)
        ground = p + GROUND * (1, -1) * camera.scale * camera.zoom / camera.scale0
        for i in range(0, len(GROUND), 2):
            pg.draw.line(
                camera.surface,
                color,
                ground[i],
                ground[i + 1],
                2
            )
    # endregion

    def draw(self, camera):
        cam_center, surf_center = (
            np.array(camera.camera_area.center)[:, None], np.array(camera.surface.get_rect().center)[:, None]
        )

        for s, trace in self.traces:
            pg.draw.lines(
                camera.surface,
                solid_color(s.rep),
                False,
                np.swapaxes(np.int32(
                    (trace - cam_center) * camera.scale * camera.zoom * ((1,), (-1,)) + surf_center
                ), 0, 1),
                2
            )

        for solid, solid_data in zip(self.sols, self.solid_data_step1):
            color = solid_color(solid.rep)
            for shape, data in solid_data:
                getattr(self, shape)(camera, color, solid, data)

        for solid, solid_data in zip(self.sols, self.solid_data_step2):
            color = solid_color(solid.rep)
            for shape, data in solid_data:
                getattr(self, shape)(camera, color, solid, data)