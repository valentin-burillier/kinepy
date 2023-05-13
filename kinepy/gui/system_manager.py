from kinepy.gui.drawing_tool import *


class SystemManager:

    def revolute_data(self, rev):
        # Regions: GREEN, BLUE, RED, YELLOW
        offset_angles = -np.pi * .5, 0, np.pi, np.pi * .5
        for s, point in (rev.s1, rev.p1), (rev.s2, rev.p2):
            # region matching
            p = np.array(point) * self.scale0 / REVOLUTE_RADIUS
            if p[1] >= 2:
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
        self.solid_data_step2[rev.s2.rep].append(('circle', (rev.p2, REVOLUTE_RADIUS)))

    get_data_dict = {
        1: 'revolute_data',
    }

    def get_data(self, joint):
        getattr(self, self.get_data_dict[joint.id_])(joint)

    def __init__(self, system, additional_points, scale0):
        self.scale0 = scale0

        # shapes, mounting points of joints
        self.solid_data_step1 = tuple([] for _ in system.sols)
        self.solid_data_step2 = tuple([] for _ in system.sols)
        self.sols = system.sols

        for joint in system.joints:
            if joint.id_ in self.get_data_dict:
                self.get_data(joint)

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
        p, rad = circle
        frame = int(camera.animation_state)
        point = camera.real_to_screen(solid.origin[:, frame] + rot(solid.angle[frame]) @ p)
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


    def draw(self, camera):
        for solid, solid_data in zip(self.sols, self.solid_data_step1):
            color = solid_color(solid.rep)
            for shape, data in solid_data:
                getattr(self, shape)(camera, color, solid, data)

        for solid, solid_data in zip(self.sols, self.solid_data_step2):
            color = solid_color(solid.rep)
            for shape, data in solid_data:
                getattr(self, shape)(camera, color, solid, data)