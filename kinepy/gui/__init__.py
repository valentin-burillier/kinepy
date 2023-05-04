from kinepy.gui.grid_manager import GridManager, Camera
from kinepy.gui.getters import *
from kinepy.gui.drawing_tool import *

from kinepy.math.geometry import unit, z_cross



# def rev_get_points(rev: RevoluteJoint):
#     return get_point(rev, 0) / units.SYSTEM[LENGTH],
#
#
# def pri_get_points(pri: PrismaticJoint):
#     u = unit(get_angle(pri, 0))
#     return get_zero(pri, 0, u) / units.SYSTEM[LENGTH], get_zero(pri, 1, u) / units.SYSTEM[LENGTH]
#
#
# def pin_get_points(pin: PinSlotJoint):
#     u = unit(pin.s1.angle + pin.a1)
#     return (
#         (pin.s1.origin + pin.d1 * z_cross(u)) / units.SYSTEM[LENGTH],
#         pin.s2.origin + rvec(pin.s2.angle, pin.p2) / units.SYSTEM[LENGTH]
#     )
#
#
# get_points = {
#     RevoluteJoint.tag: rev_get_points,
#     PrismaticJoint.tag: pri_get_points,
#     PinSlotJoint.tag: pin_get_points
# }
#
#
# class _GUI:
#     running = True
#     colors = (
#         (225, 105, 97), (255, 180, 128), (248, 243, 141), (66, 214, 164),
#         (8, 202, 209), (89, 173, 246), (157, 148, 255), (199, 128, 232)
#     )
#     joint_points = ()
#     additional_points = ()
#     scale = 1.
#     animation_state = 0
#     sol_mapping = ()
#     grid_cell = 0
#     min_ = 0
#     real_rectangle = 0
#     scale0 = 1
#
#     def __init__(self, system: _System, additional_points=(), background=(160, 160, 160), grid=False):
#         self.surface = pg.display.set_mode((640, 480), pg.RESIZABLE)
#
#         path = os.path.dirname(__file__)
#         pg.display.set_caption('Kinepy')
#         pg.display.set_icon(pg.image.load(os.path.join(path, 'logo.ico')).convert_alpha())
#
#         self.back_ground, self.grid = background, grid
#
#         self.current_ground = self.ground = pg.image.load(os.path.join(path, 'ground.png')).convert_alpha()
#
#         self.system: System = get_object(system)
#         self.additional_points = additional_points
#
#         self.camera, self.rectangle, self.screen_center = np.array((0., 0.)), np.array((0., 0.)), np.array((0., 0.))
#
#         self.make_joint_points()
#         self.make_bound_box()
#         self.set_scale()
#         self.scale0 = self.scale
#         self.compute_all_points()
#         self.make_sol_mapping()
#
#         self.clock = pg.time.Clock()
#
#     def make_joint_points(self):
#         self.joint_points = tuple(get_points[j.tag](j) for j in self.system.joints)
#
#     def make_bound_box(self):
#         min__ = (np.amin([np.amin(point, axis=1) for point in self.additional_points], axis=0),) if self.additional_points else ()
#         max__ = (np.amax([np.amax(point, axis=1) for point in self.additional_points], axis=0),) if self.additional_points else ()
#         min_ = np.amin(sum((tuple(np.amin(i, axis=1) for i in points) for points in self.joint_points), min__), axis=0)
#         max_ = np.amax(sum((tuple(np.amax(i, axis=1) for i in points) for points in self.joint_points), max__), axis=0)
#         self.camera, self.real_rectangle = (min_ + max_) / 2, (max_ - min_) * 1.05
#         self.min_ = min_
#         if self.real_rectangle[0] == 0. or self.real_rectangle[1] == 0:
#             raise ValueError("System does not use any area")
#
#     def set_scale(self):
#         self.screen_center = np.array(self.surface.get_size(), float) * .5
#         self.scale = min(self.surface.get_size() / self.real_rectangle)
#         self.min_ = self.camera - self.screen_center / self.scale
#         self.rectangle = 2 * self.screen_center / self.scale
#         self.grid_cell = 10 ** np.round(np.log10(min(self.rectangle) / 10))
#         print(f'\r{self.rectangle} {self.grid_cell} {self.min_} {self.scale}', end='')
#         self.current_ground = pg.transform.scale(self.ground, (15 * self.scale / self.scale0, 15 * self.scale / self.scale0))
#
#     def real_to_unscaled_screen(self, point):
#         return (point - self.camera[:, np.newaxis]) * ((1,), (-1,))
#
#     def compute_all_points(self):
#         self.additional_points = tuple(self.real_to_unscaled_screen(point) for point in self.additional_points)
#         self.joint_points = tuple(
#             tuple(self.real_to_unscaled_screen(point) for point in joint_points) for joint_points in self.joint_points
#         )
#
#     def make_sol_mapping(self):
#         self.sol_mapping = tuple(([], [], []) for _ in self.system.sols)
#         for joint, joint_points in zip(self.system.joints, self.joint_points):
#             jp = joint_points * 2
#             points, directions, joints = self.sol_mapping[joint.s1.rep]
#             points.append(jp[0])
#             directions.append(False)
#             joints.append(joint)
#             points, directions, joints = self.sol_mapping[joint.s2.rep]
#             points.append(jp[1])
#             directions.append(True)
#             joints.append(joint)
#
#     def event_loop(self):
#         for event in pg.event.get():
#             if event.type == pg.QUIT:
#                 self.running = False
#
#     def draw_solid(self, sol, joint_points, directions, joints):
#         if not sol.rep:
#             for point in joint_points:
#                 rect = self.current_ground.get_rect(midtop=point[:, self.animation_state] * self.scale + self.screen_center)
#                 self.surface.blit(self.current_ground, rect)
#
#     def draw_loop(self):
#         self.surface.fill(self.back_ground)
#         if self.grid:
#             start = (self.min_ // self.grid_cell + 1) * self.grid_cell
#             end = self.min_ + self.rectangle
#             x0, x1 = self.min_[0], self.min_[0] + self.rectangle[0]
#             y0, y1 = self.min_[1], self.min_[1] + self.rectangle[1]
#             while end[0] - start[0] > 0:
#                 pg.draw.line(
#                     self.surface, (0, 0, 0),
#                     ((start[0], y0) - self.camera) * (1, -1) * self.scale + self.screen_center,
#                     ((start[0], y1) - self.camera) * (1, -1) * self.scale + self.screen_center,
#                 )
#                 start[0] += self.grid_cell
#             while end[1] - start[1] > 0:
#                 pg.draw.line(
#                     self.surface, (0, 0, 0),
#                     ((x0, start[1]) - self.camera) * (1, -1) * self.scale + self.screen_center,
#                     ((x1, start[1]) - self.camera) * (1, -1) * self.scale + self.screen_center,
#                 )
#                 start[1] += self.grid_cell
#         for sol, mapping in zip(self.system.sols, self.sol_mapping):
#             self.draw_solid(sol, *mapping)
#         for joint, points in zip(self.system.joints, self.joint_points):
#             for point, sol in zip(points, (joint.s1, joint.s2)):
#                 color = self.colors[sol.rep % len(self.colors)]
#                 pg.draw.circle(self.surface, color, point[:, self.animation_state] * self.scale + self.screen_center, 4)
#         pg.display.flip()
#         self.animation_state = (self.animation_state + 1) % self.system.n
#
#     def main_loop(self):
#         while self.running:
#             self.event_loop()
#             self.set_scale()
#             self.draw_loop()
#             self.clock.tick(30)

FPS = 60
PATH = os.path.dirname(__file__)


class GUI(GridManager):
    running = True
    animation_state, animation_speed = 0, .6

    def __init__(self, system, background, animation_time, display_frames,  *args):
        self.system = get_object(system)
        self.display_frames = display_frames

        # Window setup
        self.surface = pg.display.set_mode((640, 480), pg.RESIZABLE)
        pg.display.set_caption('Kinepy')
        pg.display.set_icon(pg.image.load(os.path.join(PATH, 'logo.ico')).convert_alpha())

        self.background = background
        self.animation_speed = self.system.n / animation_time / FPS

        # Clock for the frame rate
        self.clock = pg.time.Clock()

        # Camera stuff
        Camera.__init__(self)
        self.points, self.solid_set = gather_points(self.system)
        self.set_bound_box(self.points)
        self.set_scale()
        self.scale0 = self.scale

        self.prepared_joints = prepare_joints(self.system.joints)

        # Grid stuff
        self.find_unit()
        self.camera_borders()

    def events(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.running = False
            GridManager.manage(self, event)

    def draw(self):
        self.surface.fill(self.background)
        frame = int(self.animation_state)

        GridManager.draw_grid(self, COLORMAP[0])

        if self.display_frames:
            # drawing frames
            for index, sol in enumerate(self.system.sols):
                origin = self.real_to_screen(sol.origin[:, frame])
                u = unit(-sol.angle[frame]) * 40 * self.scale / self.scale0 * self.zoom
                draw_arrow(self.surface, solid_color(index), origin, origin + u)
                draw_arrow(self.surface, solid_color(index), origin, origin - z_cross(u))

        for j, data in zip(self.system.joints, self.prepared_joints):
            if j.id_ not in draw_joint:
                continue
            draw_joint[j.id_](self, j, data)

    def tick(self):
        self.animation_state = (self.animation_state + self.animation_speed) % self.system.n

    def main_loop(self):
        while self.running:
            # manage events
            self.events()

            # do anything that needs to be done, then get displayed on the window
            self.tick()
            self.draw()
            pg.display.flip()

            # 60 fps
            self.clock.tick(FPS)


def display(system, additional_points=(), background=(0, 0, 0), display_frames_of_reference=True, animation_time=2.):
    pg.init()
    GUI(system, background, animation_time, display_frames_of_reference, additional_points).main_loop()
    pg.quit()

