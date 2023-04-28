from kinepy.gui.grid_manager import GridManager, Camera
from kinepy.gui.getters import *
from kinepy.gui.drawing_tool import *

from kinepy.math.geometry import unit, z_cross


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

        GridManager.draw_grid(self, (255, 255, 255))

        if self.display_frames:
            # drawing frames
            for index, sol in enumerate(self.system.sols):
                origin = self.real_to_screen(sol.origin[:, frame])
                u = unit(-sol.angle[frame]) * 40 * self.scale / self.scale0 * self.zoom
                draw_arrow(self.surface, SOLID_COLORS[index % len(SOLID_COLORS)], origin, origin + u)
                draw_arrow(self.surface, SOLID_COLORS[index % len(SOLID_COLORS)], origin, origin - z_cross(u))

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
