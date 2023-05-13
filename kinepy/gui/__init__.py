from kinepy.gui.grid_manager import GridManager, Camera
from kinepy.gui.getters import *
from kinepy.gui.drawing_tool import *

from kinepy.math.geometry import unit, z_cross


FPS = 60
PATH = os.path.dirname(__file__)


class GUI:
    running = True
    allow_moving = False
    camera_pos = 0, 0

    def __init__(self, system, background, animation_time, display_frames, *args):
        self.grid = GridManager()
        self.grid.use_grid = True
        self.grid.use_graduation = True

        # Window setup
        self.surface = pg.display.set_mode((640, 480), pg.RESIZABLE)
        self.camera = Camera(self.replace_camera(), system, display_frames, background)
        self.grid.change_scale(self.camera)

        pg.display.set_caption('Kinepy')
        pg.display.set_icon(pg.image.load(os.path.join(PATH, 'logo.ico')).convert_alpha())

        self.background = background
        self.camera.animation_speed = get_object(system).n / animation_time / FPS

        # Clock for the frame rate
        self.clock = pg.time.Clock()

    def replace_camera(self):
        w, h = self.surface.get_size()
        if self.grid.use_graduation:
            self.camera_pos = 0.15 * w, 0.05 * (h - 50)
            return self.surface.subsurface(pg.Rect(self.camera_pos, (.7 * w, 0.85 * (h - 50))))
        self.camera_pos = 0, 0
        return self.surface.subsurface(pg.Rect(0, 0, 1. * w, 0.9 * h))

    def do_nothing(self, event):
        pass

    def quit(self, event):
        self.running = False

    def resize(self, event):
        w, h = self.surface.get_size()
        if h < 50:
            h = 50
            self.surface = pg.display.set_mode((w, h), pg.RESIZABLE)
        self.camera.surface = self.replace_camera()
        self.camera.set_scale()
        self.grid.change_scale(self.camera)
        self.grid.set_scale(self.camera)

    def click(self, event):
        in_drawing_area = self.camera.surface.get_rect(topleft=self.camera_pos).collidepoint(event.pos)
        if event.button in (pg.BUTTON_WHEELUP, pg.BUTTON_WHEELDOWN) and in_drawing_area:
            self.camera.change_scale(event)
            self.grid.change_scale(self.camera)
        if event.button == pg.BUTTON_LEFT and in_drawing_area:
            self.allow_moving = True

    def click_release(self, event):
        if event.button == pg.BUTTON_LEFT and self.allow_moving:
            self.allow_moving = False

    def mouse_motion(self, event):
        if self.allow_moving and event.buttons == (1, 0, 0):
            self.camera.move_camera(event)
            self.grid.move_camera(self.camera)

    def keydown(self, event):
        if event.key == pg.K_TAB:
            n = (self.grid.use_grid * 2 + self.grid.use_graduation) + 1
            self.grid.use_grid = not not (n & 2)
            self.grid.use_graduation = not not (n & 1)
            self.camera.surface = self.replace_camera()Gradu

    event_dict = {
        pg.QUIT: 'quit',
        pg.VIDEORESIZE: 'resize',
        pg.MOUSEBUTTONDOWN: 'click',
        pg.MOUSEBUTTONUP: 'click_release',
        pg.MOUSEMOTION: 'mouse_motion',
        pg.KEYDOWN: 'keydown'
    }

    def manage_events(self):
        for event in pg.event.get():
            getattr(self, self.event_dict.get(event.type, 'do_nothing'))(event)

    def tick(self):
        self.camera.animation_state = (self.camera.animation_state + self.camera.animation_speed) % self.camera.system.n

    def draw(self):
        color = np.array(self.background)
        self.surface.fill(np.uint8((color > 128) * color * .8 + (255 - .8 * (255 - color)) * (color < 129)))
        self.camera.surface.fill(self.camera.background)
        self.grid.draw_grid(self.camera, COLORMAP[0])
        self.camera.draw()
        self.grid.draw_graduations(self.surface, self.camera, COLORMAP[0], self.camera_pos)

    def main_loop(self):
        while self.running:
            # manage events
            self.manage_events()

            # do anything that needs to be done, then get displayed on the window
            self.tick()
            self.draw()
            pg.display.flip()

            # 60 fps
            self.clock.tick(FPS)


def display(system, additional_points=(), background=(255, 255, 255), display_frames_of_reference=True, animation_time=2.):
    pg.init()
    GUI(system, background, animation_time, display_frames_of_reference, additional_points).main_loop()
    pg.quit()

