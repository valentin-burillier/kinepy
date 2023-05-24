from kinepy.gui.grid_manager import GridManager, Camera
from kinepy.gui.getters import *
from kinepy.gui.drawing_tool import *
import os

FPS = 24
PATH = os.path.dirname(__file__)


class GUI:
    running, animating = True, True
    allow_moving = False
    camera_pos = 0, 0

    def __init__(self, system, background_color, animation_time, frames_of_reference, grid, graduations, figure_size, points, speeds, forces, torques, joint_efforts, saving=False):
        self.grid = GridManager()

        self.grid.use_grid = grid
        self.grid.use_graduation = graduations

        # Window setup
        w, h = figure_size
        w, h = w if not graduations else int(w / .7), h + 50 if not graduations else int(h / .85 + 50)
        self.surface = pg.display.set_mode((w, h),  pg.RESIZABLE) if not saving else pg.Surface((w, h))
        self.camera = Camera(self.replace_camera(), system, frames_of_reference, background_color, points, speeds, forces, torques, joint_efforts)
        self.grid.change_scale(self.camera)

        if not saving:
            pg.display.set_caption('Kinepy')
            pg.display.set_icon(pg.image.load(os.path.join(PATH, 'logo.ico')).convert_alpha())

        self.background = background_color
        self.animation_speed = self.camera.animation_speed = get_object(system).n / animation_time / FPS

        # Clock for the frame rate
        self.clock = pg.time.Clock()

    def replace_camera(self):
        w, h = self.surface.get_size()
        if self.grid.use_graduation:
            self.camera_pos = 0.15 * w, 0.05 * (h - 50)
            return self.surface.subsurface(pg.Rect(self.camera_pos, (.7 * w, 0.85 * (h - 50))))
        self.camera_pos = 0, 0
        return self.surface.subsurface(pg.Rect(0, 0, 1. * w, h - 50))

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
        r = self.camera.surface.get_rect()
        if r.w * r.h:
            self.camera.set_scale()
            self.grid.change_scale(self.camera)
            self.grid.set_scale(self.camera)

    def click(self, event):
        in_drawing_area = self.camera.surface.get_rect(topleft=self.camera_pos).collidepoint(event.pos)
        if event.button in (pg.BUTTON_WHEELUP, pg.BUTTON_WHEELDOWN) and in_drawing_area:
            event.pos -= np.array(self.camera_pos)
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
            shifter = (1, 3, 0, 2)
            n = shifter[self.grid.use_grid + self.grid.use_graduation * 2]
            self.grid.use_grid = not not (n & 1)
            self.grid.use_graduation = not not (n & 2)
            self.camera.surface = self.replace_camera()
            self.camera.set_scale()
            self.grid.set_scale(self.camera)
            self.grid.change_scale(self.camera)
        if event.key == pg.K_SPACE:
            self.animating = not self.animating
            self.camera.animation_speed = self.animation_speed * self.animating
        if event.key == pg.K_LEFT and not self.animating:
            self.camera.animation_state = (self.camera.animation_state - 1) % self.camera.system.n
        if event.key == pg.K_RIGHT and not self.animating:
            self.camera.animation_state = (self.camera.animation_state + 1) % self.camera.system.n

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
        r = self.camera.surface.get_rect()
        if r.w * r.h:
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

            # frame rate control
            self.clock.tick(FPS)

