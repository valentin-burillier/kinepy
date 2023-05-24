from kinepy.gui.Rect import Rect
from kinepy.gui.getters import *
from kinepy.gui.drawing_tool import *
from kinepy.gui.system_manager import SystemManager
from kinepy.math.geometry import unit, z_cross


class Camera:
    scale = scale0 = zoom = 1.
    animation_state, animation_speed = 0, .6

    def __init__(self, surface, system, display_frames, background, points, speeds, forces, torques, joint_efforts):
        self.system = get_object(system)
        self.display_frames = display_frames

        self.surface = surface
        self.background = background
        self.system_area = self.camera_area = Rect(0, 0, 0, 0)

        self.points = gather_points(self.system, points, speeds, forces)
        self.set_bound_box(self.points)
        self.set_scale()
        self.scale0 = self.scale

        self.sys_mgr = SystemManager(self.system, self.scale0, points, speeds, forces, torques, joint_efforts)

    def set_bound_box(self, points: tuple):
        min_ = np.amin(points, axis=(0, 2))
        max_ = np.amax(points, axis=(0, 2))
        self.system_area = Rect(*min_, *(max_ - min_))
        self.system_area.scale(1.2, self.system_area.w * .5, self.system_area.h * .5)
        if self.system_area.w == 0. or self.system_area.h == 0:
            raise ValueError("System does not use any area")

    def set_scale(self):
        self.scale = min(self.surface.get_size() / np.array((self.system_area.w, self.system_area.h)))
        old_camera = self.camera_area.center
        self.camera_area = Rect(0, 0, *(self.surface.get_size() / np.array(self.scale)))
        self.camera_area.scale(1 / self.zoom, *self.camera_area.center)
        self.camera_area.center = old_camera
        self.camera_area.clamp(self.system_area)

    def move_camera(self, event):
        self.camera_area.move(*(event.rel * np.array((-1, 1)) / self.scale / self.zoom))
        self.camera_area.clamp(self.system_area)

    def change_scale(self, event):
        old_zoom = self.zoom
        dp = np.array((1, -1.)) * event.pos / self.zoom / self.scale + np.array((0., self.camera_area.h))
        self.zoom = max(1., min(10., self.zoom * 1.05 ** (2 * (event.button == pg.BUTTON_WHEELUP) - 1)))
        self.camera_area.scale(old_zoom / self.zoom, *dp)
        self.camera_area.clamp(self.system_area)

    def manage(self, event):
        # mouse scroll to zoom
        if event.type == pg.MOUSEBUTTONDOWN and event.button in (pg.BUTTON_WHEELUP, pg.BUTTON_WHEELDOWN):
            self.change_scale(event)

        # click and move to navigate
        elif event.type == pg.MOUSEMOTION and event.buttons == (1, 0, 0):
            self.move_camera(event)

        elif event.type == pg.WINDOWRESIZED:
            self.set_scale()

    def real_to_screen(self, point):
        return np.int32(
            (point - self.camera_area.center) * self.scale * self.zoom * (1, -1) + self.surface.get_rect().center
        )

    def draw(self):
        frame = int(self.animation_state)
        if self.display_frames:
            for index, sol in enumerate(self.system.sols):
                origin = self.real_to_screen(sol.origin[:, frame])
                u = unit(-sol.angle[frame]) * 40 * self.scale / self.scale0 * self.zoom
                draw_arrow(self.surface, solid_color(index), origin, origin + u)
                draw_arrow(self.surface, solid_color(index), origin, origin - z_cross(u))
        self.sys_mgr.draw(self)
