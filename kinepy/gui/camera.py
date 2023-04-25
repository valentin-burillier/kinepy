import pygame as pg
import numpy as np


class Camera:
    scale = scale0 = zoom = 1.
    surface: pg.Surface

    def __init__(self):
        self.system_area = self.camera_area = pg.Rect(0, 0, 0, 0)

    def set_bound_box(self, points: tuple):
        min_ = np.amin(points, axis=(0, 2))
        max_ = np.amax(points, axis=(0, 2))
        self.system_area = pg.Rect(*min_, *(max_ - min_))
        self.system_area.scale_by_ip(1.05, 1.05)

        if self.system_area.w == 0. or self.system_area.h == 0:
            raise ValueError("System does not use any area")

    def set_scale(self):
        self.scale = min(self.surface.get_size() / np.array((self.system_area.w, self.system_area.h)))
        old_camera = self.camera_area.center
        self.camera_area = pg.Rect(0, 0, *(self.surface.get_size() / np.array(self.scale)))
        self.camera_area.scale_by_ip(1 / self.zoom, 1 / self.zoom)
        self.camera_area.center = old_camera
        self.camera_area.clamp_ip(self.system_area)

    def manage(self, event):
        # mouse scroll to zoom
        if event.type == pg.MOUSEBUTTONDOWN and event.button in (pg.BUTTON_WHEELUP, pg.BUTTON_WHEELDOWN):
            old_zoom = self.zoom
            dp = (1., -1.) * (event.pos - .5 * np.array(self.surface.get_size())) / self.zoom / self.scale
            self.zoom = max(1., min(10., self.zoom * 1.05 ** (2 * (event.button == pg.BUTTON_WHEELUP) - 1)))
            self.camera_area.scale_by_ip(old_zoom / self.zoom, old_zoom / self.zoom)
            self.camera_area.move_ip(dp * (1 - old_zoom / self.zoom))
            self.camera_area.clamp_ip(self.system_area)

        # click and move to navigate
        elif event.type == pg.MOUSEMOTION and event.buttons == (1, 0, 0):
            self.camera_area.move_ip(event.rel * np.array((1, -1)) / self.scale)
            self.camera_area.clamp_ip(self.system_area)

        elif event.type == pg.WINDOWRESIZED:
            self.set_scale()
