from kinepy.gui.camera import Camera
import numpy as np
import pygame as pg


# Wanted size of grid cells in pixels
GRID_CELL = 40
SCALES = np.log10((1, 2, 5, 10))


class GridManager(Camera):
    unit: float
    borders = 0, 0, 0, 0

    def find_unit(self):
        log = np.log10(GRID_CELL / self.zoom / self.scale)
        self.unit = 10 ** (min(SCALES, key=lambda x: (x - (log % 1)) ** 2) + np.floor(log))
        print(f'\r{self.unit}', end='')

    def camera_borders(self):
        start_x = self.camera_area.x // self.unit
        start_y = self.camera_area.y // self.unit
        end_x = (self.camera_area.x + self.camera_area.w) // self.unit + 1
        end_y = (self.camera_area.y + self.camera_area.h) // self.unit + 1
        self.borders = tuple(int(x) for x in (start_x, start_y, end_x, end_y))

    def draw_grid(self, color):
        camera_center = self.camera_area.center
        for x in range(self.borders[0], self.borders[2]):
            line_x = (x * self.unit - camera_center[0]) * self.scale * self.zoom + self.surface.get_rect().centerx
            pg.draw.line(self.surface, color, (line_x, 0), (line_x, self.surface.get_height()), 1)
        for y in range(self.borders[1], self.borders[3]):
            line_y = -(y * self.unit - camera_center[1]) * self.scale * self.zoom + self.surface.get_rect().centery
            pg.draw.line(self.surface, color, (0, line_y), (self.surface.get_width(), line_y), 1)

    def change_scale(self, event):
        Camera.change_scale(self, event)
        self.find_unit()
        self.camera_borders()

    def move_camera(self, event):
        Camera.move_camera(self, event)
        self.camera_borders()

    def set_scale(self):
        Camera.set_scale(self)
        self.find_unit()
        self.camera_borders()
