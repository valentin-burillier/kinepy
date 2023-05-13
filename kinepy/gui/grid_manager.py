from kinepy.gui.camera import Camera
import kinepy.units as units
import numpy as np
import pygame as pg


# Wanted size of grid cells in pixels
GRID_CELL = 50
SCALES = np.log10((1, 2, 5, 10))


class GridManager:
    use_grid = use_graduation = False
    unit: float
    borders = 0, 0, 0, 0

    def find_unit(self, camera):
        log = np.log10(GRID_CELL / camera.zoom / camera.scale0 / units.SYSTEM[units.LENGTH])
        self.unit = 10 ** (min(SCALES, key=lambda x: abs(x - (log % 1))) + np.floor(log))
        # print(f'\r{self.unit:.1e} {units.get_unit(units.LENGTH)}', end='')

    def camera_borders(self, camera):
        start_x = camera.camera_area.x / units.SYSTEM[units.LENGTH] // self.unit
        start_y = camera.camera_area.y / units.SYSTEM[units.LENGTH] // self.unit
        end_x = (camera.camera_area.x + camera.camera_area.w) / units.SYSTEM[units.LENGTH] // self.unit + 1
        end_y = (camera.camera_area.y + camera.camera_area.h) / units.SYSTEM[units.LENGTH] // self.unit + 1
        self.borders = tuple(int(x) for x in (start_x + 1, start_y + 1, end_x, end_y))

    def draw_grid(self, camera, color):
        if not self.use_grid:
            return
        camera_center = camera.camera_area.center
        for x in range(self.borders[0], self.borders[2]):
            line_x = (x * self.unit * units.SYSTEM[units.LENGTH] - camera_center[0]) * camera.scale * camera.zoom + camera.surface.get_rect().centerx
            pg.draw.line(camera.surface, color, (line_x, 0), (line_x, camera.surface.get_height()), 1)
        for y in range(self.borders[1], self.borders[3]):
            line_y = -(y * self.unit * units.SYSTEM[units.LENGTH] - camera_center[1]) * camera.scale * camera.zoom + camera.surface.get_rect().centery
            pg.draw.line(camera.surface, color, (0, line_y), (camera.surface.get_width(), line_y), 1)

    def draw_graduations(self, surface, camera, color, camera_pos):
        if not self.use_graduation:
            return
        r, h = camera.surface.get_rect(topleft=camera_pos), surface.get_height() - 50
        pg.draw.rect(surface, color, r, 2)
        camera_center = camera.camera_area.center

        font = pg.font.SysFont('arial', int(h / 40))

        rounded_unit = float(f'{self.unit:.0e}')
        unit = self.unit * units.SYSTEM[units.LENGTH]

        # Horizontal graduations
        for x in range(self.borders[0], self.borders[2]):
            # screen x position of the line
            line_x = r.left + (x * unit - camera_center[0]) * camera.scale * camera.zoom + r.w / 2
            pg.draw.line(surface, color, (line_x, r.bottom), (line_x, r.bottom + h * 0.02), 2)

            text = font.render(f'{str(rounded_unit * x).rstrip("0").rstrip(".")}', True, color)
            surface.blit(text, text.get_rect(midtop=(line_x, r.bottom + h * 0.02)))

        # vertical graduations
        for y in range(self.borders[1], self.borders[3]):
            # screen y position of the line
            line_y = r.top - (y * unit - camera_center[1]) * camera.scale * camera.zoom + r.h / 2
            pg.draw.line(surface, color, (r.left, line_y), (r.left - surface.get_width() * 0.02, line_y), 2)

            text = font.render(f'{str(rounded_unit * y).rstrip("0").rstrip(".")} ', True, color)
            surface.blit(text, text.get_rect(midright=(r.left - surface.get_width() * 0.02, line_y)))

    def change_scale(self, camera):
        self.find_unit(camera)
        self.camera_borders(camera)

    def move_camera(self, camera):
        self.camera_borders(camera)

    def set_scale(self, camera):
        self.find_unit(camera)
        self.camera_borders(camera)
