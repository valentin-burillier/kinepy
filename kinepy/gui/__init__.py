import pygame as pg
from kinepy.interface.system import System


class GUI:
    running = True

    def __init__(self, system: System):
        self.system = system
        self.surface = pg.display.set_mode((640, 480), pg.RESIZABLE)

    def event_loop(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self.running = False

    def main_loop(self):
        while self.running:
            self.event_loop()


def display(system: System):
    pg.init()
    GUI(system).main_loop()
    pg.quit()
