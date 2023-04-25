import pygame as pg
import numpy as np
from kinepy.math.geometry import rot
import os

PATH = os.path.dirname(__file__)

ARROW1 = np.array(pg.surfarray.array3d(pg.image.load(os.path.join(PATH, 'arrow2.png'))))
ARROW2 = np.array((
    (0., 0.), (0, 10), (250., 10), (230, 40), (300., 0), (230, -40), (250., -10), (0, -10)
)) * (1, .5)


def draw_arrow1(surface, color, start, end):
    vec = np.array(end) - np.array(start)
    magnitude = np.sum(vec ** 2) ** .5
    if magnitude == 0:
        return
    angle = np.arccos(vec[0] / magnitude) * (-1, 1)[vec[1] < 0]
    arrow_start = .5 * np.array((-magnitude * np.cos(angle), magnitude * np.sin(angle)))
    arrow = pg.transform.scale_by(pg.surfarray.make_surface(ARROW1 * color), magnitude / ARROW1.shape[0])
    arrow = pg.transform.rotate(arrow, angle * 180 / np.pi)
    arrow.set_colorkey((0, 0, 0))
    surface.blit(arrow, start - (arrow.get_rect().center + arrow_start))


def draw_arrow2(surface, color, start, end):
    vec = np.array(end) - np.array(start)
    magnitude = np.sum(vec ** 2) ** .5
    if magnitude == 0:
        return
    angle = np.arccos(vec[0] / magnitude) * (1, -1)[vec[1] < 0]
    polygon = np.einsum('ik,lk->li', rot(angle), ARROW2 * magnitude / 300) + start
    pg.draw.polygon(surface, color, polygon, 0)
