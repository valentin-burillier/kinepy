import pygame as pg
import numpy as np
from kinepy.math.geometry import rot, unit


# --------------------------------------------- Predefined shapes ------------------------------------------------------

ARROW = np.array((
    # →
    (0, 10), (250., 10), (230, 40), (300., 0), (230, -40), (250., -10), (0, -10)
)) * (1, .5)

REVOLUTE_RADIUS = 8.

REVOLUTE = np.array((
    #       Mounting point
    (0, 0), (2, 0)
), float) * REVOLUTE_RADIUS
REVOLUTE_MOUNTING_POINT = 1

PRISMATIC = np.array((
    # ┌—┴—┐        Mounting point
    # └———┘             |
    (-2, 1), (0, 1), (0, 2), (0, 1), (2, 1), (2, -1), (-2, -1)
), float) * REVOLUTE_RADIUS
PRISMATIC_MOUNTING_POINT = 2

PIN_SLOT = np.array((
    # —►                                                         Mounting point
    (-3. ** .5, 1), (0., 0.), (-3. ** .5, -1), (-3. ** .5, 0), (-1 - 3. ** .5, 0), (-3. ** .5, 0)
), float) * REVOLUTE_RADIUS
PIN_SLOT_MOUNTING_POINT = 4

# point pairs for lines
GROUND = np.array((
    #  ——┴——
    #  /////
    (0, 0), (0, -1),
    (-1, -1), (1, -1),
    (-1.5, -2), (-1, -1),
    (-1, -2), (-.5, -1),
    (-.5, -2), (0., -1),
    (0, -2), (.5, -1),
    (.5, -2), (1, -1)
), float) * REVOLUTE_RADIUS

CIRCLE_ARROW = np.concatenate((
    2.12 * unit(np.linspace(0, 3 * np.pi / 2, 40)),
    ((-0.2, 0.5, -0.2), (-2.4, -2., -1.6)),
    1.88 * unit(np.linspace(3 * np.pi / 2, 0, 40))
), axis=1).swapaxes(0, 1) * REVOLUTE_RADIUS

# -------------------------------------------------- Colors ------------------------------------------------------------

COLORMAP = (
    (144, 144, 144), (61, 131, 198), (204, 0, 0), (106, 167, 79), (241, 194, 57), (227, 119, 194), (255, 127, 14),
    (148, 103, 189), (145, 220, 3), (26, 190, 207)
)


def solid_color(index):
    return COLORMAP[(index - 1) % (len(COLORMAP) - 1) + 1] if index else COLORMAP[0]

# ----------------------------------------------------------------------------------------------------------------------


def draw_arrow(surface, color, start, end):
    vec = np.array(end) - np.array(start)
    magnitude = np.sum(vec ** 2) ** .5
    if magnitude == 0:
        return
    angle = np.arccos(vec[0] / magnitude) * (1, -1)[vec[1] < 0]
    polygon = np.einsum('ik,lk->li', rot(angle), ARROW * magnitude / 300) + start
    pg.draw.polygon(surface, color, polygon, 0)

