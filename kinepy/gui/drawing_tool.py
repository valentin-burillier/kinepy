import pygame as pg
import numpy as np
from kinepy.math.geometry import rot, rvec


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


def prepare_rev(rev):
    return rev.s2.origin + rvec(rev.s2.angle, rev.p2)


def prepare_pri(pri):
    p11 = rvec(pri.a1 + pri.s1.angle, (min(pri.sliding), pri.d1))
    p12 = rvec(pri.a1 + pri.s1.angle, (max(pri.sliding), pri.d1))
    p2 = rvec(pri.s2.angle + pri.a2, (0., pri.d2))
    return pri.s1.origin + p11, pri.s1.origin + p12, pri.s2.origin + p2


def prepare_pin(pin):
    p11 = rvec(pin.a1 + pin.s1.angle, (min(pin.sliding), pin.d1))
    p12 = rvec(pin.a1 + pin.s1.angle, (max(pin.sliding), pin.d1))
    p2 = rvec(pin.s2.angle, pin.p2)
    return pin.s1.origin + p11, pin.s1.origin + p12, pin.s2.origin + p2


prepare = {
    1: prepare_rev,
    2: prepare_pri,
    3: prepare_pin
}


def prepare_joints(joints):
    return [prepare[j.id_](j) if j.id_ in prepare else () for j in joints]


def draw_rev(self, rev, data):
    pg.draw.circle(
        self.surface,
        self.background,
        self.real_to_screen(data[:, int(self.animation_state)]),
        REVOLUTE_RADIUS * self.scale / self.scale0 * self.zoom
    )
    pg.draw.circle(
        self.surface,
        solid_color(rev.s2.rep),
        self.real_to_screen(data[:, int(self.animation_state)]),
        REVOLUTE_RADIUS * self.scale / self.scale0 * self.zoom, 2
    )


def draw_pri(self, pri, data):
    p1, p2, p3 = data
    frame = int(self.animation_state)
    pg.draw.line(
        self.surface,
        solid_color(pri.s1.rep),
        self.real_to_screen(p1[:, frame]),
        self.real_to_screen(p2[:, frame]),
        2
    )
    rec = self.real_to_screen(p3[:, frame]) + np.einsum(
        'ik,lk->li',
        rot(-pri.s2.angle[frame] - pri.a2),
        PRISMATIC * self.scale / self.scale0 * self.zoom
    )
    pg.draw.polygon(
        self.surface,
        self.background,
        rec,
        0
    )
    pg.draw.polygon(
        self.surface,
        solid_color(pri.s2.rep),
        rec,
        2
    )


def draw_pin(self, pin, data):
    p1, p2, p3 = data
    frame = int(self.animation_state)
    pg.draw.line(
        self.surface,
        solid_color(pin.s1.rep),
        self.real_to_screen(p1[:, frame]),
        self.real_to_screen(p2[:, frame]),
        2
    )
    rec = self.real_to_screen(p3[:, frame]) + np.einsum(
        'ik,lk->li',
        rot(-pin.s2.angle[frame]),
        PIN_SLOT * self.scale / self.scale0 * self.zoom
    )
    pg.draw.polygon(
        self.surface,
        self.background,
        rec,
        0
    )
    pg.draw.polygon(
        self.surface,
        solid_color(pin.s2.rep),
        rec,
        2
    )


draw_joint = {
    1: draw_rev,
    2: draw_pri,
    3: draw_pin
}
