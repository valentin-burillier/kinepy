import pygame as pg
import numpy as np
from kinepy.math.geometry import rot, rvec
import os

PATH = os.path.dirname(__file__)

ARROW1 = np.array(pg.surfarray.array3d(pg.image.load(os.path.join(PATH, 'arrow2.png'))))
ARROW2 = np.array((
    (0, 10), (250., 10), (230, 40), (300., 0), (230, -40), (250., -10), (0, -10)
)) * (1, .5)

REVOLUTE_RADIUS = 8.
PRISMATIC = np.array((
    (-2 * REVOLUTE_RADIUS, REVOLUTE_RADIUS), (2 * REVOLUTE_RADIUS, REVOLUTE_RADIUS),
    (2 * REVOLUTE_RADIUS, -REVOLUTE_RADIUS), (-2 * REVOLUTE_RADIUS, -REVOLUTE_RADIUS)
))
PIN_SLOT = np.array((
    (-3. ** .5 * REVOLUTE_RADIUS, REVOLUTE_RADIUS), (0., 0.), (-3. ** .5 * REVOLUTE_RADIUS, -REVOLUTE_RADIUS)
))


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


SOLID_COLORS = (
    (225, 105, 97), (255, 180, 128), (248, 243, 141), (66, 214, 164), (8, 202, 209), (89, 173, 246), (157, 148, 255),
    (199, 128, 232)
)


def draw_rev(self, rev, data):
    pg.draw.circle(
        self.surface,
        self.background,
        self.real_to_screen(data[:, int(self.animation_state)]),
        REVOLUTE_RADIUS * self.scale / self.scale0 * self.zoom
    )
    pg.draw.circle(
        self.surface,
        SOLID_COLORS[rev.s2.rep % len(SOLID_COLORS)],
        self.real_to_screen(data[:, int(self.animation_state)]),
        REVOLUTE_RADIUS * self.scale / self.scale0 * self.zoom, 2
    )


def draw_pri(self, pri, data):
    p1, p2, p3 = data
    frame = int(self.animation_state)
    pg.draw.line(
        self.surface,
        SOLID_COLORS[pri.s1.rep % len(SOLID_COLORS)],
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
        SOLID_COLORS[pri.s2.rep % len(SOLID_COLORS)],
        rec,
        2
    )


def draw_pin(self, pin, data):
    p1, p2, p3 = data
    frame = int(self.animation_state)
    pg.draw.line(
        self.surface,
        SOLID_COLORS[pin.s1.rep % len(SOLID_COLORS)],
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
        SOLID_COLORS[pin.s2.rep % len(SOLID_COLORS)],
        rec,
        2
    )


draw_joint = {
    1: draw_rev,
    2: draw_pri,
    3: draw_pin
}
