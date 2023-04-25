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
