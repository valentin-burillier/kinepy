import numpy as np


ARROW = np.array((
    # →
    (0, 10), (250., 10), (230, 40), (300., 0), (230, -40), (250., -10), (0, -10)
)) * (1, .5)

REVOLUTE_RADIUS = 8.

REVOLUTE = np.array((
    #  Mounting point        |
    #  |                     O
    (0, 2), *((np.sin(a), np.cos(a)) for a in np.linspace(0, 2 * np.pi, 16))
), float) * REVOLUTE_RADIUS
REVOLUTE_MOUNTING_POINT = 0

PRISMATIC = np.array((
    # ┌—┴—┐        Mounting point
    # └———┘             |
    (-2, 1), (0, 1), (0, 2), (0, 1), (2, 1), (2, -1), (-2, -1)
), float) * REVOLUTE_RADIUS
PRISMATIC_MOUNTING_POINT = 2

PIN_SLOT = np.array((
    # —►                                                         Mounting point
    (-3. ** .5, 1), (0., 0.), (-3. ** .5, -1), (-3. ** .5, 0), (-1 - 3. ** .5, 0), (-3. ** .5, 0)
), float)[:, ::-1] * REVOLUTE_RADIUS
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
), float) * REVOLUTE_RADIUS * (1, -1)

# CIRCLE_ARROW = np.concatenate((
#     2.12 * unit(np.linspace(0, 3 * np.pi / 2, 40)),
#     ((-0.2, 0.5, -0.2), (-2.4, -2., -1.6)),
#     1.88 * unit(np.linspace(3 * np.pi / 2, 0, 40))
# ), axis=1).swapaxes(0, 1) * REVOLUTE_RADIUS