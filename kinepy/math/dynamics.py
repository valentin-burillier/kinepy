import numpy as np

from kinepy.math.geometry import *
from kinepy.objects.config import Config


class Newtons2ndLaw:
    @staticmethod
    def force(config: Config, eq: tuple[int, ...]) -> np.ndarray:
        # -(sum(known_forces) - m.a) = sum(unknown_forces)
        return -np.sum(config.results.solid_dynamics[eq, 0:2, :], axis=0)

    @staticmethod
    def torque(config: Config, eq: tuple[int, ...], point: np.ndarray) -> np.ndarray:
        # babar
        # -(sum(known_torques(g) - J.aa + pg x (sum(known_forces) - m.a)) = sum(unknown_torques(p))
        # np.cross: shape ((eq, 2, n) - (1, 2, n)) x (eq, 2, n) -> (eq, n)
        return -np.sum(config.results.solid_dynamics[eq, 4, :] + np.cross(config.results.solid_dynamics[eq, 2:4, :] - point[np.newaxis, ...], config.results.solid_dynamics[eq, 0:2, :], axis=1), axis=0)


class JointInput:
    @staticmethod
    def solve_revolute(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        pass