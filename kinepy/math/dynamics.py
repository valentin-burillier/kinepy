from kinepy.math.geometry import *
from kinepy.objects.config import Config


class Newtons2ndLaw:
    _ground_is_not_a_free_body = "Clearly, zero does not belong here, yet there it is anyway !?"

    @staticmethod
    def force(config: Config, eq: tuple[int, ...]) -> np.ndarray:
        # -(sum(known_forces) - m.a) = sum(unknown_forces)
        assert 0 not in eq, Newtons2ndLaw._ground_is_not_a_free_body
        return -np.sum(config.results.solid_dynamics[eq, 0:2, :], axis=0)

    @staticmethod
    def torque(config: Config, eq: tuple[int, ...], point: np.ndarray) -> np.ndarray:
        # -(sum(known_torques(g) - J.aa + pg x (sum(known_forces) - m.a)) = sum(unknown_torques(p))
        assert 0 not in eq, Newtons2ndLaw._ground_is_not_a_free_body
        # babar
        # np.cross: shape ((eq, 2, n) - (1, 2, n)) x (eq, 2, n) -> (eq, n)
        return -np.sum(config.results.solid_dynamics[eq, 4, :] + np.cross(config.results.solid_dynamics[eq, 2:4, :] - point[np.newaxis, ...], config.results.solid_dynamics[eq, 0:2, :], axis=1), axis=0)

    @staticmethod
    def select_group(all_eqs: tuple[tuple[int, ...], ...], target_indices: tuple[int], ground_eq: int) -> tuple[tuple[int, ...], float]:
        target_mask = sum(1 << eq_index for eq_index in target_indices)
        sign = 1.0
        if (target_mask >> ground_eq) & 1:
            target_mask ^= (1 << len(all_eqs)) - 1
            sign = -1.0
        return sum((eq for i, eq in enumerate(all_eqs) if (i >> target_mask) & 1), ()), sign


class JointInput:
    @staticmethod
    def solve_revolute(config: Config, s1: int, s2: int, joint: int, eq1: tuple[int, ...], eq2: tuple[int, ...], zero_holder: int):
        pass
