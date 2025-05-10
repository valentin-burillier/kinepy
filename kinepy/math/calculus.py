import numpy as np


class Derivation:
    @staticmethod
    def derivative(arr: np.ndarray, axis, frame_time):
        return 0.5 * np.diff(arr, axis=axis, prepend=float('NaN')) + np.diff(arr, axis=axis, append=float('NaN')) / frame_time

    @staticmethod
    def second_derivative(arr: np.ndarray, axis, frame_time):
        return np.diff(arr, 2, axis=axis, prepend=float('NaN'), append=float('NaN')) / frame_time / frame_time
