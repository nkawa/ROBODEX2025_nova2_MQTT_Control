from collections import deque

import numpy as np


class SMAFilter:
    def __init__(self, n_windows: int = 10) -> None:
        self.n_windows = n_windows

    def reset(self, data: float | np.ndarray) -> None:
        self.previous_measurements = deque([data] * self.n_windows)
        self.previous_filtered_measurement = data

    def filter(self, new_measurement: float | np.ndarray) -> float | np.ndarray:
        n_windows_previous_measurement = \
            self.previous_measurements.popleft()
        self.previous_measurements.append(new_measurement)
        self.previous_filtered_measurement = (
            self.previous_filtered_measurement
            - n_windows_previous_measurement / self.n_windows
            + new_measurement / self.n_windows
        )
        return self.previous_filtered_measurement

    def predict_only(self, new_measurement: float | np.ndarray) -> float | np.ndarray:
        n_windows_previous_measurement = \
            self.previous_measurements[0]
        return (
            self.previous_filtered_measurement
            - n_windows_previous_measurement / self.n_windows
            + new_measurement / self.n_windows
        )


class ButterworthFilter:
    def __init__(self, low_pass_filter_coeff: float = 1.5):
        self.scale_term = 1 / (1 + low_pass_filter_coeff)
        self.feedback_term = 1 - low_pass_filter_coeff

    def reset(self, data: float | np.ndarray) -> None:
        self.previous_measurements = [data, data]
        self.previous_filtered_measurement = data
    
    def filter(self, new_measurement: float | np.ndarray) -> float | np.ndarray:
        self.previous_measurements[1] = self.previous_measurements[0]
        self.previous_measurements[0] = new_measurement
        self.previous_filtered_measurement = self.scale_term * (
            self.previous_measurements[1]
            + self.previous_measurements[0]
            - self.feedback_term * self.previous_filtered_measurement
        )
        return self.previous_filtered_measurement
