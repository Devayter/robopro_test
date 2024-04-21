from typing import List

import numpy as np

from constants import TRANSFORMATION_MATRIX_SIZE


class DHKinematics:
    """
    Модель прямой кинематики на основе параметров DH.
    """
    def __init__(self, joints_params: List[List[float]]):
        self.joints = [Joint(params) for params in joints_params]

    def forward_kinematics(self):
        """
        Метод получения общей матрицы преобразования T.
        """
        T = np.eye(TRANSFORMATION_MATRIX_SIZE)
        for joint in self.joints:
            T = np.dot(T, joint.dh_matrix())
        return T


class Joint:
    """
    Модель сочленения.
    """
    def __init__(self, params: List[float]):
        self.a, self.alpha, self.d, self.theta = params

    def dh_matrix(self):
        """
        Метод генерация матрицы преобразования Денавита-Хартенберга для 
        соответствующего сочленения.
        """
        return np.array([
            [
                np.cos(self.theta),
                -np.cos(self.alpha) * np.sin(self.theta),
                np.sin(self.alpha) * np.sin(self.theta),
                self.a * np.cos(self.theta)
            ],
            [
                np.sin(self.theta),
                np.cos(self.alpha) * np.cos(self.theta),
                -np.sin(self.alpha) * np.cos(self.theta),
                self.a * np.sin(self.theta)
            ],
            [0, np.sin(self.alpha), np.cos(self.alpha), self.d],
            [0, 0, 0, 1]
        ])
