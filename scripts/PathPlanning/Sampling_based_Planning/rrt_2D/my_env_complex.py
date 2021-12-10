"""
Environment for rrt_2D
@author: huiming zhou
"""


class MyEnv:
    def __init__(self):
        self.x_range = (-78, 178)
        self.y_range = (-78, 178)
        self.obs_boundary = self.obs_boundary()
        self.obs_rectangle = self.obs_rectangle()
        self.obs_circle = self.obs_circle()

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [-75, -75, 1, 250],
            [-75, 175, 250, 1],
            [-74, -75, 250, 1],
            [175, -75, 1, 250]
        ]
        return obs_boundary


    @staticmethod
    def obs_rectangle():
        obs_rectangle = [
            [100, 20, 20, 20]
        ]
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [30, 25, 10],
            [60, 70, 10],
            [10, 90, 10]
        ]

        return obs_cir
