"""
Environment for rrt_2D
@author: huiming zhou
"""


class MyEnv:
    def __init__(self):
        self.x_range = (-100, 200)
        self.y_range = (-100, 200)
        self.obs_boundary = self.obs_boundary()
        self.obs_rectangle = self.obs_rectangle()

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
            [35, 35, 30, 30]
        ]
        return obs_rectangle
