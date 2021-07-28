import random
import pybullet as p

class set_start:

    def __init__(self, set_random_start=True, set_known_start=False, start_point=None):
        self.dim_xy = 2
        self.dim = 3
        self.robotId = 0
        self.joint_indices = None
        self.set_random_start = set_random_start
        self.set_known_start = set_known_start
        self.start_point = start_point

    def gen_random_start_point(self) -> list:  # TODO: generate random start points
        '''
            Generate a random start state in Pybullet, this start point should not collide with the object
        '''
        random_start_point = [0] * self.dim_xy  # x, y, z

        random_x = 0.5
        random_y = 0.5
        while 0.35 < random_x < 0.65 and 0.35 < random_y < 0.65:
            for i in range(self.dim):
                random_x = random.uniform(-1.5, 1.5)
                random_y = random.uniform(-1.5, 1.5)

        random_start_point[0] = random_x
        random_start_point[1] = random_y

        return random_start_point


    def set_random_start_point(self, random_start_point):
        '''
            Set a random start state in Pybullet
        '''
        for i in range(self.dim_xy):
            p.resetJointState(self.robotId, self.joint_indices[i], random_start_point[i])


    def set_known_start_point(self):
        '''
            Set a known start state in Pybullet
        '''
        known_start_point = [0.2, 0.2, 0]
        # ic(known_start_point)
        for i in range(self.dim_xy):
            p.resetJointState(self.robotId, self.joint_indices[i], known_start_point[i])


    def set_start_points(self, start_point: list):
        '''
            Set the start state in Pybullet
        '''
        # ic(start_point)
        for i in range(self.dim_xy):
            p.resetJointState(self.robotId, self.joint_indices[i], start_point[i])

    def set(self):
        if self.set_random_start:
            random_start_point = self.gen_random_start_point()
            self.set_random_start_point(random_start_point)
        elif self.set_random_start:
            self.set_known_start_point()
        else:
            self.set_start_points(self.start_point)