import pybullet as p
import time
import pybullet_data
import os
import numpy as np
import random
import math

class start_bullet_env:

    def __init__(self, set_random_start=False, set_known_start=True, start_point=None):
        self.dim_xy = 2
        self.dim = 3
        self.robotId = 0
        self.joint_indices = None
        self.set_random_start = set_random_start
        self.set_known_start = set_known_start
        self.start_point = start_point
        self.joint_indices = [0, 1]
        self.end_point = [1.2, 1.0]

        self.joint_indices = [0, 1]
        self.trajectory = None
        self.xy_traj = None
        self.robotId = None
        self.cascade_threshold = 0.2  # TODO: Set different threshold to check the performance
        self.k_d = 1.
        self.delta = 0.5  # TODO: Change here to decide the step!!!
        self.dim = 3
        self.dim_xy = 2
        self.num_path_points = 0
        self.cascade_control_path = []
        self.whole_cascade_control_path = []
        self.end_point = [1.2, 1.0]
        self.end_point_threshold = 0.02

        self.velocity_matrix = None  # TODO: add on 07.21
        self.grid_number = 301
        self.grid_step = 0.01
        self.grid_step_reciprocal = 1 / self.grid_step
        self.grid_map_min = -1.5
        self.grid_map_max = 1.5

        self.round_whole_paths = None

        self.box_position = [0.5, 0.5]
        self.repulsive_threshold = 0.5  # TODO: Think about which value is better. Add on 07.24
        self.repulsive_scale = 5.  # TODO: Think about which value is better. Add on 07.24

    def start_pybullet(self, activate_GUI=False) -> (int, list):  # load Tiago in Pybullet

        joint_indices = self.joint_indices

        if activate_GUI:
            physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
        else:
            physicsClient = p.connect(p.DIRECT)  #

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
        p.setGravity(0, 0, -9.81)

        base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
        robot_dir = os.path.join(base_dir, 'robots/tiago/')
        urdf_filename = os.path.join(robot_dir, 'tiago_single_with_holoBaseCXY.urdf')

        planeId = p.loadURDF("plane.urdf")
        startPos = [0., 0., 0.]
        startOrientation = [0., 0., 0.]
        self.robotId = p.loadURDF(urdf_filename, startPos, p.getQuaternionFromEuler([0., 0., 0.]), useFixedBase=1)
        # p.loadURDF("cube_small.urdf", np.array([0.4, -0.5, 0.5]),
        #            p.getQuaternionFromEuler([0, 0, 0]),
        #            useFixedBase=True, useMaximalCoordinates=True)
        p.loadURDF('cube_small.urdf', np.array([0.5, 0.5, 0.15]),  # TODO: Put an object in target postion
                   p.getQuaternionFromEuler([0, 0, 0]), globalScaling=6,
                   useFixedBase=True)

        # Num_Joints = p.getNumJoints(self.robotId)
        # # print('p.getNumJoints(robotId)', p.getNumJoints(robotId))
        # # for i in range(Num_Joints):
        # #     print('p.getJointState(robotId, {})'.format(i), p.getJointState(robotId, i))
        # for i in range(Num_Joints):
        # 	print('p.getJointInfo(robotId, {})'.format(i), p.getJointInfo(self.robotId, i))

        self.set()

        robotId = self.robotId

        return robotId, joint_indices

    def visualize_trajectory(self, xy_traj):

        for jj in range(len(xy_traj)):  # TODO: PYBULLET set joint positions
            p.resetJointState(self.robotId, self.joint_indices[0], xy_traj[jj][0])
            p.resetJointState(self.robotId, self.joint_indices[1], xy_traj[jj][1])
            time.sleep(0.1)

            def start_pybullet(self) -> (int, list):  # load Tiago in Pybullet
                if self.activate_GUI:
                    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
                else:
                    physicsClient = p.connect(p.DIRECT)  #
                p.setAdditionalSearchPath(pybullet_data.getDataPath())
                p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
                p.setGravity(0, 0, -9.81)

                base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
                robot_dir = os.path.join(base_dir, 'robots/tiago/')
                urdf_filename = os.path.join(robot_dir, 'tiago_single_with_holoBaseCXY.urdf')

                planeId = p.loadURDF("plane.urdf")
                startPos = [0., 0., 0.]
                startOrientation = [0., 0., 0.]
                self.robotId = p.loadURDF(urdf_filename, startPos, p.getQuaternionFromEuler([0., 0., 0.]), useFixedBase=1)
                # p.loadURDF("cube_small.urdf", np.array([0.4, -0.5, 0.5]),
                #            p.getQuaternionFromEuler([0, 0, 0]),
                #            useFixedBase=True, useMaximalCoordinates=True)
                p.loadURDF('cube_small.urdf', np.array([0.5, 0.5, 0.15]),  # TODO: Put an object in target postion
                           p.getQuaternionFromEuler([0, 0, 0]), globalScaling=6,
                           useFixedBase=True)

                # Num_Joints = p.getNumJoints(self.robotId)
                # # print('p.getNumJoints(robotId)', p.getNumJoints(robotId))
                # # for i in range(Num_Joints):
                # #     print('p.getJointState(robotId, {})'.format(i), p.getJointState(robotId, i))
                # for i in range(Num_Joints):
                # 	print('p.getJointInfo(robotId, {})'.format(i), p.getJointInfo(self.robotId, i))

                time.sleep(0.5)

                return self.robotId, self.joint_indices

    def get_robot_current_state(self) -> [list, list]:
        '''
            return current position and velocity
        '''
        joint_state = p.getJointStates(self.robotId, self.joint_indices)
        # ic(joint_state)

        joint_positions = [joint_state[i][0] for i in range(len(self.joint_indices))]
        # ic(joint_positions)
        # ic(type(joint_positions))
        # ic(joint_positions[0])
        # ic(len(joint_positions))

        joint_velocities = [joint_state[j][1] for j in range(len(self.joint_indices))]
        # ic(joint_velocities)
        # ic(type(joint_velocities))

        current_state = [joint_positions, joint_velocities]

        return current_state

    def start_baseline_resetJointState(self, next_position):
        '''
            In pybullet reset the joint state, joint X and Y
        '''
        for jj in range(len(next_position)):  # TODO: PYBULLET set joint positions
            p.resetJointState(self.robotId, self.joint_indices[0], next_position[0])
            p.resetJointState(self.robotId, self.joint_indices[1], next_position[1])
            time.sleep(0.1)

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
        known_start_point = [0.2, 0.2]
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
        elif self.set_known_start:
            self.set_known_start_point()
        else:
            self.set_start_points(self.start_point)

    def get_x_y_traj(self, trajectory) -> list:
        '''
            Get the 2d trajectory which is generated by RRTstar in MoveIt
        '''
        length = len(trajectory)
        xy_traj = [[0, 0] for i in range(length)]
        for i in range(length):
            xy_traj[i] = trajectory[i][:2]

        return xy_traj

    def check_arrive(self) -> bool:
        '''
            According to self.end_point_threshold=0.01 -> check the robot arrive at the target position or not
        '''
        curren_state = self.get_robot_current_state()
        curren_position = curren_state[0]
        cur_dist = self.compute_euclidean_distance(self.end_point, curren_position)

        if cur_dist < self.end_point_threshold:
            return True

    def compute_euclidean_distance(self, first_point: list, second_point: list) -> float:
        '''
            return: the euclidean distance between two points
        '''

        assert len(first_point) == len(second_point), "ERROR: Two points have different dimension!!!"

        dist = 0
        for i in range(len(first_point)):
            dist += pow((first_point[i] - second_point[i]), 2)

        return math.sqrt(dist)