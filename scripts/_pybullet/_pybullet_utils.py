import pybullet as p
import time
import pybullet_data
import os
import numpy as np

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

def visualize_trajectory(self):
    for jj in range(len(self.xy_traj)):  # TODO: PYBULLET set joint positions
        p.resetJointState(self.robotId, self.joint_indices[0], self.xy_traj[jj][0])
        p.resetJointState(self.robotId, self.joint_indices[1], self.xy_traj[jj][1])
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