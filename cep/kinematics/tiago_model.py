from cep.kinematics.robot_model import Robot
import os
import pybullet as p

base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
robot_dir = os.path.join(base_dir, 'robots/tiago/')
urdf_filename = os.path.join(robot_dir, 'tiago_single_modified.urdf')

link_names = ['arm_1_link', 'arm_2_link', 'arm_3_link', 'arm_4_link', 'arm_5_link', 'arm_6_link', 'arm_7_link']

class TiagoRobot(Robot): # TODO: Done
    def __init__(self):
        super(TiagoRobot, self).__init__(urdf_file=urdf_filename, link_name_list=link_names)
        #check if it's successful loading
        # p.connect(p.GUI)
        # robot = p.loadURDF(urdf_filename)
        # num_joints = p.getNumJoints(robot)
        # for i in range(num_joints):
        #     print(p.getJointInfo(robot, i))
        # while True:
        #     p.stepSimulation()

if __name__ == '__main__':
    tiago_kin = TiagoRobot()