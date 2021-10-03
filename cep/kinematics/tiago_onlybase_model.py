from cep.kinematics.robot_model import Robot
import os
import pybullet as p

base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
robot_dir = os.path.join(base_dir, 'robots/tiago/')
urdf_filename = os.path.join(robot_dir, 'tiago_only_base.urdf')

link_names = ['X', 'Y']

class TiagoRobotOnlyBase(Robot): # TODO: Done
    def __init__(self):
        super(TiagoRobotOnlyBase, self).__init__(urdf_file=urdf_filename, link_name_list=link_names)
        # check if it's successful loading
        p.connect(p.GUI)
        robot = p.loadURDF(urdf_filename)
        num_joints = p.getNumJoints(robot)
        for i in range(num_joints):
            print(p.getJointInfo(robot, i))
        while True:
            p.stepSimulation()

if __name__ == '__main__':
    tiago_kin = TiagoRobotOnlyBase()