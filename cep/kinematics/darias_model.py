from cep.kinematics.robot_model import Robot
import os

base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
robot_dir = os.path.join(base_dir, 'robots/darias_description/robots')
urdf_filename = os.path.join(robot_dir, 'darias_clean.urdf')

link_names = ['R_1_link', 'R_2_link', 'R_3_link', 'R_4_link','R_5_link','R_6_link', 'R_endeffector_link']

class DarIASArm(Robot):
    def __init__(self):
        super(DarIASArm, self).__init__(urdf_file=urdf_filename, link_name_list=link_names)


if __name__ == '__main__':
    dar_kin = DarIASArm()



