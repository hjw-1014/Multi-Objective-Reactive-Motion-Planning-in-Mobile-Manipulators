from utils import *
from _pybullet import *
from _plot import *
import pybullet as p

if __name__ == "__main__":
    # TODO: Read json trajectory
    traj, num_path_points = open_json('qtrjs.json')
    xy_traj = get_x_y_traj(traj)

    # TODO: tiago environment
    tiago_env = start_bullet_env(set_random_start=True, set_known_start=False, start_point=None)
    robotId, joint_indexes = tiago_env.start_pybullet(activate_GUI=False) #TODO: Change here to start the GUI

    plot_moveit_traj(xy_traj)
    while True:
        tiago_env.visualize_trajectory()
        p.stepSimulation()

