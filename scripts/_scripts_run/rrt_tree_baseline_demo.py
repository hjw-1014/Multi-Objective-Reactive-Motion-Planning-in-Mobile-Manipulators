import numpy as np
import pybullet as p
import time
from utils import *
from _pybullet import start_bullet_env
from _controller import *
from _compute import *
from _plot import *
# sys.path.append("/home/ias/Documents/Jiawei/comp_energy_policy-Tiago/comp_energy_policy-main/scripts/PathPlanning/Sampling_based_Planning/rrt_2D/")
# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + \
#                 'PathPlanning/Sampling_based_Planning/rrt_2d')

class Node:
    def __init__(self, n: list):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


if __name__ == "__main__":
    # TODO: Read json trajectory
    traj, num_path_points = open_json('qtrjs.json')
    xy_traj = get_x_y_traj(traj)

    rrt_path = load_rrt_path('path_rrt.npy') / 100  # numpy.ndarray
    rrt_path = np.around(rrt_path, 3)
    graph_rrt = load_rrt_nodes_list("graph_rrt.npy") / 100  # numpy.ndarray(np.array)
    graph_rrt = np.around(graph_rrt, 3)
    #rrt_vertex = load_rrt_vertex("vertex_rrt.npy")

    graph_rrt_son, graph_rrt_father = split_son_father(graph_rrt)

    # TODO: tiago environment
    tiago_env = start_bullet_env(set_random_start=True, set_known_start=False, start_point=None)
    robotId, joint_indexes = tiago_env.start_pybullet(activate_GUI=True) #TODO: Change here to start the GUI

    # TODO: Define parameters
    cascade_control_path = []
    end_point = [1.2, 1.0]
    end_point_threshold = 0.02
    delta = 0.1  # TODO: Change here to decide the step!!!
    iteration = 1

    start_time = time.time()
    while True:
        next_position = [0] * 2
        ic(iteration)
        iteration += 1

        current_state = tiago_env.get_robot_current_state()
        cur_position = current_state[0]

        # TODO: Check if arrive the target point
        # Calculate the distance between end point and current point
        cur_dist = compute_euclidean_distance(end_point, cur_position)
        ic(cur_position)
        ic(cur_dist)
        if cur_dist < end_point_threshold:
            ic(cur_position)
            print("######### End point arrived!!! #########")
            print("--- %s seconds!!! ---" % (time.time() - start_time))
            plot_cascade_control_traj(cascade_control_path, rrt_path, len(rrt_path))  # TODO: Plot generated path

            break

        # Based on the current state, calculate the velocity command

        dx = cascade_control_rrt_tree(tiago_env, current_state,
                                      graph_rrt, graph_rrt_son, graph_rrt_father, rrt_path)
        ic(dx)

        cascade_control_path.append(cur_position)  # TODO: Record one path

        for i in range(len(next_position)):
            next_position[i] = cur_position[i] + delta * dx[i]

        tiago_env.start_baseline_resetJointState(next_position)
        p.stepSimulation()
