import pybullet as p
from utils import *
from _pybullet import start_bullet_env
from _controller import *
from _compute import *
from _plot import *

class Node:
    def __init__(self, n: list):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

def cascade_control_get_dx(xy):  ## TODO: added on 08.12

    # TODO: Read json trajectory from moveit and rrt tree
    traj, num_path_points = open_json('_scripts_run/qtrjs.json')
    xy_traj = get_x_y_traj(traj)

    rrt_path = load_rrt_path('_scripts_run/path_rrt.npy') / 100  # numpy.ndarray
    rrt_path = np.around(rrt_path, 3)
    graph_rrt = load_rrt_nodes_list("_scripts_run/graph_rrt.npy") / 100  # numpy.ndarray(np.array)
    graph_rrt = np.around(graph_rrt, 3)
    #rrt_vertex = load_rrt_vertex("vertex_rrt.npy")
    graph_rrt_son, graph_rrt_father = split_son_father(graph_rrt)

    # Based on the current state, calculate the velocity command
    dx = cep_cascade_control_rrt_tree(xy, graph_rrt_son, graph_rrt_father)

    return dx