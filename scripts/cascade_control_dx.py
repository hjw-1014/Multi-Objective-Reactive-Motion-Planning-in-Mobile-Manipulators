import numpy as np
import torch
import pybullet as p
from _utils import *
from _pybullet import start_bullet_env
from _controller import *
from _compute import *
from _plot import *

class Node:
    def __init__(self, n: list):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

def torch2numpy(x):
    if x is None:
        print(x)

    if x.device.type=='cuda':
        return x.cpu().detach().numpy()
    else:
        return x.detach().numpy()

def cascade_control_get_n_x(xy: list, v_l: list, num:int):  ## TODO: added on 09.15

    # TODO: Read json trajectory from moveit and rrt tree
    #traj, num_path_points = open_json('./_scripts_run/qtrjs.json')
    #xy_traj = get_x_y_traj(traj)

    rrt_path = np.load('path_rrt.npy') / 100  # numpy.ndarray
    rrt_path = np.around(rrt_path, 3)
    graph_rrt = load_rrt_nodes_list("graph_rrt.npy") / 100  # numpy.ndarray(np.array)
    graph_rrt = np.around(graph_rrt, 3)
    #rrt_vertex = load_rrt_vertex("vertex_rrt.npy")
    graph_rrt_son, graph_rrt_father = split_son_father(graph_rrt)

    # Based on the current state, calculate the closest points
    pos_n = cep_cascade_control_rrt_tree_n_pos(xy, v_l, graph_rrt_son, graph_rrt_father, num=num) # TODO: Return n ddx from x points | 09.02

    return pos_n

def cascade_control_get_x(xy: list, v_l: list):  ## TODO: added on 09.13

    # TODO: Read json trajectory from moveit and rrt tree
    #traj, num_path_points = open_json('./_scripts_run/qtrjs.json')
    #xy_traj = get_x_y_traj(traj)

    rrt_path = np.load('path_rrt.npy') / 100  # numpy.ndarray
    rrt_path = np.around(rrt_path, 3)
    graph_rrt = load_rrt_nodes_list("graph_rrt.npy") / 100  # numpy.ndarray(np.array)
    graph_rrt = np.around(graph_rrt, 3)
    #rrt_vertex = load_rrt_vertex("vertex_rrt.npy")
    graph_rrt_son, graph_rrt_father = split_son_father(graph_rrt)

    # Based on the current state, calculate the closest points
    pos = cep_cascade_control_rrt_tree_pos(xy, v_l, graph_rrt_son, graph_rrt_father) # TODO: Return n ddx from x points | 09.02

    return pos

def cascade_control_get_dx(xy: list, v_l: list):  ## TODO: added on 08.12, 08.17

    # TODO: Read json trajectory from moveit and rrt tree
    #traj, num_path_points = open_json('./_scripts_run/qtrjs.json')
    #xy_traj = get_x_y_traj(traj)

    rrt_path = np.load('path_rrt.npy') / 100  # numpy.ndarray
    rrt_path = np.around(rrt_path, 3)
    graph_rrt = load_rrt_nodes_list("graph_rrt.npy") / 100  # numpy.ndarray(np.array)
    graph_rrt = np.around(graph_rrt, 3)
    #rrt_vertex = load_rrt_vertex("vertex_rrt.npy")
    graph_rrt_son, graph_rrt_father = split_son_father(graph_rrt)

    # Based on the current state, calculate the velocity command
    ddx = cep_cascade_control_rrt_tree(xy, v_l, graph_rrt_son, graph_rrt_father) # TODO: Return n ddx from x points | 09.02

    return ddx

def cascade_control_get_n_ddx(xy: list, v_l: list, num: int):  ## TODO: added on 08.12, 08.17, 09.09

    # TODO: Read json trajectory from moveit and rrt tree
    #traj, num_path_points = open_json('./_scripts_run/qtrjs.json')
    #xy_traj = get_x_y_traj(traj)

    #rrt_path = np.load('path_rrt.npy') / 100  # numpy.ndarray
    #rrt_path = np.around(rrt_path, 3)
    graph_rrt = load_rrt_nodes_list("graph_rrt.npy") / 100  # numpy.ndarray(np.array)
    graph_rrt = np.around(graph_rrt, 3)
    #rrt_vertex = load_rrt_vertex("vertex_rrt.npy")
    graph_rrt_son, graph_rrt_father = split_son_father(graph_rrt)

    ### Based on the current state, calculate the velocity command / accelaration command
    ddx = cep_cascade_control_n_points(xy, v_l, graph_rrt_son, graph_rrt_father, num)  # TODO: Return n ddx from x points | 09.02, 09.09

    return ddx

def track_father_get_ddx(xy: list, v_l: list, K):  ## TODO: added on 09.20

    # TODO: Read json trajectory from moveit and rrt tree
    rrt_path = np.load('path_rrt.npy') / 100  # numpy.ndarray
    rrt_path = np.around(rrt_path, 3)

    graph_rrt = load_rrt_nodes_list("graph_rrt.npy") / 100  # numpy.ndarray(np.array)
    graph_rrt = np.around(graph_rrt, 3)
    #rrt_vertex = load_rrt_vertex("vertex_rrt.npy")
    graph_rrt_son, graph_rrt_father = split_son_father(graph_rrt)

    # Based on the current state, calculate the acceleration command
    ddx = cep_track_father_rrt_tree(xy, v_l, graph_rrt_son, graph_rrt_father, K=K)  # TODO: Return 1 ddx from 1 points | 09.20

    return ddx

def track_father_get_n_ddx(xy: list, v_l: list, K:int):  ## TODO: added on 09.23

    # TODO: Read json trajectory from moveit and rrt tree
    rrt_path = np.load('path_rrt.npy') / 100  # numpy.ndarray
    rrt_path = np.around(rrt_path, 3)

    graph_rrt = load_rrt_nodes_list("graph_rrt.npy") / 100  # numpy.ndarray(np.array)
    graph_rrt = np.around(graph_rrt, 3)
    #rrt_vertex = load_rrt_vertex("vertex_rrt.npy")
    graph_rrt_son, graph_rrt_father = split_son_father(graph_rrt)

    # Based on the current state, calculate the acceleration command
    ddx, x_goal_dist = cep_track_father_rrt_tree(xy, v_l, graph_rrt_son, graph_rrt_father, K=K)  # TODO: Return n ddx from n points | 09.23

    return ddx, x_goal_dist