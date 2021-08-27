import json
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from icecream import ic

def open_json(filename) -> list:
    '''
        Open the json file which stores the path
    '''
    f = open(filename, )
    data = json.load(f)
    trajectory = data['trajectories'][0]['positions']

    # Iterating through the json
    # for i in data['trajectories'][0]['positions']:
    # 	print(i)

    f.close()
    num_path_points = len(trajectory)

    return trajectory, num_path_points

def get_x_y_traj(trajectory) -> list:
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

def load_rrt_gragh_2_maps(filename): # TODO: add on 08.02

    G = nx.read_graphml(filename)
    nx.draw(G)
    plt.show()

    # Save the relationship of nodes: key->target(son), value->source(father)
    node_pos_map = dict()  # key->node name, value->position, np.array->(x, y)
    son_father_map = dict()  # key->son node name, value-> father node name
    edge_weight_map = dict()  # key->edge name, value->edge weight(float)
    edge_connect_father_son_map = dict()  # key->edge name, value-> (father node name, son node name))

    for node in G.nodes(data=True):

        coords = node[1]["coords"].split(",")
        x = float(coords[0])
        y = float(coords[1])
        yaw = float(coords[2])
        xy = np.array([[x, y]])
        node_pos_map[node[0]] = xy
    # ic(node_pos_map)

    for edge in G.edges(data=True):

        son_father_map[edge[1]] = edge[0]
        edge_weight_map[edge[2]["id"]] = edge[2]["weight"]
        edge_connect_father_son_map[edge[2]["id"]] = (edge[1], edge[0])
    # ic(son_father_map)
    # ic(edge_weight)
    # ic(edge_connect_father_son_map)

    return node_pos_map, son_father_map, edge_weight_map, edge_connect_father_son_map

def transforem_node_map_2_list(node_pos_map: dict) -> list:
    '''
        return a list contains the positions of nodes of the graph, without orders
    '''
    nodes_pos_graph_list = []
    for val in node_pos_map.values():
        nodes_pos_graph_list.append(val)

    return nodes_pos_graph_list

def load_rrt_nodes_list(filename):

    rrt_nodes = np.load(filename)

    return rrt_nodes

def load_rrt_path(filename):

    rrt_path = np.load(filename)

    return rrt_path


def load_rrt_vertex(filename):

    rrt_vertex = np.load(filename, allow_pickle=True)

    return rrt_vertex

def split_son_father(graph_rrt: np.array):

    graph_rrt_son = np.zeros((len(graph_rrt), 2))
    graph_rrt_father = np.zeros((len(graph_rrt), 2))
    for i in range(len(graph_rrt)):
        graph_rrt_son[i] = graph_rrt[i][0]
        graph_rrt_father[i] = graph_rrt[i][1]

    return graph_rrt_son, graph_rrt_father
