"""
utils for collision check
@author: huiming zhou
"""

import math
import numpy as np
import os
import sys
from icecream import ic
from copy import deepcopy
import json

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Sampling_based_Planning/")

from Sampling_based_Planning.rrt_2D import my_env
from Sampling_based_Planning.rrt_2D.my_rrt import Node


class Utils:
    def __init__(self):
        self.env = my_env.MyEnv()

        self.delta = 28  # TODO: TIAGO base radius is 27cm
        #self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary
        #self.my_obs_rectangle = self.env.my_obs_rectangle

    def update_obs(self, obs_cir, obs_bound, obs_rec):
        #self.obs_circle = obs_cir
        self.obs_boundary = obs_bound
        self.obs_rectangle = obs_rec
        #self.my_obs_rectangle = self.env.my_obs_rectangle

    def get_obs_vertex(self):
        delta = self.delta
        obs_list = []

        for (ox, oy, w, h) in self.obs_rectangle:
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]
            obs_list.append(vertex_list)

        return obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):
        v1 = [o[0] - a[0], o[1] - a[1]]
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r + delta:
                return True

        return False

    def is_collision(self, start, end):
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        o, d = self.get_ray(start, end)
        obs_vertex = self.get_obs_vertex()

        for (v1, v2, v3, v4) in obs_vertex:
            if self.is_intersect_rec(start, end, o, d, v1, v2):
                return True
            if self.is_intersect_rec(start, end, o, d, v2, v3):
                return True
            if self.is_intersect_rec(start, end, o, d, v3, v4):
                return True
            if self.is_intersect_rec(start, end, o, d, v4, v1):
                return True

        # for (x, y, r) in self.obs_circle:
        #     if self.is_intersect_circle(o, d, [x, y], r):
        #         return True

        return False

    def is_inside_obs(self, node):
        delta = self.delta

        # for (x, y, r) in self.obs_circle:
        #     if math.hypot(node.x - x, node.y - y) <= r + delta:
        #         return True

        for (x, y, w, h) in self.obs_rectangle:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        for (x, y, w, h) in self.obs_boundary:
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True

        return False

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)

    @staticmethod
    def dfs(root: list, nodes_list):
        if not root:
            return
        nodes_list.append(root)

    @staticmethod
    def vertex_2_xy_list(vertex):  # TODO: added on 08.03

        node_arr = np.zeros((2, 2))
        nodes_list = []
        num_nodes = len(vertex)

        for i in range(num_nodes):
            if not vertex[i].parent:
                node_arr[0][0] = 120
                node_arr[0][1] = 100
                node_arr[1][0] = 120
                node_arr[1][1] = 100
            else:
                node_arr[0][0] = vertex[i].x  # son node x
                node_arr[0][1] = vertex[i].y  # son node y
                node_arr[1][0] = vertex[i].parent.x  # father node x
                node_arr[1][1] = vertex[i].parent.y  # father node y
            nodes_list.append(deepcopy(node_arr))
            node_arr = np.zeros((2, 2))
        ic(num_nodes)
        ic(len(nodes_list))
        return nodes_list

    @staticmethod
    def save_graph_in_json(nodes_list: list):

        file_name = 'graph_rrt.json'
        with open(file_name, 'w') as file_object:
            json.dump(nodes_list, file_object)

    @staticmethod
    def save_graph_in_npy(nodes_list: list):

        np.save("graph_rrt.npy", nodes_list)

        print("Save graph in graph_rrt.npy!!!")


    @staticmethod
    def save_vertex_in_npy(nodes_list: list):

        np.save("vertex_rrt.npy", nodes_list)

        print("Save vertex in vertex_rrt.npy!!!")

    @staticmethod
    def save_path_in_npy(path: list):

        np.save("path_rrt.npy", path)

        print("Save path in path_rrt.npy!!!")