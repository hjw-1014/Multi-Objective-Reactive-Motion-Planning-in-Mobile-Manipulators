"""
RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np
from icecream import ic
import time

sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/comp_energy_policy-main/scripts/PathPlanning/Sampling_based_Planning/")
from Sampling_based_Planning.rrt_2D import my_plotting_complex, my_utils_complex, my_env_complex


class Node:
    def __init__(self, n: list):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]  # start root

        self.env = my_env_complex.MyEnv()  # TODO: change environment
        self.plotting = my_plotting_complex.Plotting(s_start, s_goal)
        self.utils = my_utils_complex.Utils()

        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary

    def planning(self):
        for i in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len and not self.utils.is_collision(node_new, self.s_goal):
                    self.new_state(node_new, self.s_goal)
                    print('### iteration: {}'.format(i))

                    # TODO: Extract all the nodes 08.03
                    t = time.strftime("%Y-%m-%d-%H_%M_%S", time.localtime())
                    save_path = self.utils.create_dir(t)
                    nodes_list = self.utils.vertex_2_xy_list(self.vertex)
                    path = self.extract_path(node_new)
                    self.utils.save_vertex_in_npy(self.vertex, t, save_path)
                    self.utils.save_graph_in_npy(nodes_list, t, save_path)
                    self.utils.save_path_in_npy(path, t, save_path)
                    cost2go_map = self.utils.calculate_cost_to_go(self.vertex)
                    # self.utils.save_cost2go_in_npy(cost2go_map, t, save_path)
                    return path, self.vertex, nodes_list, cost2go_map

        return None

    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)


def main():
    x_start = (120, 120)  # Starting node
    x_goal = (-10, -10)  # Goal node

    start_time = time.time()

    rrt = Rrt(x_start, x_goal, 5, 1e-5, 100000)
    path, graph, nodes_son_father_list, cost2go_map = rrt.planning()
    # print(cost2go_map)
    # print(len(cost2go_map))
    # print(len(nodes_son_father_list))
    print("--- %s seconds ---" % (time.time() - start_time))

    if path:
        rrt.plotting.animation(rrt.vertex, path, "RRT", True)
    else:
        print("No Path Found!")


if __name__ == '__main__':
    main()
