import numpy as np
import math
from copy import deepcopy
from icecream import ic

repulsive_threshold = 0.42  # 0.27+0.15
cascade_threshold = 0.15
end_point_threshold = 0.02
end_point = [1.2, 1.0]
box_position = [0.5, 0.5]
k_d = 1.
repulsive_scale = 1.
delta = 0.15

def compute_current_distance_from_path_points_batch(cur_position, xy_traj) -> list:
    '''
        return the distances between current position and all the points of the path
    '''
    xy_traj_arr = np.asarray(xy_traj)
    cur_pos_arr = np.asarray(cur_position)

    dist_arr = xy_traj_arr - cur_pos_arr
    current_distances_from_path_points = np.diag(np.dot(dist_arr, dist_arr.T)).reshape([-1, 1])

    return current_distances_from_path_points.tolist()

def compute_cur_dist_graph_points_batch(cur_position: list, graph_rrt_son: np.array, graph_rrt_father: np.array):
    '''
        return the distances between current position and all the points of the rrt graph
    '''
    cur_pos_arr = np.asarray(cur_position)

    dist_arr_son = graph_rrt_son - cur_pos_arr
    dist_arr_father = graph_rrt_father - cur_pos_arr
    current_distances_from_path_points_son = np.diag(np.dot(dist_arr_son, dist_arr_son.T)).reshape([-1, 1])
    current_distances_from_path_points_father = np.diag(np.dot(dist_arr_father, dist_arr_father.T)).reshape([-1, 1])

    return np.around(current_distances_from_path_points_son, 3).tolist(), np.around(current_distances_from_path_points_father, 3).tolist()

def compute_cur_dist_graph_points(cur_position, graph_rrt_son: np.array, graph_rrt_father: np.array) -> list:
    ''' # TODO: added on 08.05, good but we need batch computation, so this function just for test
        return the distances between current position and all the points of the path
    '''

    current_distances_from_path_points_son = []
    current_distances_from_path_points_father = []
    for i in range(len(graph_rrt_son)):
        dist_son = compute_euclidean_distance(cur_position, graph_rrt_son[i])
        current_distances_from_path_points_son.append(dist_son)
        dist_father = compute_euclidean_distance(cur_position, graph_rrt_father[i])
        current_distances_from_path_points_father.append(dist_father)

    return current_distances_from_path_points_son, current_distances_from_path_points_father


def compute_current_distance_from_path_points(cur_position, xy_traj) -> list:
    '''
        return the distances between current position and all the points of the path
    '''

    current_distances_from_path_points = []

    for i in range(len(xy_traj)):
        dist = compute_euclidean_distance(cur_position, xy_traj[i])
        current_distances_from_path_points.append(dist)

    return current_distances_from_path_points


def compute_current_distance_from_path_points_viz(xy_traj, cur_position: list) -> list:  # TODO: add on 07.24

    '''
        return the distances between current position and all the points of the path
    '''


    current_distances_from_path_points = []

    for i in range(len(xy_traj)):
        dist = compute_euclidean_distance(cur_position, xy_traj[i])
        current_distances_from_path_points.append(dist)

    return current_distances_from_path_points

def compute_current_distance_from_rrtGraph_points_viz(graph_rrt_son_list, cur_position: list) -> list:  # TODO: add on 08.04

    '''
        return the distances between current position and all the points of the path
    '''


    current_distances_from_rrtGraph_points = []

    for i in range(len(graph_rrt_son_list)):
        dist = compute_euclidean_distance(cur_position, graph_rrt_son_list[i])
        current_distances_from_rrtGraph_points.append(dist)

    return current_distances_from_rrtGraph_points

def compute_euclidean_distance(first_point: list, second_point: list) -> float:
    '''
        return: the euclidean distance between two points
    '''

    assert len(first_point) == len(second_point), "ERROR: Two points have different dimension!!!"

    dist = 0
    for i in range(len(first_point)):
        dist += pow((first_point[i] - second_point[i]), 2)

    return math.sqrt(dist)


def choose_min_dist_point(tiago_env, num_path_points, current_distances_from_path_points: list) -> (float, int):  # TODO: Need to update if add a path grpgh 08.02
    '''
        Firstly, check if this point is already close enough to the end point.
        if not:
            Choose the closest distance and its index from the given list, this list contains
            the distances of between a known point and all the points of the path (generated by RRTstar)
            if the distance is smaller than self.cascade_threshold:
                go to the next point until bigger than self.cascade_threshold or close enough to the end point
        if yes:
            return the last point of the path
    '''
    if not tiago_env.check_arrive():

        # current_distances_sorted = sorted(current_distances_from_path_points)
        min_dist = min(current_distances_from_path_points)
        # ic(min_dist)
        # ic(current_distances_from_path_points[-1])
        min_dist_index = current_distances_from_path_points.index(min_dist)
        # ic(min_dist_index)

        idx = 1
        while min_dist[0] < cascade_threshold and not tiago_env.check_arrive():  # TODO: ERROR -> Get stuck here!!! FIXED 0720
            min_dist_index += 1
            # ic(min_dist_index)

            if min_dist_index == num_path_points:
                return min_dist, min_dist_index - 1

            min_dist = current_distances_from_path_points[min_dist_index]

        return min_dist, min_dist_index

    else:
        return end_point_threshold, num_path_points - 1

def choose_min_dist_point_graph_batch(tiago_env,
                                cur_dist_son: list,
                                cur_dist_father: list,
                                graph_rrt_son: np.array,
                                graph_rrt_father: np.array) -> list:
    # TODO: Need to update if add a path gragh | added on 08.03
    '''
        Firstly, check if this point is already close enough to the end point.
        if not:
            Choose the closest distance and its index from the given list, this list contains
            the distances of between a known point and all the points of the path (generated by RRTstar)
            if the distance is smaller than self.cascade_threshold:
                go to the next point until bigger than self.cascade_threshold or close enough to the end point
        if yes:
            return the last point of the path
    '''
    if not tiago_env.check_arrive() and tiago_env.next_step_arrive():
        return end_point

    elif tiago_env.check_arrive():
        return end_point

    elif not tiago_env.check_arrive() and not tiago_env.next_step_arrive():
        son_dist = min(cur_dist_son)
        son_dist_index = cur_dist_son.index(son_dist)

        # TODO: -> cur_node, 08.03
        while son_dist < cascade_threshold:

            # TODO: 1Thinking how to move to the next point (Father node) which is not inside the cascade_threshold on 08.04 !!!!!!!!!!!!!
            father_node_dist = cur_dist_father[son_dist_index]
            father_node = graph_rrt_father[son_dist_index].tolist()
            father_end_dist = math.hypot(father_node[0] - end_point[0], father_node[1] - end_point[1])

            # if father_end_dist < delta:
            #     return end_point
            # elif father_node_dist[0] <= cascade_threshold and father_end_dist >= delta:
            #     son_dist_index = cur_dist_son.index(father_node_dist)
            #     son_dist = cur_dist_father[son_dist_index]
            # elif father_node_dist[0] > cascade_threshold and father_end_dist >= delta:
            #     return father_node

            print('2')
            # TODO: 2Thinking how to move to the next point (Father node) which is not inside the cascade_threshold on 08.04 !!!!!!!!!!!!!
            # father_node_dist = cur_dist_father[son_dist_index]
            # # print("son_dist: ", son_dist)
            # # print('father_node_dist: ', father_node_dist)
            # father_node = graph_rrt_father[son_dist_index].tolist()
            # father_end_dist = math.hypot(father_node[0] - end_point[0], father_node[1] - end_point[1])
            # if father_end_dist < delta:
            #     print('3')
            #     return end_point  # TODO: THINKING!!! 08.04
            # elif delta <= father_node_dist[0] < cascade_threshold:
            #     print('4')
            #     son_dist_index = cur_dist_son.index(father_node_dist)
            #     son_dist = cur_dist_father[son_dist_index]
            #     father_node_idx = cur_dist_father.index(father_node_dist)
            # elif father_node_dist[0] >= cascade_threshold:
            #     print("5")
            #     return father_node

            # # TODO: 3Change here 08.09
            if father_node_dist >= cascade_threshold:
                return father_node
            elif father_node_dist < cascade_threshold:
                if father_end_dist < delta:
                    return father_node
                son_dist_index = cur_dist_son.index(father_node_dist)
                son_dist = cur_dist_father[son_dist_index]

        return graph_rrt_son[son_dist_index].tolist()

def choose_min_dist_point_graph_batch_viz(cur_position: list,  # TODO: -> added on 08.05
                                cur_dist_son: list,
                                cur_dist_father: list,
                                graph_rrt_son: np.array,
                                graph_rrt_father: np.array) -> (float, int):

    cur_end_dist = math.hypot(cur_position[0]-end_point[0], cur_position[1]-end_point[1])

    if end_point_threshold <= cur_end_dist <= delta:
        return end_point

    elif cur_end_dist < end_point_threshold:
        return end_point

    elif cur_end_dist > delta:
        son_dist = min(cur_dist_son)
        son_dist_index = cur_dist_son.index(son_dist)
        while son_dist < cascade_threshold:
            # TODO: Thinking how to move to the next point (Father node) which is not inside the cascade_threshold on 08.04 !!!!!!!!!!!!!
            # TODO: Make a change on 08.05, works
            father_node_dist = cur_dist_father[son_dist_index]
            father_node = graph_rrt_father[son_dist_index].tolist()
            father_end_dist = math.hypot(father_node[0] - end_point[0], father_node[1] - end_point[1])
            if father_node_dist >= cascade_threshold:
                return father_node
            elif father_node_dist < cascade_threshold:
                if father_end_dist < delta:
                    return father_node
                #print('father_end_dist: ', father_end_dist)
                #print("father_node_dist: ", father_node_dist)
                son_dist_index = cur_dist_son.index(father_node_dist)
                son_dist = cur_dist_father[son_dist_index]

        return graph_rrt_son[son_dist_index].tolist()

def choose_min_dist_point_graph_viz(cur_position: list,  # TODO: -> add on 08.04
                                cur_dist_son: list,
                                cur_dist_father: list,
                                graph_rrt_son: np.array,
                                graph_rrt_father: np.array) -> (float, int):

    cur_end_dist = math.hypot(cur_position[0]-end_point[0], cur_position[1]-end_point[1])

    if end_point_threshold <= cur_end_dist <= delta:
        return end_point

    elif cur_end_dist < end_point_threshold:
        return end_point

    elif cur_end_dist > delta:
        son_dist = min(cur_dist_son)
        son_dist_index = cur_dist_son.index(son_dist)
        while son_dist < cascade_threshold:
            print('2')
            # TODO: Thinking how to move to the next point (Father node) which is not inside the cascade_threshold on 08.04 !!!!!!!!!!!!!
            father_node_dist = cur_dist_father[son_dist_index]
            # print("son_dist: ", son_dist)
            # print('father_node_dist: ', father_node_dist)
            father_node = graph_rrt_father[son_dist_index].tolist()
            father_end_dist = math.hypot(father_node[0] - end_point[0], father_node[1] - end_point[1])
            if father_end_dist < delta:
                print('3')
                return end_point  # TODO: THINKING!!! 08.04
            elif delta <= father_node_dist < cascade_threshold:
                print('4')
                son_dist_index = cur_dist_son.index(father_node_dist)
                son_dist = cur_dist_father[son_dist_index]
                father_node_idx = cur_dist_father.index(father_node_dist)
            elif father_node_dist >= cascade_threshold:
                print("5")
                return father_node

        return graph_rrt_son[son_dist_index].tolist()

def choose_min_dist_point_viz_vector(current_distances_from_path_points: list) -> (float, int):  # TODO: add on 07.24
    '''
        Choose the closest distance and its index from the given list, this list contains
        the distances of between a known point and all the points of the path (generated by RRTstar)
    '''
    min_dist = min(current_distances_from_path_points)
    min_dist_index = current_distances_from_path_points.index(min_dist)

    return min_dist, min_dist_index  # TODO: add


def compute_repulsive_potential_force(current_position) -> list:  # TODO: 07.24
    '''
        Compute repulsive force for current position
    '''
    cur_dist = compute_euclidean_distance(current_position, box_position)
    cur_dist_arr = np.clip(np.asarray(cur_dist), a_min=end_point_threshold, a_max=float('inf'))
    cur_dist_clip = cur_dist_arr.tolist()
    gradient = [0] * len(current_position)
    for i in range(len(current_position)):
        gradient[i] = current_position[i] - box_position[i]

    repulsive_force = [0] * len(current_position)
    if cur_dist > repulsive_threshold:
        return [0, 0]
    else:
        for j in range(len(current_position)):
            repulsive_force[j] = repulsive_scale * (1. / repulsive_threshold - 1. / cur_dist_clip) * (
                        cur_dist ** 2) * gradient[j]
        return repulsive_force


def compute_attractive_potential_force(current_position, closest_point) -> list:  # TODO 07.24

    attractive_force = [0] * len(current_position)

    for i in range(len(current_position)):
        attractive_force[i] = -k_d * (current_position[i] - closest_point[i])

    return attractive_force


def round_path_position_values(self, num: int) -> list:  # TODO: set num to 2
    '''
        Round the position values to 0.01
    '''
    whole_cascade_control_path = deepcopy(self.whole_cascade_control_path)
    num_paths = len(whole_cascade_control_path)

    for i in range(num_paths):
        cur_path = whole_cascade_control_path[i]
        for pos in cur_path:
            for j in range(self.dim_xy):
                pos[j] = round(pos[j], num)

    round_whole_paths = whole_cascade_control_path
    self.round_whole_paths = round_whole_paths

    return round_whole_paths


def transform_path_2_velocity_matrix(self) -> list:
    '''
        transform the rounded path to velocity path
    '''
    round_whole_paths = deepcopy(self.round_whole_paths)
    num_paths = len(round_whole_paths)

    for i in range(num_paths):
        cur_path = round_whole_paths[i]
        len_cur_path = len(cur_path)
        for j in range(len_cur_path - 1):
            cur_path[j][0] = cur_path[j + 1][0] - cur_path[j][0]
            cur_path[j][1] = cur_path[j + 1][1] - cur_path[j][1]
        cur_path[len_cur_path - 1][0] = 0.
        cur_path[len_cur_path - 1][1] = 0.

    velocity_matrix = round_whole_paths
    self.velocity_matrix = velocity_matrix

    return velocity_matrix


def velocity_matrix_2_UV(self) -> list:
    '''
        transform the velocity path to U and V. U, V -> np.array((301, 301))
    '''
    # U = np.zeros((n, n))
    # V = np.zeros((n, n))
    U_list = []
    V_list = []

    round_whole_paths = deepcopy(self.round_whole_paths)
    velocity_matrix = deepcopy(self.velocity_matrix)

    num_paths = len(velocity_matrix)

    for n in range(num_paths):
        cur_vel_path = velocity_matrix[n]
        cur_path = round_whole_paths[n]

        len_cur_path = len(cur_path)
        len_cur_vel_path = len(cur_vel_path)

        assert len_cur_vel_path == len_cur_path, "ERROR: current velocity dimension doesn't match path's dimension"

        U = np.zeros((self.grid_number, self.grid_number))
        V = np.zeros((self.grid_number, self.grid_number))

        U = np.ones((self.grid_number, self.grid_number)) * 0.
        V = np.ones((self.grid_number, self.grid_number)) * 0.
        for i in range(int(self.grid_number / 2)):
            V[i][0] = 0.1

        for k in range(len_cur_path - 1):
            ic(k)
            cur_point = cur_path[k]
            cur_vel = cur_vel_path[k]
            # Get the position x and y in the map
            x = cur_point[0]
            y = cur_point[1]
            x_i = int((x - self.grid_map_min) * self.grid_step_reciprocal)
            y_j = int((y - self.grid_map_min) * self.grid_step_reciprocal)

            # Velocity for visualization
            ic(x_i)
            ic(y_j)
            U[x_i][y_j] = cur_vel[0]
            V[x_i][y_j] = cur_vel[1]
            ic(U[x_i][y_j])
            ic(V[x_i][y_j])

        U_list.append(U)
        V_list.append(V)

    return U_list, V_list