import numpy as np
from _compute import *
import torch

k_d = 1.
ka = 0.1
kp = 1.
kv = 1.
def cascade_control(tiago_env, num_path_points, current_state, xy_traj) -> list:
    '''
        current_state:
            current_state[0] -> current position [x, y]
            current_state[1] -> current velocity [dx, dy]
        return: velocity command dx, [vel_x, vel_y]
    '''

    current_position = current_state[0]
    dx = [0] * len(current_state)  # y and x

    current_distances_from_path_points = compute_current_distance_from_path_points_batch(current_position, xy_traj)
    # ic(len(current_distances_from_path_points))
    min_dist, min_dist_index = choose_min_dist_point(tiago_env, num_path_points, current_distances_from_path_points)

    closest_point = xy_traj[min_dist_index]

    for i in range(len(current_position)):  # TODO, change on 07.24
        dx[i] = -k_d * (current_position[i] - closest_point[i])

    return dx

def cep_cascade_control_n_points(current_position:list, current_velocity:list, graph_rrt_son, graph_rrt_father, num: int) -> list:
    ''' # TODO: Return n closest points |  09.08
        current_state:
            current_state[0] -> current position [x, y]
            current_state[1] -> current velocity [dx, dy]
        return: velocity command dx, [vel_x, vel_y] or
                acceleration command ddx [acc_x, acc_y] or
                n acc commands [[acc_x_1, acc_y_1], [acc_x_2, acc_y_2], [acc_x_2, acc_y_2], ...]
    '''

    ddx = [[0., 0.] for _ in range(num)]

    cur_dist_son, cur_dist_father = compute_cur_dist_graph_points(current_position, graph_rrt_son, graph_rrt_father)
    closest_points = choose_n_closest_points_graph(current_position,  # TODO: Return n closest points | 09.02, 09.08
                                                   cur_dist_son,
                                                   cur_dist_father,
                                                   graph_rrt_son, graph_rrt_father, num)
    print("closest_points: ", closest_points)
    for i in range(num):
        closest_point = closest_points[i]

        # Calculate dx | P control
        # for i in range(len(current_position)):  # TODO, change on 07.24
        #     dx[i] = -kp * (current_position[i] - closest_point[i])
        # sum_dx = math.hypot(dx[0], dx[1])
        # for ii in range(2):
        #     dx[ii] = dx[ii] / sum_dx

        # Calculate ddx | PD control  # TODO: need to add repulsive force | 09.10
        for j in range(2):
            ddx[i][j] = kp * (closest_point[j] - current_position[j]) + kv * (0. - current_velocity[j])
        sum_ddx = math.hypot(ddx[i][0], ddx[i][1])
        for jj in range(2):
            ddx[i][jj] /= sum_ddx

    print("ddx:", ddx)
    return ddx[:]

def cep_cascade_control_rrt_tree(current_position, current_velocity, graph_rrt_son, graph_rrt_father) -> list:
    '''
        current_state:
            current_state[0] -> current position [x, y]
            current_state[1] -> current velocity [dx, dy]
        return: velocity command dx, [vel_x, vel_y]
    '''

    dx = [0] * 2  # y and x
    ddx = [0] * 2

    cur_dist_son, cur_dist_father = compute_cur_dist_graph_points(current_position, graph_rrt_son, graph_rrt_father)
    closest_point = choose_min_dist_point_graph_batch_viz(current_position,  # TODO: Return n closest points | 09.02, 09.08
                                                   cur_dist_son,
                                                   cur_dist_father,
                                                   graph_rrt_son, graph_rrt_father)
    print("closest_point: ", closest_point)

    # Calculate dx | P control
    for i in range(len(current_position)):  # TODO, change on 07.24
        dx[i] = -kp * (current_position[i] - closest_point[i])
    sum_dx = math.hypot(dx[0], dx[1])
    for ii in range(2):
        dx[ii] = dx[ii] / sum_dx

    # Calculate ddx | PD control
    for j in range(2):
        ddx[j] = kp * (closest_point[j] - current_position[j]) + kv * (0 - current_velocity[j])
    sum_ddx = math.hypot(ddx[0], ddx[1])
    for jj in range(2):
        ddx[jj] = ddx[jj] / sum_ddx

    return ddx[:]

def cascade_control_rrt_tree(tiago_env, current_state, graph_rrt_son, graph_rrt_father) -> list:
    '''
        current_state:
            current_state[0] -> current position [x, y]
            current_state[1] -> current velocity [dx, dy]
        return: velocity command dx, [vel_x, vel_y]
    '''

    current_position = current_state[0]
    dx = [0] * len(current_state)  # y and x

    cur_dist_son, cur_dist_father = compute_cur_dist_graph_points(current_position, graph_rrt_son, graph_rrt_father)
    closest_point = choose_min_dist_point_graph_batch(tiago_env,
                                                   cur_dist_son,
                                                   cur_dist_father,
                                                   graph_rrt_son, graph_rrt_father)

    for i in range(len(current_position)):  # TODO, change on 07.24
        dx[i] = -k_d * (current_position[i] - closest_point[i])

    return dx

def cascade_control_all_points(xy_traj, min_x=-1.5, min_y=-1.5, max_x=1.5, max_y=1.5,
                               n_sample=301) -> np.array:  # TODO: Add repulsive potential field
    ''' # TODO: Add on 07.24
        grid_map: np.array((90601, 2))
        return: uv -> np.array((90601, 2)), the values for each position is the x and y velocity
    '''
    # Define the grid map
    n_sample = n_sample
    x = np.linspace(min_x, max_x, n_sample)
    y = np.linspace(min_y, max_y, n_sample)
    XY = np.meshgrid(x, y)  # TODO: So each step is 0.01,  XY is a list with 2 arrays
    xy = np.concatenate((XY[0][..., None], XY[1][..., None]), 2)  # (301, 301, 2)
    xy_flat = np.reshape(xy, (-1, 2))  # (90601, 2)
    grid_map = xy_flat  # np.array (90601, 2)

    velocity_map = []  # Define the velocity map

    for i in range(len(grid_map)):  # Transverse all the points in the map
        cur_pos = grid_map[i]  # current position
        dx = [0] * len(cur_pos)  # -> [vel_x, vel_y]
        cur_pos_in_list = cur_pos.tolist()
        cur_dist_paths = compute_current_distance_from_path_points_viz(xy_traj, cur_pos_in_list)  # For cuurent postion, get distance paths
        min_dist, min_dist_index = choose_min_dist_point_viz_vector(
            cur_dist_paths)  # Get the closest point and it's index
        closest_point = xy_traj[min_dist_index]  # Get the closest point

        # Potential field force
        attractive_force = compute_attractive_potential_force(current_position=cur_pos, closest_point=closest_point)
        repulive_force = compute_repulsive_potential_force(cur_pos)

        for j in range(len(cur_pos)):  # TODO, change on 07.24 # Based on the distance, get the attractive potential
            dx[j] = attractive_force[j] + repulive_force[j]  # x and y velocity
        dx_arr = np.asarray(dx)  # TODO: Add repulsive potential field
        velocity_map.append(dx_arr)

    velocity_map_arr = np.asarray(velocity_map)

    return velocity_map_arr

def cascade_control_all_nodes_rrtTree_viz(
                                      graph_rrt_son,
                                      graph_rrt_father,
                                      min_x=-1.5, min_y=-1.5, max_x=1.5, max_y=1.5,
                                      n_sample=301) -> np.array:  # TODO: Add repulsive potential field
    ''' # TODO: Add on 07.24
        grid_map: np.array((90601, 2))
        return: uv -> np.array((90601, 2)), the values for each position is the x and y velocity
    '''
    # Define the grid map
    x = np.linspace(min_x, max_x, n_sample)
    y = np.linspace(min_y, max_y, n_sample)
    XY = np.meshgrid(x, y)  # TODO: So each step is 0.01 or 0.001???,  XY is a list with 2 arrays
    xy = np.concatenate((XY[0][..., None], XY[1][..., None]), 2)  # (301, 301, 2)
    xy_flat = np.reshape(xy, (-1, 2))  # (90601, 2)
    grid_map = xy_flat  # np.array (90601, 2)

    velocity_map = []  # Define the velocity map

    for i in range(len(grid_map)):  # Transverse all the points in the map
        print("grid_map: ", i)
        cur_pos = grid_map[i]  # current position
        print('cur_pos: ', cur_pos)
        dx = [0] * len(cur_pos)  # -> [vel_x, vel_y]
        cur_pos_in_list = cur_pos.tolist()
        #cur_dist_son, cur_dist_father = compute_cur_dist_graph_points_batch(cur_pos_in_list, graph_rrt_son, graph_rrt_father)
        cur_dist_son, cur_dist_father = compute_cur_dist_graph_points(cur_pos_in_list, graph_rrt_son, graph_rrt_father)
        closest_point = choose_min_dist_point_graph_batch_viz(
                                cur_pos_in_list,
                                cur_dist_son,
                                cur_dist_father,
                                graph_rrt_son,
                                graph_rrt_father)  # Get the closest point and it's index
        # Potential field forces
        attractive_force = compute_attractive_potential_force(current_position=cur_pos, closest_point=closest_point)
        repulsive_force = compute_repulsive_potential_force(cur_pos)

        for j in range(len(cur_pos)):  # TODO, change on 07.24 # Based on the distance, get the attractive potential
            dx[j] = attractive_force[j] + repulsive_force[j]  # x and y velocity
        dx_arr = np.asarray(dx)  # TODO: Add repulsive potential field
        velocity_map.append(dx_arr)
    velocity_map_arr = np.asarray(velocity_map)

    return velocity_map_arr


def cep_cascade_control_rrt_tree_pos(current_position, current_velocity, graph_rrt_son, graph_rrt_father) -> list:

    """ # TODO: 09.13
        current_state:
            current_state[0] -> current position [x, y]
            current_state[1] -> current velocity [dx, dy]
        return: closest point(s), [pos_x, pos_y]
    """

    cur_dist_son, cur_dist_father = compute_cur_dist_graph_points(current_position, graph_rrt_son, graph_rrt_father)
    closest_point = choose_min_dist_point_graph_batch_viz(current_position,
                                                   cur_dist_son,
                                                   cur_dist_father,
                                                   graph_rrt_son, graph_rrt_father)
    print("closest_point: ", closest_point)

    return closest_point

def cep_cascade_control_rrt_tree_n_pos(current_position, current_velocity, graph_rrt_son, graph_rrt_father, num) -> list:

    """ # TODO: 09.13
        current_state:
            current_state[0] -> current position [x, y]
            current_state[1] -> current velocity [dx, dy]
        return: closest point(s), [pos_x, pos_y]
    """

    cur_dist_son, cur_dist_father = compute_cur_dist_graph_points(current_position, graph_rrt_son, graph_rrt_father)
    closest_points = choose_n_closest_points_graph(current_position,
                                                   cur_dist_son,
                                                   cur_dist_father,
                                                   graph_rrt_son, graph_rrt_father, num=num)
    print("closest_points: ", closest_points)

    return closest_points


def cep_track_father_rrt_tree(current_position, current_velocity, graph_rrt_son, graph_rrt_father, K): # TODO: added 09.20

    """
        current_state:
            current_state[0] -> current position [x, y]
            current_state[1] -> current velocity [dx, dy]
        return: acceleration command ddx, params for closest points
    """

    cur_dist_son, cur_dist_father = compute_cur_dist_graph_points(current_position, graph_rrt_son, graph_rrt_father)

    if K == 1:
        closest_point = track_father_baseline(current_position,  # TODO: Return n closest points | 09.02, 09.08
                                               cur_dist_son,
                                               cur_dist_father,
                                               graph_rrt_son,
                                               graph_rrt_father,
                                               K=K)
        # Calculate dx | P control
        ddx = [0] * 2

        # Calculate ddx | PD control
        for j in range(2):
            ddx[j] = kp * (closest_point[j] - current_position[j]) + kv * (0 - current_velocity[j])
        sum_ddx = math.hypot(ddx[0], ddx[1])
        for jj in range(2):
            ddx[jj] = ddx[jj] / sum_ddx

        return ddx[:]

    else:
        closest_points = track_father_baseline(current_position,
                                              cur_dist_son,
                                              cur_dist_father,
                                              graph_rrt_son,
                                              graph_rrt_father,
                                              K=K)
        print("closest_points: ", closest_points)
        num = len(closest_points)
        ddx = [[0., 0.] for _ in range(num)]
        x_dist_goal = []

        for i in range(num):
            closest_point = closest_points[i]

            # Calculate the distance from goal point for each closest point# TODO: need to add repulsive force | 09.23
            cur_x_dist = math.hypot(end_point[0] - closest_point[0], end_point[1] - closest_point[1])
            x_dist_goal.append(cur_x_dist)

            # Calculate ddx | PD control  # TODO: need to add repulsive force | 09.10
            for j in range(2):
                ddx[i][j] = kp * (closest_point[j] - current_position[j]) + kv * (0. - current_velocity[j])
            sum_ddx = math.hypot(ddx[i][0], ddx[i][1])
            for jj in range(2):
                ddx[i][jj] /= sum_ddx

        print("closest_points: ", closest_points)

        # TODO: Softmax | 09.23
        softmax_sum = 0
        x_dist_goal_t = torch.tensor(x_dist_goal)
        for i in range(num):
            softmax_sum += torch.exp(x_dist_goal_t[i])
        for j in range(num):
            x_dist_goal_t[j] = (softmax_sum - torch.exp(x_dist_goal_t[j])) / softmax_sum

        return ddx[:], x_dist_goal_t


