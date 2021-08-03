import numpy as np
from _compute import *

k_d = 1.

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

def cascade_control_rrt_tree(tiago_env, num_path_points, current_state, graph_rrt, graph_rrt_son, graph_rrt_father, rrt_path) -> list:
    '''
        current_state:
            current_state[0] -> current position [x, y]
            current_state[1] -> current velocity [dx, dy]
        return: velocity command dx, [vel_x, vel_y]
    '''

    current_position = current_state[0]
    dx = [0] * len(current_state)  # y and x

    cur_dist_son, cur_dist_father = compute_cur_dist_graph_points_batch(current_position, graph_rrt, graph_rrt_son, graph_rrt_father)
    closest_point = choose_min_dist_point_gragh(tiago_env,
                                                   num_path_points,
                                                   cur_dist_son,
                                                   cur_dist_father,
                                                   graph_rrt, graph_rrt_son, graph_rrt_father, rrt_path)

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