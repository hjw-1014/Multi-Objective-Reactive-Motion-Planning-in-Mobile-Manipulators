from utils import _utils
from _pybullet import _pybullet_utils
from _set_start import *
from _compute import _compute
import pybullet as p
import time
from _controller import *
from _compute import *
from _plot import *

nums = 2

if __name__ == "__main__":
    # TODO: Read json trajectory
    traj, num_path_points = _utils.open_json('qtrjs.json')
    xy_traj = _utils.get_x_y_traj(traj)

    # TODO: tiago environment
    tiago_env = _pybullet_utils.start_bullet_env(set_random_start=True, set_known_start=False, start_point=None)
    robotId, joint_indexes = tiago_env.start_pybullet(activate_GUI=False) #TODO: Change here to start the GUI

    cascade_control_path = []
    whole_cascade_control_path = []
    end_point = [1.2, 1.0] # TODO: Change here to decide end point!!!
    end_point_threshold = 0.01
    delta = 0.5  # TODO: Change here to decide the step!!!

    start_time = time.time()
    start_points = []

    for _ in range(nums):
        random_start_point = gen_random_start_point()
        start_points.append(random_start_point)

    for point_number in range(len(start_points)):
        ic(point_number)
        start_point = start_points[point_number]
        ic(start_point)
        # TODO: set known start points or random point
        set_start_points(start_point, robotId, joint_indices)
        # self.set_known_start_point()
        # random_start_point = self.gen_random_start_point()
        # self.set_random_start_point(random_start_point)

        iteration = 1
        while True:
            next_position = [0] * 2
            ic(iteration)
            iteration += 1

            current_state = tiago_env.get_robot_current_state()
            cur_position = current_state[0]
            cascade_control_path.append(cur_position)  # TODO: Record current position

            # TODO: Check if arrive the target point
            # Calculate the distance between end point and current point
            cur_dist = _compute.compute_euclidean_distance(end_point, cur_position)
            if cur_dist < end_point_threshold:
                ic(cur_position)
                ic(cur_dist)
                print("######### End point arrived!!! #########")

                whole_cascade_control_path.append(cascade_control_path[:])
                #plot_cascade_control_traj(cascade_control_path, xy_traj, num_path_points)  # TODO: Plot generated path

                cascade_control_path = []
                break

            # Based on the current state, calculate the velocity command
            dx = cascade_control(tiago_env, num_path_points, current_state, xy_traj)

            cascade_control_path.append(cur_position)  # TODO: Record one path

            for i in range(len(next_position)):
                next_position[i] = cur_position[i] + delta * dx[i]

            tiago_env.start_baseline_resetJointState(next_position)
            p.stepSimulation()

            print("--- %s seconds ---" % (time.time() - start_time))

    plot_multiple_cascade_trajectories(whole_cascade_control_path)  # TODO: Plot generated path