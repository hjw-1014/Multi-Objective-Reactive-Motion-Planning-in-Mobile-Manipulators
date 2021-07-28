import pybullet as p
from icecream import ic
from utils import _utils
from _pybullet import _pybullet_utils
from _set_start import _set_start
from _controller import _controller
from _compute import _compute

# TODO: Read json trajectory
traj = _utils.open_json('qtrjs.json')
xy_traj = _utils.get_x_y_traj(traj)
robotId, joint_indexes = _pybullet_utils.start_pybullet()

# TODO: set known start points or random point
set_start_point = _set_start.set_start(set_random_start=True, set_known_start=False, start_point=None)
set_start_point.set()

next_position = [0] * 2
cascade_control_path = []
end_point = []
for jj in range(1000):
    ic(jj)

    cur_state = _pybullet_utils.get_robot_current_state()
    # Based on the current state, calculate the velocity command
    dx = _controller.cascade_control(cur_state)

    cur_position = cur_state[0]

    cascade_control_path.append(cur_position)  # TODO: Record one path

    # Calculate the distance btw end point and current point
    cur_dist = _compute.compute_euclidean_distance(end_point, cur_position)
    # ic(cur_dist)
    if cur_dist < self.end_point_threshold:
        ic(cur_position)
        print("######### End point arrived!!! #########")

        current_cascade_control_path = self.cascade_control_path
        self.whole_cascade_control_path.append(current_cascade_control_path)
        self.cascade_control_path = []

        self.plot_cascade_control_traj()  # TODO: Plot generated path
        break

    for i in range(len(next_position)):
        next_position[i] = cur_position[i] + self.delta * dx[i]

    self.start_baseline_resetJointState(next_position)

    p.stepSimulation()

from read_json_trajectory_pybullet import *

if __name__ == '__main__':

	tiago_2d = tiago_2d_visualize()

	start_time = time.time()
	tiago_2d.base_line()
	print("--- %s seconds ---" % (time.time() - start_time))