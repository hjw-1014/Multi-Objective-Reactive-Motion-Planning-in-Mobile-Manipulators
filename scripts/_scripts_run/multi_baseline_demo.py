from icecream import ic

def mutiple_baseline(self, nums: int):
    traj = self.open_json('qtrjs.json')
    xy_traj = self.get_x_y_traj(traj)
    robotId, joint_indexes = self.start_pybullet()

    start_points = []

    for _ in range(nums):
        random_start_point = self.gen_random_start_point()
        start_points.append(random_start_point)

    for point_number in range(len(start_points)):
        ic(point_number)
        start_point = start_points[point_number]
        ic(start_point)
        # TODO: set known start points or random point
        self.set_start_points(start_point)
        # self.set_known_start_point()
        # random_start_point = self.gen_random_start_point()
        # self.set_random_start_point(random_start_point)

        next_position = [0] * 2

        for iteration in range(1000):
            # ic(iteration)
            this_time = time.time()
            # if self.check_arrive():
            # 	self.plot_cascade_control_traj()

            cur_state = self.get_robot_current_state()
            # ic(cur_state)
            dx = self.cascade_control(cur_state)
            # ic(dx)

            cur_position = cur_state[0]
            # ic(cur_position)

            self.cascade_control_path.append(cur_position)  # TODO: Record one path

            cur_dist = self.compute_euclidean_distance(self.end_point, cur_position)
            # ic(cur_dist)
            if cur_dist < self.end_point_threshold:
                ic(cur_position)
                ic(cur_dist)
                print("######### End point arrived!!! #########")

                current_cascade_control_path = self.cascade_control_path
                self.whole_cascade_control_path.append(current_cascade_control_path)
                self.cascade_control_path = []

                # self.plot_cascade_control_traj()
                # time.sleep(2.)
                print("--- %s seconds ---" % (time.time() - this_time))
                break

            for i in range(len(next_position)):
                next_position[i] = cur_position[i] + self.delta * dx[i]

            self.start_baseline_resetJointState(next_position)

            p.stepSimulation()

    self.plot_multiple_cascade_trajectories()  # TODO: Visualize
    self.plot_vector_field_in_arrows()
    self.visualize_vector_field()

from read_json_trajectory_pybullet import *

if __name__ == '__main__':
    tiago_2d = tiago_2d_visualize()

    start_time = time.time()
    tiago_2d.mutiple_baseline(nums=2)
    print("--- %s seconds ---" % (time.time() - start_time))