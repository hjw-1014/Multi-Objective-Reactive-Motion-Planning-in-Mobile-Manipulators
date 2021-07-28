from read_json_trajectory_pybullet import *


def moveit_trajectory_demo(self):
    traj = self.open_json('qtrjs.json')
    xy_traj = self.get_x_y_traj(traj)
    robotId, joint_indexes = self.start_pybullet()

    self.plot_moveit_traj(xy_traj)

    while True:
        self.visualize_trajectory()
        p.stepSimulation()


if __name__ == '__main__':

	tiago_2d = tiago_2d_visualize(activate_GUI=True)

	start_time = time.time()
	tiago_2d.moveit_trajectory_demo()
	print("--- %s seconds ---" % (time.time() - start_time))