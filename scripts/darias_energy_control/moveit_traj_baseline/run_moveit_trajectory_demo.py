from read_json_trajectory_pybullet import *

if __name__ == '__main__':

	tiago_2d = tiago_2d_visualize(activate_GUI=False)

	start_time = time.time()
	tiago_2d.moveit_trajectory_demo()
	print("--- %s seconds ---" % (time.time() - start_time))