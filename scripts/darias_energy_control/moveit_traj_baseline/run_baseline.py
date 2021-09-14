from read_json_trajectory_pybullet import *

if __name__ == '__main__':

	tiago_2d = tiago_2d_visualize()

	start_time = time.time()
	tiago_2d.base_line()
	#tiago_2d.save_gif()
	print("--- %s seconds ---" % (time.time() - start_time))