from read_json_trajectory_pybullet import *

if __name__ == '__main__':

	tiago_2d = tiago_2d_visualize()

	start_time = time.time()
	tiago_2d.visualize_vector_field()
	print("--- %s seconds ---" % (time.time() - start_time))