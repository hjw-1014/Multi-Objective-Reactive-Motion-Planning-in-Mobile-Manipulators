# TODO: Load json to visualize trajectory in 2d

import json
from icecream import ic
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

# Opening JSON file
f = open('qtrjs.json',)

# returns JSON object as
# a dictionary
data = json.load(f)

ic(type(data))
ic(type(data['trajectories'][0]['positions']))
ic(len(data['trajectories'][0]['positions']))

traj = data['trajectories'][0]['positions']
# ic(traj)
# ic(traj[0])
# ic(traj[-1])
# ic(traj[:][:2])

# Iterating through the json
# list
# for i in data['trajectories'][0]['positions']:
# 	print(i)

# Closing file
f.close()

def get_x_y_traj(traj: list):
	length = len(traj)
	xy_traj = [[0, 0] for i in range(length)]
	for i in range(length):
		xy_traj[i] = traj[i]
	return xy_traj

def plot_traj(xy_traj: list):

	lenth = len(xy_traj)

	fig, ax = plt.subplots()
	center_size = 10
	marker_size = 7500
	for i in range(lenth):
		ax.scatter(xy_traj[i][0], xy_traj[i][1], s=center_size, alpha=1)
		ax.scatter(xy_traj[i][0], xy_traj[i][1], s=marker_size, alpha=.05)
	ax.grid(True)

	ax.set_xlim(-0.5, 1.5)
	ax.set_ylim(-0.5, 1.5)

	box_center_x = 0.5
	box_center_y = 0.5
	ax.text(box_center_x, box_center_y, "box")

	left, bottom, width, height=  (0.35, 0.35, 0.3, 0.3)
	ax.add_patch(Rectangle((left, bottom), width, height,
						 facecolor="black", alpha=0.5))

	plt.show()

if __name__ == '__main__':
	xy_traj = get_x_y_traj(traj)
	plot_traj(xy_traj)