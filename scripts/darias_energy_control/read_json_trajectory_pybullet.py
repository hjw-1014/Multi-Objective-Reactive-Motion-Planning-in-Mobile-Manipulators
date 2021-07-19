# TODO: Load json to visualize trajectory in 2d

import json
from icecream import ic
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pybullet as p
import os
import numpy as np
import pybullet_data
import time
#from tiago_utils import *

class tiago_2d_visualiz:

	def __init__(self):
		self.joint_indexes = [0, 1]
		self.xy_traj = None

	def open_json(self, filename):
		# Opening JSON file
		f = open(filename,)

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

		return traj

	def visualize_trajectory(self):

		traj = self.xy_traj
		for jj in range(len(traj)):  # TODO: PYBULLET set joint positions
			p.resetJointState(self.robotId, self.joint_indexes[0], traj[jj][0])
			p.resetJointState(self.robotId, self.joint_indexes[1], traj[jj][1])
			time.sleep(0.1)

	def start_pybullet(self):  # load Tiago in Pybullet

		physicsClient = p.connect(p.GUI)  #or p.DIRECT for non-graphical version
		p.setAdditionalSearchPath(pybullet_data.getDataPath())
		p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
		p.setGravity(0, 0, -9.81)

		base_dir = os.path.abspath(os.path.dirname(__file__) + '../../..')
		robot_dir = os.path.join(base_dir, 'robots/tiago/')
		urdf_filename = os.path.join(robot_dir, 'tiago_single_with_holoBaseCXY.urdf')

		planeId = p.loadURDF("plane.urdf")
		startPos = [0., 0., 0.]
		startOrientation = [0., 0., 0.]
		self.robotId = p.loadURDF(urdf_filename, startPos, p.getQuaternionFromEuler([0., 0., 0.]), useFixedBase=1)
		# p.loadURDF("cube_small.urdf", np.array([0.4, -0.5, 0.5]),
		#            p.getQuaternionFromEuler([0, 0, 0]),
		#            useFixedBase=True, useMaximalCoordinates=True)
		p.loadURDF('cube_small.urdf', np.array([0.5, 0.5, 0.15]),  # TODO: Put an object in target postion
			   p.getQuaternionFromEuler([0, 0, 0]), globalScaling=6,
			   useFixedBase=True)

		Num_Joints = p.getNumJoints(self.robotId)
		# print('p.getNumJoints(robotId)', p.getNumJoints(robotId))
		# for i in range(Num_Joints):
		#     print('p.getJointState(robotId, {})'.format(i), p.getJointState(robotId, i))
		for i in range(Num_Joints):
			print('p.getJointInfo(robotId, {})'.format(i), p.getJointInfo(self.robotId, i))

		return self.robotId, self.joint_indexes

	def get_x_y_traj(self, traj: list):

		length = len(traj)
		xy_traj = [[0, 0] for i in range(length)]
		for i in range(length):
			xy_traj[i] = traj[i]

		self.xy_traj = xy_traj

		return self.xy_traj

	def plot_traj(self, xy_traj: list):

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
	tiago_2d = tiago_2d_visualiz()
	traj = tiago_2d.open_json('qtrjs.json')
	xy_traj = tiago_2d.get_x_y_traj(traj)
	robotId, joint_indexes = tiago_2d.start_pybullet()
	while True:
		tiago_2d.visualize_trajectory()
		p.stepSimulation()
		#time.sleep(1./100.)

	tiago_2d.plot_traj(xy_traj)
