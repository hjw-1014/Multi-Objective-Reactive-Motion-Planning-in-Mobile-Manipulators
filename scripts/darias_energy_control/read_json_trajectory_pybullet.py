# TODO: Load json to visualize trajectory in 2d

import json
import random

from icecream import ic
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pybullet as p
import os
import numpy as np
import pybullet_data
import time
import math
import random
#from tiago_utils import *


class tiago_2d_visualize:  # TODO: add on 07.20

	def __init__(self):

		self.joint_indices = [0, 1]
		self.trajectory = None
		self.xy_traj = None
		self.robotId = None
		self.cascade_threshold = 0.2  # TODO: Set different threshold to check the performance
		self.k_d = 1.
		self.delta = 0.1
		self.dim = 3
		self.dim_xy = 2
		self.num_path_points = 0
		self.cascade_control_path = []
		self.end_point = [1.2, 1.0]
		self.end_point_threshold = 0.01

	def open_json(self, filename):
		# Opening JSON file
		f = open(filename,)

		# returns JSON object as
		# a dictionary
		data = json.load(f)

		# ic(type(data))
		# ic(type(data['trajectories'][0]['positions']))
		# ic(len(data['trajectories'][0]['positions']))

		self.trajectory = data['trajectories'][0]['positions']
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

		self.num_path_points = len(self.trajectory)

		return self.trajectory

	def visualize_trajectory(self):

		for jj in range(len(self.xy_traj)):  # TODO: PYBULLET set joint positions
			p.resetJointState(self.robotId, self.joint_indices[0], self.xy_traj[jj][0])
			p.resetJointState(self.robotId, self.joint_indices[1], self.xy_traj[jj][1])
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

		# Num_Joints = p.getNumJoints(self.robotId)
		# # print('p.getNumJoints(robotId)', p.getNumJoints(robotId))
		# # for i in range(Num_Joints):
		# #     print('p.getJointState(robotId, {})'.format(i), p.getJointState(robotId, i))
		# for i in range(Num_Joints):
		# 	print('p.getJointInfo(robotId, {})'.format(i), p.getJointInfo(self.robotId, i))

		time.sleep(0.5)

		return self.robotId, self.joint_indices

	def get_x_y_traj(self, traj):

		length = len(self.trajectory)
		self.xy_traj = [[0, 0] for i in range(length)]
		for i in range(length):
			self.xy_traj[i] = traj[i][:2]

		return self.xy_traj

	def plot_moveit_traj(self, xy_traj: list):

		lenth = len(xy_traj)

		fig, ax = plt.subplots()
		center_size = 10
		marker_size = 5000

		for i in range(lenth):
			ax.scatter(xy_traj[i][0], xy_traj[i][1], s=center_size, alpha=1)
			ax.scatter(xy_traj[i][0], xy_traj[i][1], s=marker_size, alpha=.05)
			ax.plot(xy_traj[i][0], xy_traj[i][1])

		ax.grid(True)

		ax.set_xlim(-0.5, 1.5)
		ax.set_ylim(-0.5, 1.5)

		box_center_x = 0.5
		box_center_y = 0.5
		ax.text(box_center_x, box_center_y, "box")

		left, bottom, width, height = (0.35, 0.35, 0.3, 0.3)
		ax.add_patch(Rectangle((left, bottom), width, height,
							 facecolor="black", alpha=0.5))

		plt.show()

	def plot_cascade_control_traj(self):

		lenth = len(self.cascade_control_path)

		fig, ax = plt.subplots()
		center_size = 10
		marker_size = 5000

		xy_traj = self.xy_traj
		x_traj = [0] * self.num_path_points
		y_traj = [0] * self.num_path_points

		for i in range(lenth):
			ax.scatter(self.cascade_control_path[i][0], self.cascade_control_path[i][1], s=center_size, alpha=1)
			ax.scatter(self.cascade_control_path[i][0], self.cascade_control_path[i][1], s=marker_size, alpha=.05)
			#ax.plot(self.cascade_control_path[i][0], self.cascade_control_path[i][1])

		# ax.scatter(x_traj, y_traj, c="tab:blue", s=center_size, label="tab:blue", alpha=1)
		# ax.scatter(x_traj, y_traj, c="tab:green", s=marker_size,  label="tab:green", alpha=.05)
		#ax.legend('')

		for j in range(self.num_path_points):
			x_traj[j] = xy_traj[j][0]
			y_traj[j] = xy_traj[j][1]

		ax.plot(x_traj, y_traj, 'C3', lw=1.)

		ax.grid(True)

		ax.set_xlim(-0.5, 1.5)
		ax.set_ylim(-0.5, 1.5)

		box_center_x = 0.5
		box_center_y = 0.5
		ax.text(box_center_x, box_center_y, "box")

		left, bottom, width, height = (0.35, 0.35, 0.3, 0.3)
		ax.add_patch(Rectangle((left, bottom), width, height,
							   facecolor="black", alpha=0.5))

		plt.show()

	def get_robot_current_state(self) -> [list, list]:
		'''
			return current position and velocity
		'''
		joint_state = p.getJointStates(self.robotId, self.joint_indices)
		#ic(joint_state)

		joint_positions = [joint_state[i][0] for i in range(len(self.joint_indices))]
		# ic(joint_positions)
		# ic(type(joint_positions))
		# ic(joint_positions[0])
		# ic(len(joint_positions))

		joint_velocities = [joint_state[j][1] for j in range(len(self.joint_indices))]
		# ic(joint_velocities)
		# ic(type(joint_velocities))

		current_state = [joint_positions, joint_velocities]

		return current_state

	def compute_current_distance_from_path_points(self, current_state) -> list:

		'''
			return the distances between current position and all the points of the path
		'''

		xy_traj = self.xy_traj
		cur_positon = current_state[0]

		current_distances_from_path_points = []

		for i in range(len(xy_traj)):
			dist = self.compute_euclidean_distance(cur_positon, xy_traj[i])
			current_distances_from_path_points.append(dist)

		return current_distances_from_path_points

	def compute_euclidean_distance(self, first_point: list, second_point: list) -> float:

		'''
			return: the euclidean distance between two points
		'''

		assert len(first_point) == len(second_point), "ERROR: Two points have different dimension!!!"

		dist = 0
		for i in range(len(first_point)):
			dist += pow((first_point[i] - second_point[i]), 2)

		return math.sqrt(dist)

	def cascade_control(self, current_state) -> list:
		'''
			return velocity command dx
		'''

		xy_traj = self.xy_traj
		current_position = current_state[0]
		dx = [0] * len(current_state)  # y and x

		current_distances_from_path_points = self.compute_current_distance_from_path_points(current_state)
		ic(len(current_distances_from_path_points))
		min_dist, min_dist_index = self.choose_min_dist_point(current_distances_from_path_points)
		print("min_dist")
		closest_point = self.xy_traj[min_dist_index]

		for i in range(len(current_state)):
			dx[i] = -self.k_d * (current_position[i] - closest_point[i])

		return dx

	def choose_min_dist_point(self, current_distances_from_path_points: list):

		if not self.check_arrive():

			#current_distances_sorted = sorted(current_distances_from_path_points)
			min_dist = min(current_distances_from_path_points)
			#ic(min_dist)
			ic(current_distances_from_path_points[-1])
			min_dist_index = current_distances_from_path_points.index(min_dist)
			ic(min_dist_index)

			idx = 1
			while min_dist < self.cascade_threshold and not self.check_arrive():  # TODO: ERROR -> Get stuck here!!!
				min_dist_index += 1
				ic(min_dist_index)

				if min_dist_index == self.num_path_points:
					return min_dist, min_dist_index-1

				min_dist = current_distances_from_path_points[min_dist_index]
				ic(min_dist)
				#if min_dist < self.cascade_threshold:

				#idx += 1
				#min_dist_index = current_distances_from_path_points.index(min_dist)

			return min_dist, min_dist_index

		else:
			return self.end_point_threshold, self.num_path_points-1

	def gen_random_start_point(self):  # TODO:
		random_start_point = [0] * self.dim  # x, y, z

		random_x = 0.5
		random_y = 0.5
		while 0.35 < random_x < 0.65 and 0.35 < random_y < 0.65:
			for i in range(self.dim):
				random_x = random.random()
				random_y = random.random()

		random_start_point[0] = random_x
		random_start_point[1] = random_y

		return random_start_point

	def set_random_start_point(self, random_start_point):

		for i in range(self.dim_xy):
			p.resetJointState(self.robotId, self.joint_indices[i], random_start_point[i])

	def set_known_start_point(self):

		known_start_point = [0.2, 0.2, 0]
		ic(known_start_point)
		for i in range(self.dim_xy):
			p.resetJointState(self.robotId, self.joint_indices[i], known_start_point[i])

	def set_start_points(self, start_point: list):

		ic(start_point)
		for i in range(self.dim_xy):
			p.resetJointState(self.robotId, self.joint_indices[i], start_point[i])

	def start_base_line(self, next_position):

		for jj in range(len(next_position)):  # TODO: PYBULLET set joint positions
			p.resetJointState(self.robotId, self.joint_indices[0], next_position[0])
			p.resetJointState(self.robotId, self.joint_indices[1], next_position[1])
			time.sleep(0.1)

	def check_arrive(self):

		curren_state = self.get_robot_current_state()
		curren_position = curren_state[0]
		cur_dist = self.compute_euclidean_distance(self.end_point, curren_position)

		if cur_dist < self.end_point_threshold:
			return True

		return False

	def base_line(self, start_point):

		traj = self.open_json('qtrjs.json')
		xy_traj = self.get_x_y_traj(traj)
		robotId, joint_indexes = self.start_pybullet()

		# TODO: set known start points or random point
		self.set_start_points(start_point)
		#self.set_known_start_point()
		#random_start_point = self.gen_random_start_point()
		#self.set_random_start_point(random_start_point)

		next_position = [0] * 2

		for jj in range(1000):
			ic(jj)

			# if self.check_arrive():
			# 	self.plot_cascade_control_traj()

			cur_state = self.get_robot_current_state()
			ic(cur_state)
			dx = self.cascade_control(cur_state)
			ic(dx)

			cur_position = cur_state[0]
			ic(cur_position)
			self.cascade_control_path.append(cur_position)  # TODO: Record path

			cur_dist = self.compute_euclidean_distance(self.end_point, cur_position)
			ic(cur_dist)
			if cur_dist < self.end_point_threshold:
				ic(cur_position)
				print("######### End point arrived!!! #########")
				self.plot_cascade_control_traj()
				break

			for i in range(len(next_position)):
				next_position[i] = cur_position[i] + self.delta * dx[i]

			self.start_base_line(next_position)

			p.stepSimulation()


	def demo(self):

		traj = self.open_json('qtrjs.json')
		xy_traj = self.get_x_y_traj(traj)
		robotId, joint_indexes = self.start_pybullet()

		self.plot_moveit_traj(xy_traj)

		while True:
			self.visualize_trajectory()
			p.stepSimulation()

if __name__ == '__main__':

	tiago_2d = tiago_2d_visualize()
	tiago_2d.base_line()
	#tiago_2d.demo()

