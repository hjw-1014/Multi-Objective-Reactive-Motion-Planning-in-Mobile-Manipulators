#!/usr/bin/env python
import os
import json, codecs

import sys
import copy

import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState



def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPegInAHole(object):
    def __init__(self):
        super(MoveGroupPegInAHole, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "y_x_c"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())


        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")

        # Misc variables
        self.box_name = []
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.move_group.set_planning_time(60.)

        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        self.goal = None
        self.dim = 3  # TODO: Change to corresponding planning group

    def add_wall_box(self, center_z=0.6865, center_y=1., center_x=-0.05, size=0.9, timeout=4.):
        scene = self.scene

        size = size+1
        total_size = 2

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = center_z
        box_pose.pose.position.x = center_y
        box_pose.pose.position.y = center_x
        box_name = "box1"
        scene.add_box(box_name, box_pose, size=(0.4, 4., 0.1))
        self.box_name.append(box_name)

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = center_z
        box_pose.pose.position.x = center_y - 0.35
        box_pose.pose.position.y = center_x - total_size/2 - (total_size-size)/2
        box_name = "box2"
        scene.add_box(box_name, box_pose, size=(0.3, size, 0.1))
        self.box_name.append(box_name)

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = center_z
        box_pose.pose.position.x = center_y - 0.35
        box_pose.pose.position.y = center_x + total_size/2 + (total_size-size)/2
        box_name = "box3"
        scene.add_box(box_name, box_pose, size=(0.3, size, 0.1))
        self.box_name.append(box_name)

        rospy.sleep(1.)

        goal = self.move_group.get_current_pose().pose
        goal.position.y = center_x
        goal.position.x = center_y - 0.3
        goal.position.z = center_z

        goal.orientation.x = 0.
        goal.orientation.y = 0.
        goal.orientation.z = 0.
        goal.orientation.w = 1.
        self.goal = goal
        self.goal_joint = None
        self.move_group.set_pose_target(goal)

        #return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def set_init_state(self, joint_pose=None):
        moveit_robot_state = self.move_group.get_current_state()
        joint_state = moveit_robot_state.joint_state
        joint_state.header.stamp = rospy.Time.now()

        random_pose = np.random.rand(5) * 2 * np.pi - np.pi
        #random_pose[1] = 1.1
        #random_pose[0] = -0.935
        joint_state.position = random_pose.tolist()

        moveit_robot_state.joint_state = joint_state
        self.move_group.set_start_state(moveit_robot_state)
        rospy.sleep(1.)

    def cartesian_plan(self):

        waypoints = []
        w1 = copy.deepcopy(self.goal)
        w1.position.x -= 0.3

        waypoints.append(w1)

        waypoints.append(self.goal)

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0, avoid_collisions=True)

        return plan, fraction

    def joint_plan(self):
        plan = self.move_group.plan(self.goal_joint)
        return plan

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        # Publish
        display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True)

    def set_goal_joint(self):
        self.goal_joint = self.robot.get_current_state()

    def save_in_json(self, plan, filename='qtrjs.json'):
        PATH = os.path.dirname(__file__)
        file = os.path.join(PATH, filename)

        mode = 'r+' if os.path.exists(file) else 'w'
        with open(file, mode) as json_file:
            try:
                data = json.load(json_file)
            except:
                data = {}
                data['trajectories']=[]

            d = {}
            points = plan.joint_trajectory.points
            positions = np.zeros((0, self.dim))
            velocities = np.zeros((0, self.dim))
            times = np.zeros((0))
            for p_i in points:
                pos = np.array(p_i.positions)
                vel = np.array(p_i.velocities)
                t = np.array([p_i.time_from_start.to_sec()])
                positions = np.concatenate((positions, pos[None, :]), 0)
                velocities = np.concatenate((velocities, vel[None, :]), 0)
                times = np.concatenate((times, t), 0)


            d['lenght'] = len(points)
            d['positions']  = positions.tolist()
            d['velocities'] = velocities.tolist()
            d['times'] = times.tolist()
            data['trajectories'].append(d)

            json_file.seek(0)
            json.dump(data, codecs.getwriter('utf-8')(json_file), ensure_ascii=False, indent=4)
            json_file.truncate()


def main():
  try:
    move_group = MoveGroupPegInAHole()

    move_group.add_wall_box(size=0.95)

    plan, fractions = move_group.cartesian_plan()

    move_group.display_trajectory(plan)
    move_group.execute_plan(plan)

    ### set goal target ###
    move_group.set_goal_joint()

    move_group.set_init_state()

    while(rospy.is_shutdown() == False):
        move_group.set_init_state()
        plan = move_group.joint_plan()
        if len(plan.joint_trajectory.points) !=0:
            move_group.save_in_json(plan)

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
