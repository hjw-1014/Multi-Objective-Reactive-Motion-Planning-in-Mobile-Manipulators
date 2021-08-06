#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
import sys

import rospy
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import Pose
import moveit_commander
import copy

# def talker():
#     pub = rospy.Publisher('chatter', String, queue_size=10)
#     rospy.init_node('talker', anonymous=True)
#     rate = rospy.Rate(10) # 10hz
#     while not rospy.is_shutdown():
#         hello_str = "hello world %s" % rospy.get_time()
#         rospy.loginfo(hello_str)
#         pub.publish(hello_str)
#         rate.sleep()

# class MoveitTiago(object):
#
#     def __init__(self):
#         super(MoveitTiago, self).__init__()
#
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node("movegroup_tiago_test")
#
#     def plan_cartesian_path(scale=1):
#
#         group_name = "tiago_arm"
#         move_group = moveit_commander.MoveGroupCommander(group_name)
#         waypoints = []
#
#         wpose = move_group.get_current_pose().pose
#         wpose.position.z -= scale * 0.1  # First move up (z)
#         wpose.position.y += scale * 0.2  # and sideways (y)
#         waypoints.append(copy.deepcopy(wpose))
#
#         wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
#         waypoints.append(copy.deepcopy(wpose))
#
#         wpose.position.y -= scale * 0.1  # Third move sideways (y)
#         waypoints.append(copy.deepcopy(wpose))
#
#         (plan, fraction) = move_group.compute_cartesian_path(
#             waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
#         )  # jump_threshold
#
#         return plan, fraction

def point_sender():
    # mv = MoveitTiago()
    # plan, frac = mv.plan_cartesian_path()
    pub = rospy.Publisher("pointer", data_class=Pose, queue_size=10)
    rospy.init_node('pointSender', anonymous=True)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():

        tiago_pose = Pose()
        tiago_pose.position.x = 0.6
        tiago_pose.position.y = 0.
        tiago_pose.position.z = 0.79
        tiago_pose.orientation.x = 0.
        tiago_pose.orientation.y = 0.
        tiago_pose.orientation.z = 0.
        tiago_pose.orientation.w = 1.

        rospy.loginfo(tiago_pose)
        pub.publish(tiago_pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        #talker()
        point_sender()
    except rospy.ROSInterruptException:
        pass
