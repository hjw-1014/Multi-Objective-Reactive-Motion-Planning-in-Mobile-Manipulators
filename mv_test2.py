import sys
import rospy
import moveit_commander
import copy

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("movegroup_tiago_test")

group_name = "panda_arm"
move_group = moveit_commander.MoveGroupCommander(group_name)
# robot = moveit_commander.RobotCommander

scale = 1.
way_points = []
wpose = move_group.get_current_pose().pose
wpose.position.z -= scale * 0.1
wpose.position.y += scale * 0.2
way_points.append(copy.deepcopy(wpose))

wpose.position.x += scale * 0.1
way_points.append(copy.deepcopy(way_points))

wpose.position.y += 0.1
way_points.append(copy.deepcopy(way_points))

(path, fraction) = move_group.compute_cartesian_path(way_points, 0.01, 0.0)
print(path, fraction)