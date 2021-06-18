#!/bin/bash

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash  #TODO change use your own workspace

export ROS_IP=192.168.10.1
export ROS_MASTER_URI=http://192.168.10.4:11311   # IP of fermat 

exec "$@"
