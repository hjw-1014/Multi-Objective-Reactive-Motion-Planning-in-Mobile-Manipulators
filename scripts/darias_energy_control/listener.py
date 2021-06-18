#!/usr/bin/env python

## Simple talker demo that listens to std_msgs/Strings published
## to the 'chatter' topic
import time

import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Pose
import numpy as np
from TIAGo2 import Tiago_OSC

p = None
class Listener(object):

    def __init__(self):
        self.point = None
        self.px = 0
        self.py = 0
        self.pz = 0

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + 'I heard ', data.position)

        global p

        self.px = data.position.x
        self.py = data.position.y
        self.pz = data.position.z
        self.point = np.array([self.px, self.py, self.pz])
        Tiago_OSC(self.point)
        p = self.point
        print('self.point: ', self.point)

        time.sleep(1)

    # def get_point(data):
    #     #rospy.loginfo(rospy.get_caller_id() + "Get point is %f", data.data)

    def pointer_receiver(self):

        rospy.init_node('pointReceiver', anonymous=True)

        rospy.Subscriber(name='pointer', data_class=Pose, callback=self.callback)

# def listener():
#
#     # In ROS, nodes are uniquely named. If two nodes with the same
#     # name are launched, the previous one is kicked off. The
#     # anonymous=True flag means that rospy will choose a unique
#     # name for our 'listener' node so that multiple listeners can
#     # run simultaneously.
#     rospy.init_node('listener', anonymous=True)
#
#     rospy.Subscriber('chatter', String, callback)
#
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

if __name__ == '__main__':
    #listener()
    L = Listener()

    L.pointer_receiver()

    rospy.spin()
    print('p: ', p)