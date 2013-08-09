#!/usr/bin/env python
PACKAGE = 'butler'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
import tf
import math
from geometry_msgs.msg import PoseWithCovarianceStamped



def angles():
	rospy.init_node('angles_server')
	rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, callback)
	rospy.spin()


def callback(data):
	print tf.transformations.quaternion_from_euler(0, 0, 30*math.pi/180)

if __name__ == '__main__':
    angles()
