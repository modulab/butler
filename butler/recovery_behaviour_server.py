#!/usr/bin/env python
PACKAGE = 'butler'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.client
from std_msgs.msg import Bool


client = dynamic_reconfigure.client.Client('move_base')

def brake_server():
	rospy.init_node('recovery_behaviour_server')
	rospy.Subscriber("butler/enable_recovery_behaviours", Bool, callback)
	rospy.spin()


def callback(data):
	params = { 'recovery_behavior_enabled' : data.data }
	config = client.update_configuration(params)

if __name__ == '__main__':
    brake_server()
