#!/usr/bin/env python
PACKAGE = 'brake'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
import dynamic_reconfigure.client
from std_msgs.msg import Bool


client = dynamic_reconfigure.client.Client('b21')

def brake_server():
	rospy.init_node('brake_server')
	rospy.Subscriber("b21/cmd_brake_power", Bool, callback)
	rospy.spin()


def callback(data):
	params = { 'brake_enabled' : data.data }
	config = client.update_configuration(params)

if __name__ == '__main__':
    brake_server()
