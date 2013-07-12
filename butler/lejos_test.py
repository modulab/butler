#!/usr/bin/env python
PACKAGE = 'butler'
import roslib;roslib.load_manifest(PACKAGE)
import rospy
from std_msgs.msg import String
from nxt_lejos_msgs.msg import DNSCommand


def talker():
    pub = rospy.Publisher('nxt/dns_command', DNSCommand)
    rospy.init_node('talker')
    while not rospy.is_shutdown():
	
        pub.publish("travel",2.0)
        rospy.sleep(1.0)

        pub.publish("travel",-2.0)
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
