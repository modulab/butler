#!/usr/bin/env python
import roslib; roslib.load_manifest('butler')
import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32


# The points as a list of (x,y,z)
pts = [(5,4,3),
       (7,4,3),
       ]

if __name__ == '__main__':
    node = rospy.init_node("fake_perception")
    pub = rospy.Publisher("/fake_points", PointCloud)

    points = PointCloud()
    points.header.frame_id="/map"
    for i in pts:
        p = Point32()
        p.x, p.y, p.z = i
        points.points.append(p)
    
    r= rospy.Rate(5)
    while not rospy.is_shutdown():
        pub.publish(points)
        points.header.stamp = rospy.Time.now()
        r.sleep()

