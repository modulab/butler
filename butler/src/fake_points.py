#!/usr/bin/env python
import roslib; roslib.load_manifest('butler')
import rospy

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32


# The points as a list of (x,y,z)
pts = [ (29.2, -49.6, 1),
        (28.5, -49,  1),
        (28.1, -50.8, 1.0),
        (27.7, -51.5, 1.0),
        (26.7, -50.6, 1.0),
        (26.0, -50.0, 1.0),

        (31.9, -46.4, 1.0),
        (32.6, -45.0, 1.0),
        (33.7, -43.9, 1.0),
        (33.1, -43.5, 1.0)
       
       ]

if __name__ == '__main__':
    node = rospy.init_node("fake_perception")
    pub = rospy.Publisher("/fake_points", PointCloud)

    points = PointCloud()
    points.header.frame_id="/map"
    for i in pts:
        p = Point32()
        print i
        p.x, p.y, p.z = i
        points.points.append(p)
    
    r= rospy.Rate(5)
    while not rospy.is_shutdown():
        pub.publish(points)
        points.header.stamp = rospy.Time.now()
        r.sleep()

