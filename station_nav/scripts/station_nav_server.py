#!/usr/bin/python

import rospy

import sys
import os
from optparse import OptionParser
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Bool, Int32
import yaml

class StationNavigationServer(object):
    def __init__(self, stations):
        """
        stations: dictionary of geometry_msgs/Pose
        """
        self._stations=stations
        self._goal_sub = rospy.Subscriber("/crowded_nav/go", Int32, self._new_goal)
        self._stop_sub = rospy.Subscriber("/crowded_nav/stop", Bool, self._stop_goal)
        self._feedback_pub = rospy.Publisher("/crowded_nav/feedback", String)
        self._result_pub = rospy.Publisher("/crowded_nav/result", Bool)

    def __new_goal(goal_station):
        rospy.loginfo("Got a new goal, station id=%s" % str(goal_station))
        self._feedback_pub.publish("Got a new goal.")
        if not self._stations.has_key(str(goal_station)):
            rospy.logwarn("Trying to navigate to unknown station number.")
            self._feedback_pub.publish("Station unknown.")
            self._result_pub.publish(False)
        else:
            self._feedback_pub.publish("Station known, navigation starting.")

    def _stop_goal(stop):
        rospy.loginfo("Stopping navigation.")
        self._feedback_pub.publish("Stopping navigation")
        

if __name__ == "__main__":
    rospy.init_node("station_navigation_server")
    parser = OptionParser()
    parser.add_option("-f", "--file", dest="yaml_filename",
                      help="write or add station to FILE", metavar="FILE")
    (options, args) = parser.parse_args()



    if options.yaml_filename is None:
        rospy.logerr("Need a file name")
        sys.exit(1)

    rospy.loginfo("Loading stations from file '%s'"%options.yaml_filename)

    if not os.path.isfile(options.yaml_filename):
        rospy.loginfo("Station file is non-existent. Aborting.")
        sys.exit(1)
    else:
        with open(options.yaml_filename,"r") as f:
            stations = yaml.load(f)

    rospy.loginfo("Running with stations:")
    for i in stations.keys():
        rospy.loginfo(" - %s"%i)

    server = StationNavigationServer(stations)
        
