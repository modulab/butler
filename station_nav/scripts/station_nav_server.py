#!/usr/bin/python

import rospy

import sys
import os
from optparse import OptionParser
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Bool, Int32
import yaml
import math

import actionlib
from move_base_msgs.msg import *

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

        self._action_client =  actionlib.SimpleActionClient("/move_base",
                                                            MoveBaseAction)
        self._current_goal=None
        rospy.loginfo("Wating for move_base action server...")
        self._action_client.wait_for_server()
        rospy.loginfo("got it.")

    def _new_goal(self, goal_station):
        goal = str(goal_station.data)
        rospy.loginfo("Got a new goal, station id=%s" % goal)
            
        self._feedback_pub.publish("Got a new goal.")
        if not self._stations.has_key(goal):
            rospy.logwarn("Trying to navigate to unknown station number.")
            self._feedback_pub.publish("Station unknown.")
            self._result_pub.publish(False)
        else:
            self._feedback_pub.publish("Station known, navigation starting.")
            self._move_base_to(self._stations[goal])

    def _stop_goal(self, stop):
        rospy.loginfo("Stopping navigation.")
        self._feedback_pub.publish("Stopping navigation")
        if self._current_goal is not None:
            self._current_goal = None
            self._action_client.cancel_goal()
        else:
            rospy.logwarn("Tried to cancel station navigation when no navigation active.")


    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(3)

    def _move_base_to(self, pose):
        """
        Use move base to get robot to pose.
        :Args:
        | pose: geometry_msgs/Pose
        """
        stamped = PoseStamped()
        stamped.pose = pose
        stamped.header.frame_id="/map"

        if self._current_goal is not None:
            self._current_goal = None
            rospy.loginfo("Canceling previous goal pose")
            self._action_client.cancel_goal()

        self._current_goal = stamped
            
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose = self._current_goal
        self._action_client.send_goal(mb_goal, self._move_base_result,
                                      self._move_base_active,
                                      self._move_base_feedback)


    def _move_base_active(self):
        rospy.loginfo("move_base goal went active.")

    def _move_base_result(self, status, result):
        if status == 3:
            ## succeeded
            self._current_goal = None
            self._result_pub.publish(True)
            rospy.loginfo("Move base succeeeded.")
        elif self._current_goal is not None:
            rospy.loginfo("Move base failed: resending goal.")
            mb_goal = MoveBaseGoal()
            mb_goal.target_pose = self._current_goal
            self._action_client.send_goal(mb_goal, self._move_base_result,
                                          self._move_base_active,
                                          self._move_base_feedback)
            

    def _move_base_feedback(self, feedback):
        if self._current_goal is None:
            rospy.logwarn("Got some feedback on movebase when not tracking a goal.")
        now_x = feedback.base_position.pose.position.x
        now_y = feedback.base_position.pose.position.y
        goal_x  = self._current_goal.pose.position.x
        goal_y  = self._current_goal.pose.position.y
        self._feedback_pub.publish("Distance to goal: "+str(math.sqrt((now_x-goal_x)**2 + (now_y-goal_y)**2)) )

        

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
    server.run()
