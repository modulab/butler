import rospy

import smach
import smach_ros
from smach import *
from smach_ros import *
from std_msgs.msg import Bool
from drink_sensor.msg import DrinksStatus

from sm_global_data import GlobalData

"""
A generic smach state that monitors a std_msgs/Bool topic, and exits with outcome
'involid' as soon as true is published on that topic.
"""
class BooleanMonitor(smach_ros.MonitorState):
    def __init__(self,  topic):
        smach_ros.MonitorState.__init__(self, topic, Bool, self._callback)
    
    def _callback(self,  ud,  msg):
        return not msg.data



"""
A smach state the monitors the drink sensor for changes. If a change occurs,
then the sensor publishes a message, triggering this state to exit with outcome
'invalid'.
"""
class StolenBottleMonitor(smach_ros.MonitorState):
    def __init__(self):
        smach_ros.MonitorState.__init__(self, "/drinks_status",
                                        DrinksStatus,
                                        self._callback)
    
    def _callback(self,  ud,  msg):
        # If anything gets published here, it means something changed on the
        # tray so no need to check what...
        # Return False means 'invalid' outcome of state 
        return False
    
    
    
