import rospy

import smach
import smach_ros
from smach import *
from smach_ros import *
from std_msgs.msg import Bool
from drink_sensor.msg import DrinksStatus
from sensor_msgs.msg import Joy

#from sm_global_data import GlobalData
import sm_global_data as application


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
        # Check that we still have enough drinks to fullfill order
        required =  application.app_data.n_drinks
        if sum([1 for x in msg.status if x]) <  required:
            application.app_data.status_publisher.publish("Bottle stolen!")
            return False
        else:
            return True
    

"""
A monitor for a remote button. The remote button is enabled and disabled
by the Bool on enable_topic, and when the remote button is pressed input topic
receives True, see BooleanMonitor.
"""
class ButtonMonitor(BooleanMonitor):
    def __init__(self, button_topic):
        BooleanMonitor.__init__(self, button_topic+"/callback")
        self._publisher =  rospy.Publisher(button_topic+"/enable", Bool, latch=True)
        
    def execute(self, ud):
        self._publisher.publish(True)
        ret = BooleanMonitor.execute(self, ud)
        self._publisher.publish(False)
        return ret

        
    
"""
Monitor state that checks given joystick button
"""
class JoystickButtonMonitor(smach_ros.MonitorState):
    def __init__(self, button):
        self.button = button
        smach_ros.MonitorState.__init__(self, "/joy",
                                        Joy,
                                        self._callback)
    
    def _callback(self,  ud,  msg):
        if msg.buttons[self.button] == 1:
            return False
        else:
            return True
    
"""
A parallel monitor for a GUI button or a joystick button
"""
class JoystickAndButtonMonitor(smach.Concurrence):
    def __init__(self, button_topic, joystick_button):
        smach.Concurrence.__init__(self, outcomes=['invalid',
                                                   'valid',
                                                   'preempted'],
                                   default_outcome='valid',
                                   child_termination_cb=self.child_term_cb,
                                   outcome_cb = self.out_cb,
                                   )
        with self:
            smach.Concurrence.add('BUTTON_MONITOR', ButtonMonitor(button_topic))
            smach.Concurrence.add('JOYSTICK_MONITOR',JoystickButtonMonitor(joystick_button))
        
    def child_term_cb(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        if (outcome_map['BUTTON_MONITOR'] == 'invalid'
            or outcome_map["JOYSTICK_MONITOR"]=="invalid"):
            return True
        return False
    
    def out_cb(self, outcome_map):
        # determine what the outcome of this machine is
        if outcome_map['BUTTON_MONITOR'] == 'invalid':
            return 'invalid'
        if  outcome_map["JOYSTICK_MONITOR"]=="invalid":
            return "invalid"
        return 'preempted' # must have been, or it would still be going.
