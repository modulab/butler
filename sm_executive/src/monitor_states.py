import rospy

import smach
import smach_ros
from smach import *
from smach_ros import *


#import topic messages
#from scitos_msgs.msg import MotorStatus
#from scitos_msgs.msg import BatteryState
#from ap_msgs.msg import NavStatus
from std_msgs.msg import Bool

#singleton
from sm_global_data import GlobalData


#this file has the monitor states that will run in parallel with other behaviours, using a smach concurrence container. These states subscribe to a given topic and remain active until their callback returns False.  When that happens they terminate and return outcome 'invalid'
#Current monitors:
#bumper status (pressed or not)
#battery status (low, very_low or charged)
#stuck trying to rotate on carpet







#true -> continue monitor
#false -> terminate and state returns 'invalid'

    
def go_button_cb(ud,msg):
    return not msg.go_button_pressed

            
def cancel_cb(ud,msg):
    return not msg.cancel_button_pressed

            
def force_completion_cb(ud,msg):
    return not msg.force_completion_button_pressed

    

            
def stolen_bottle_cb(ud,msg):
    return not msg.bottle_stolen


    
def go_button_monitor():    
    state=smach_ros.MonitorState("/go_button_topic", Bool, cancel_cb)
    return state
    
    
def cancel_monitor():    
    state=smach_ros.MonitorState("/cancel_button_topic", Bool, cancel_cb)
    return state
  
        
        
def force_completion_monitor():   
    state=smach_ros.MonitorState("/force_completion_button_topic", Bool, force_completion_cb)
    return state
    


def stolen_bottle_monitor():   
    state=smach_ros.MonitorState("/stolen_bottle_topic", Bool, stolen_bottle_cb)
    return state
  
