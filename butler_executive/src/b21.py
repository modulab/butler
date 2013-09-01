#! /usr/bin/env python
import roslib; roslib.load_manifest('butler_executive')
import rospy

import sys

import smach
import smach_ros

from monitor_states import BooleanMonitor #go_button_monitor
from go_to_station import cancelable_go_to_station

from std_msgs.msg import Bool

# ParameterStore is a singleton class that contains all the battery tresholds. 
# It is used so we that the updated tresholds can then be read by the battery 
# monitor in monitor_states.py
from sm_global_data import GlobalData

# This file implements the higher level state machine for long term patrolling.
# It uses both the navigation and the dock and charge state machines

class GoToBase(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded']
        )
        #create client for service
        rospy.sleep(1)


    def execute(self,userdata):
        #execute service
        rospy.sleep(1)
        return 'succeeded'


class GetAndMarkOrders(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded']
        )
        #create client for service        
        rospy.sleep(1)


    def execute(self,userdata):
        #execute service
        rospy.sleep(1)
        return 'succeeded'        


class SayOrders(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['empty_tray']
        )
        #create client for speaking service
        #subscribe to beer button topic
        self.tray_empty=False
        rospy.sleep(1)

    def beer_button_cb(self,msg):
        self.tray_empty=msg.is_tray_empty

    def execute(self,userdata):
        #execute speaking service
        
        #loop until tray is empty
        rospy.sleep(1)
        return 'empty_tray'
        
        
class MarkOrdersComplete(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded']
        )
        #create client for service        
        rospy.sleep(1)


    def execute(self,userdata):
        #execute service
        rospy.sleep(1)
        return 'succeeded'        
        
        
        
def main():
    rospy.init_node('buttler')
	
    # Create a SMACH state machine
    butler_sm = smach.StateMachine(outcomes=['succeeded','aborted'])
    with butler_sm:
        smach.StateMachine.add('GO_TO_BASE', GoToBase(),  transitions={'succeeded':'GET_AND_MARK_ORDERS'})
        smach.StateMachine.add('GET_AND_MARK_ORDERS', GetAndMarkOrders(),  transitions={'succeeded':'WAIT_FOR_GO'})
        smach.StateMachine.add('WAIT_FOR_GO', BooleanMonitor('/go_button'),
                               transitions={'invalid':'CANCELABLE_GO_TO_STATION',
                                            'valid':'WAIT_FOR_GO',
                                            'preempted':'WAIT_FOR_GO'})
        smach.StateMachine.add('CANCELABLE_GO_TO_STATION', cancelable_go_to_station(),  transitions={'arrived_to_station':'SAY_ORDERS','forced_completion':'SAY_ORDERS','cancelled':'GO_TO_BASE'})
        smach.StateMachine.add('SAY_ORDERS', SayOrders(),  transitions={'empty_tray':'MARK_ORDERS_COMPLETE'})
        smach.StateMachine.add('MARK_ORDERS_COMPLETE',  MarkOrdersComplete(),  transitions={'succeeded':'GO_TO_BASE'})
        
    # Introspection to allow smach_viewer
    sis = smach_ros.IntrospectionServer('server_name', butler_sm, '/SM_ROOT')
    
    # Execute SMACH plan
    sis.start()
    outcome = butler_sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()






0
