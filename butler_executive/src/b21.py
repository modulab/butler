#! /usr/bin/env python
import roslib; roslib.load_manifest('butler_executive')
import rospy

import sys

import smach
import smach_ros

from monitor_states import BooleanMonitor, ButtonMonitor #go_button_monitor
from go_to_station import cancelable_go_to_station

from talker.srv import Speach
from std_msgs.msg import Bool,  String

import sm_global_data as application

class GoToBase(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded']
        )
        #create client for service
        #rospy.sleep(1)


    def execute(self,userdata):
        #execute service
        application.app_data.status_publisher.publish("Going to base station.")

        rospy.sleep(1)
        return 'succeeded'


class GetAndMarkOrders(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded']
        )
        #create client for service        
        #rospy.sleep(1)


    def execute(self,userdata):
        application.app_data.status_publisher.publish("Looking up order list.")
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
        #rospy.sleep(1)

    def beer_button_cb(self,msg):
        self.tray_empty=msg.is_tray_empty

    def execute(self,userdata):
        application.app_data.status_publisher.publish("Arrived with orders, off-loading")
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
        #rospy.sleep(1)


    def execute(self,userdata):
        application.app_data.status_publisher.publish("Marking orders complete.")
        #execute service
        rospy.sleep(1)
        return 'succeeded'        
        
        
        
def main():
    rospy.init_node('buttler')
    
    # Initialise talking services
    try:
        rospy.wait_for_service("/say", 10)
        application.app_data.talk_service =  rospy.ServiceProxy("/say", Speach )
    except:
        rospy.logerr("Talking service not started")
        application.app_data.talk_service =  lambda x: rospy.loginfo(x)
      
    # Create a publisher for log status
    application.app_data.status_publisher = rospy.Publisher("/buttler_status_messages",  String)
    application.app_data.status_publisher.publish("Starting up")
    
    # Create a SMACH state machine
    butler_sm = smach.StateMachine(outcomes=['succeeded','aborted'])
    with butler_sm:
        smach.StateMachine.add('GO_TO_BASE', GoToBase(),
                               transitions={'succeeded':'GET_AND_MARK_ORDERS'})
        smach.StateMachine.add('GET_AND_MARK_ORDERS', GetAndMarkOrders(),
                               transitions={'succeeded':'WAIT_FOR_GO'})
        smach.StateMachine.add('WAIT_FOR_GO', ButtonMonitor('/remote_buttons/go/cb',
                                                            '/remote_buttons/go/enable'),
                               transitions={'invalid':'CANCELABLE_GO_TO_STATION',
                                            'valid':'WAIT_FOR_GO',
                                            'preempted':'WAIT_FOR_GO'})
        smach.StateMachine.add('CANCELABLE_GO_TO_STATION', cancelable_go_to_station(),
                               transitions={'arrived_to_station':'SAY_ORDERS',
                                            'forced_completion':'SAY_ORDERS',
                                            'cancelled':'GO_TO_BASE'})
        smach.StateMachine.add('SAY_ORDERS', SayOrders(),
                               transitions={'empty_tray':'MARK_ORDERS_COMPLETE'})
        smach.StateMachine.add('MARK_ORDERS_COMPLETE',  MarkOrdersComplete(),
                               transitions={'succeeded':'GO_TO_BASE'})
        
    # Introspection to allow smach_viewer
    sis = smach_ros.IntrospectionServer('server_name', butler_sm, '/SM_ROOT')
    
    # Execute SMACH plan
    sis.start()
    outcome = butler_sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()


