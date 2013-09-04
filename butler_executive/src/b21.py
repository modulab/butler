#! /usr/bin/env python
import roslib; roslib.load_manifest('butler_executive')
import rospy

import sys

import smach
import smach_ros

from monitor_states import BooleanMonitor, ButtonMonitor, JoystickAndButtonMonitor
from go_to_station import CancelableGoToStation, JoystickOverideableGoToStation
from order_states import * 

from talker.srv import Speach
from std_msgs.msg import Bool,  String
from web_connector.srv import *
from drink_sensor.srv import * 

import sm_global_data as application

"""
State that checks that the operator loaded enough beer.
"""
class CheckCorrectLoading(smach.State):
    def __init__(self, max_beers=6):
        smach.State.__init__(self,
            outcomes    = ['succeeded', 'failed']
        )

        # service to query drinks status
        try:
            rospy.wait_for_service("request_drinks_status", 4)
        except:
            rospy.logerr("Can't find drink sensor services!")
            sys.exit(1)
            
        self.request_drinks_status = rospy.ServiceProxy("request_drinks_status",
                                                        RequestDrinksStatus)
        

    def execute(self,userdata):
        application.app_data.status_publisher.publish("Checking correct loading.")
        #execute service
        resp = self.request_drinks_status()
        drinks = resp.status.status
        have =  sum(1 for x in drinks if x)
        application.app_data.status_publisher.publish(" need " +
                                                      str(application.app_data.n_drinks) +
                                                      ",  have " + str(have))
        
        if (application.app_data.n_drinks != have):
            application.app_data.status_publisher.publish("Check loading and press go.")
            return 'failed'
        else:
            return "succeeded"


        
        
def main():
    rospy.init_node('butler_executive')
    
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
        smach.StateMachine.add('GO_TO_BASE', JoystickOverideableGoToStation(to_base=True),
                               transitions={'succeeded':'GET_AND_MARK_ORDERS'})
        smach.StateMachine.add('GET_AND_MARK_ORDERS', GetAndMarkOrders(),
                               transitions={'succeeded':'WAIT_FOR_GO',
                                            'no_orders': 'GET_AND_MARK_ORDERS',
                                            'not_enough_beers': 'GET_AND_MARK_ORDERS',}) 
        #smach.StateMachine.add('WAIT_FOR_GO', ButtonMonitor('/remote_buttons/go'),
        smach.StateMachine.add('WAIT_FOR_GO', JoystickAndButtonMonitor(button_topic='/remote_buttons/go',
                                                                       joystick_button=7),
                               transitions={'invalid':'CHECK_LOADING',
                                            'valid':'WAIT_FOR_GO',
                                            'preempted':'WAIT_FOR_GO'})
        smach.StateMachine.add('CHECK_LOADING', CheckCorrectLoading(),
                               transitions={'failed':'WAIT_FOR_GO',
                                            'succeeded':'CANCELABLE_GO_TO_STATION'})
        smach.StateMachine.add('CANCELABLE_GO_TO_STATION', CancelableGoToStation(),
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


