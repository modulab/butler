#! /usr/bin/env python
import roslib; roslib.load_manifest('butler_executive')
import rospy

import sys

import smach
import smach_ros

from monitor_states import BooleanMonitor, ButtonMonitor #go_button_monitor
from go_to_station import CancelableGoToStation, JoystickOverideableGoToStation

from talker.srv import Speach
from std_msgs.msg import Bool,  String
from web_connector.srv import *
from drink_sensor.srv import * 

import sm_global_data as application

"""
State the gets the current orders, selects which ones to process, marks them
as active and exits 'succeeded' if orders, 'no_orders' if nothing to do.
The selected orders are then stored in the globabl application data singleton
for use in other states.
"""
class GetAndMarkOrders(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded', 'no_orders']
        )
        # create client for service
        try:
            rospy.wait_for_service("get_orders", 4)
            rospy.wait_for_service("mark_active_orders", 4)
        except:
            rospy.logerr("Can't find web_connector services!")
            sys.exit(1)
            
        self.get_orders = rospy.ServiceProxy("get_orders", GetOrders)
        self.mark_active_orders = rospy.ServiceProxy("mark_active_orders", MarkActiveOrders)
        


    def execute(self,userdata):
        application.app_data.status_publisher.publish("Looking up order list.")
        #execute service
        resp = self.get_orders()
        orders = resp.orders
        application.app_data.status_publisher.publish(" -> found "+str(len(orders)))
        
        if len(orders) < 1:
            rospy.sleep(10)
            return 'no_orders'
        
        # select which orders will be processed
        order_weights = {'1beer': 1, '2beer': 2, '3beer': 3,}
        MAX_LOAD = 6
        service_station =  orders[0].station_id
        carrying = 0
        this_round = []
        for order in orders:
            if order.station_id != service_station:
                continue
            carrying += order_weights[order.drinks]
            if carrying > MAX_LOAD:
                carrying -= order_weights[order.drinks]
                break
            this_round.append(order)
        
        # mark the orders as active
        req =  MarkActiveOrdersRequest()
        req.order_ids = []
        for order in this_round:
            req.order_ids.append(order.order_id)
            
        self.mark_active_orders(req)
        
        application.app_data.order_list =  this_round
        application.app_data.n_drinks = carrying
        
        application.app_data.status_publisher.publish("Load " + str(carrying) +" beers and press go!")        
        return 'succeeded'        


"""
Say orders is entered when the robot gets to a drink station to offload drinks.
It speaks the name of the people the order is for and waits for the drink caddy
to be empty. Timeout after X seconds.
"""
class SayOrders(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['empty_tray']
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
        application.app_data.status_publisher.publish("Arrived with orders, off-loading")
        # Say the names of the peoples orders
        say =  "Order here for "
        for order in application.app_data.order_list:
            say = say + order.name + " and "
        application.app_data.talk_service(String(say))
        
        # Wait until the drinks are all taken...timeout too
        for i in range(200): # 20 seconds timeout
            response =  self.request_drinks_status()
            status =  response.status.status
            # check the number of drinks
            n_drinks = sum([1 for x in status if x ])
            if n_drinks == 0:
                application.app_data.talk_service(String("See you later."))
                return 'empty_tray'
            rospy.sleep(0.1)
            
        application.app_data.status_publisher.publish("Not all beers taken.")
        return 'empty_tray' # even though its not, this state has no other outcome
        
"""
Once drinks offloads, this state is entered and marks the orders as completed
on the server.
"""
class MarkOrdersComplete(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded']
        )
        # create client for service
        try:
            rospy.wait_for_service("mark_order_complete", 4)
        except:
            rospy.logerr("Can't find web_connector services!")
            sys.exit(1)
            
        self.mark_order_complete = rospy.ServiceProxy("mark_order_complete", MarkOrderComplete)
        
        #create client for service        
        #rospy.sleep(1)


    def execute(self,userdata):
        application.app_data.status_publisher.publish("Marking orders complete.")
        
        for order in application.app_data.order_list:
            resp = self.mark_order_complete(order.order_id)
                
        return 'succeeded'        
        
        
        
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
                                            'no_orders': 'GET_AND_MARK_ORDERS',})
        smach.StateMachine.add('WAIT_FOR_GO', ButtonMonitor('/remote_buttons/go'),
                               transitions={'invalid':'CANCELABLE_GO_TO_STATION',
                                            'valid':'WAIT_FOR_GO',
                                            'preempted':'WAIT_FOR_GO'})
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


