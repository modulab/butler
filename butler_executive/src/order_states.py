import rospy

import smach
import smach_ros
from smach import *
from smach_ros import *
from std_msgs.msg import Bool
from drink_sensor.msg import DrinksStatus


from talker.srv import Speach
from std_msgs.msg import Bool,  String
from web_connector.srv import *
from drink_sensor.srv import *

#from sm_global_data import GlobalData
import sm_global_data as application


"""
State the gets the current orders, selects which ones to process, marks them
as active and exits 'succeeded' if orders, 'no_orders' if nothing to do.
The selected orders are then stored in the globabl application data singleton
for use in other states.
The constructor parameter 'already_carrying' forces only orders that will be
covered by the already loaded beers. In this case, the outcome could be
'not_enough_beers', and max_beers is taken to be the number of beers in the caddy.
"""
class GetAndMarkOrders(smach.State):
    def __init__(self, max_beers=6, already_carrying=False):
        smach.State.__init__(self,
            outcomes    = ['succeeded', 'no_orders', 'not_enough_beers']
        )
        self.max_beers = max_beers
        
        # create client for service
        try:
            rospy.wait_for_service("get_orders", 4)
            rospy.wait_for_service("mark_active_orders", 4)
        except:
            rospy.logerr("Can't find web_connector services!")
            sys.exit(1)
        
         # service to query drinks status
        try:
            rospy.wait_for_service("request_drinks_status", 4)
        except:
            rospy.logerr("Can't find drink sensor services!")
            sys.exit(1)
            
        self.get_orders = rospy.ServiceProxy("get_orders", GetOrders)
        self.mark_active_orders = rospy.ServiceProxy("mark_active_orders", MarkActiveOrders)
        self.request_drinks_status = rospy.ServiceProxy("request_drinks_status",
                                                        RequestDrinksStatus)
        self.already_carrying = already_carrying


    def execute(self,userdata):
        application.app_data.status_publisher.publish("Looking up order list.")
        #execute service
        resp = self.get_orders()
        orders = resp.orders
        application.app_data.status_publisher.publish(" -> found "+str(len(orders)))

        if self.already_carrying == True:
            status =  self.request_drinks_status().status.status
            self.max_beers = sum([1 for x in status if x])
            application.app_data.status_publisher.publish("Already carrying " +
                                                          str(self.max_beers))

        
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
            if carrying > self.max_beers:
                carrying -= order_weights[order.drinks]
                break
            this_round.append(order)

        #
        
        # mark the orders as active
        req =  MarkActiveOrdersRequest()
        req.order_ids = []
        for order in this_round:
            req.order_ids.append(order.order_id)
            
        self.mark_active_orders(req)
        
        application.app_data.order_list =  this_round
        application.app_data.n_drinks = carrying
        
        if not self.already_carrying:
            application.app_data.status_publisher.publish("Load " + str(carrying) +
                                                          " beers and press go!")
        else:
            application.app_data.status_publisher.publish("Adapted delivery for lost beer.")
            
        if carrying == 0:  # if nothing could be loaded, not enough beers
            return 'not_enough_beers'
        
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
        initial = self.request_drinks_status().status.status
        initial =  sum([1 for x in initial if x])
        application.app_data.status_publisher.publish("Arrived with orders, off-loading")
        # Say the names of the peoples orders
        say =  "Hello! "
        order_weights = {'1beer': "1 beer", '2beer': "Two beers", '3beer': "Three beers",}

        for order in application.app_data.order_list:
            say = say + order_weights[order.drinks] + " here for " +  order.name + ", and "
        say = say[:-6] # remove the last and
        application.app_data.talk_service(String(say))
        
        # Wait until the drinks are all taken...timeout too
        for i in range(200): # 20 seconds timeout
            response =  self.request_drinks_status()
            status =  response.status.status
            # check the number of drinks
            n_drinks = sum([1 for x in status if x ])
            if (initial - n_drinks) == application.app_data.n_drinks:
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
        