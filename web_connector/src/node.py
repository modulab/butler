#!/usr/bin/env python

"""
ROS node that exposes the methods of WebInterface class as services over the ROS network.
"""

import rospy
#import rospy

from connector import *

from web_connector.srv import *
from web_connector.msg import Order

class  RosWebInterface(WebInterface):
    def __init__(self):
        rospy.init_node("RosWebInterface")
        self.get_orders_srv = rospy.Service("get_orders", GetOrders, self.get_orders_handle)
        self.mark_order_complete_srv = rospy.Service("mark_order_complete", MarkOrderComplete,
                                                     self.mark_order_complete_handle)
        self.mark_active_orders_srv = rospy.Service("mark_active_orders", MarkActiveOrders,
                                                    self.mark_active_orders_handle)

        rospy.loginfo("[WEB INTERFACE] Services node active")
        rospy.spin()

    # ROS service handle for getting orders
    def get_orders_handle(self, req):
        resp = GetOrdersResponse()
        rospy.loginfo("[WEB INTERFACE] Getting orders")
        orders = self.get_orders()
        for order in orders:
            o = Order()
            o.order_id = order[0]
            o.station_id = order[1]
            o.drinks = order[2]
            o.name = order[3]
            
            resp.orders.append(o)
        return resp

    # ROS service handle for marking order active
    def mark_active_orders_handle(self, req):
        resp = MarkActiveOrdersResponse()
        rospy.loginfo("[WEB INTERFACE] Marking orders " + str(req.order_ids) + " as active.")
        self.mark_active_orders(req.order_ids)
        return resp

    # ROS service handle for marking order complete or cancelled
    def mark_order_complete_handle(self, req):
        resp = MarkOrderCompleteResponse()
        rospy.loginfo("[WEB INTERFACE] Marking order " + str(req.order_id) + " as deceased.")
        self.mark_order_complete(req.order_id)
        return resp
    

if __name__ == '__main__':
    node = RosWebInterface()
