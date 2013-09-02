#!/usr/bin/env python
try:
    import requests
except:
    print "Need the python-rquests library to connect to the web. Please install it."
    raise Exception("Can't find requests package")

ORDERS_FILE_URL="http://www.cs.bham.ac.uk/~burbrcjc/bsf2013/data/orders.txt"
ORDER_COMPLETE_URL="http://www.cs.bham.ac.uk/~burbrcjc/bsf2013/completed.php?order_number="
ACTIVE_ORDERS_URL="http://www.cs.bham.ac.uk/~burbrcjc/bsf2013/active_order.php?active_orders="
ACTIVE_ORDERS_FILE_URL="http://www.cs.bham.ac.uk/~burbrcjc/bsf2013/data/active.txt"

class WebInterface(object):
    def __init__(self):
        pass

    def get_orders(self):
        orders = requests.get(ORDERS_FILE_URL).text
        if orders.find("<html>") > 0:
            raise Exception("Orders URL bad: html returned!")

        lines = orders.split("\n")
        orders_list=[]
        for line in lines[:-1]:
            if line=="":
                continue
            order = line.split(" ")
#            order = order.extend(["-"] * (4 - len(order)))
            checked = order[0:3]
            name=""
            for i in order[3:]:
                name = name + i + " "
            name = name[:-1]
            checked.append(name)
            orders_list.append(checked)
        return orders_list

    def mark_order_complete(self, order_id):
        result = requests.get(ORDER_COMPLETE_URL+str(order_id))
        return

    def mark_active_orders(self, order_ids):
        orders=""
        for order in order_ids:
            orders+="."+str(order)
        result = requests.get(ACTIVE_ORDERS_URL+orders)
        return

    def get_active_orders(self):
        orders = requests.get(ACTIVE_ORDERS_FILE_URL).text
        if orders.find("<html>") > 0:
            raise Exception("Orders URL bad: html returned!")

        lines = orders.split("\n")
        return lines



if __name__=="__main__":
 #   rospy.init_node("web_interface_connector")
    connector = WebInterface()
    for i in connector.get_orders():
        print i

#    connector.mark_order_complete(22)
#    connector.mark_active_orders([22])
    
