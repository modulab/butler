#!/usr/bin/env python
import roslib; roslib.load_manifest('service_manager')
import rospy
from web_connector.srv import *
from std_msgs.msg import Float32, Bool
from drink_sensor.msg import DrinksStatus
from drink_sensor.srv  import *

import pygtk
import gtk
import gobject

class App(object):
    def __init__(self):
        self._init_gui()
        
        self.node = rospy.init_node("service_manager")
        self.battery_sub = rospy.Subscriber("/b21/voltage",Float32,self.battery_cb)
        self.drink_status_sub = rospy.Subscriber("/drinks_status",DrinksStatus, self.drinks_cb)


        # Update the drink sensor display
        try:
            rospy.wait_for_service("request_drinks_status", 1)
            self.get_drink_status_srv = rospy.ServiceProxy("request_drinks_status",RequestDrinksStatus)
            status = self.get_drink_status_srv().status.status
            self.update_drinks_indicator(status)
        except:
            rospy.logwarn("Couldn't call drink sensor service.")

    def _init_gui(self):
        self.main_window = gtk.Window()
        self.main_window.set_title("Service Manager")
        self.main_window.connect('destroy', gtk.main_quit)
        self.main_window.set_size_request(400, 200)
        vbox = gtk.VBox(False, 8)

        robot_status = gtk.Frame("Robot status")
        robot_status_v = gtk.VBox()
        self.battery_label = gtk.Label("Battery voltage: no data!")
        self.brake_button = gtk.ToggleButton("Brakes disabled")
        self.brake_button.connect("clicked",self.brake_button_cb)
        robot_status_v.pack_start(self.battery_label,False,False)
        robot_status_v.pack_start(self.brake_button, False, False)
        robot_status.add(robot_status_v)
        vbox.pack_start(robot_status, False, False)

        orders_status = gtk.Frame("Orders")
        orders_status_v = gtk.VBox()
        sw = gtk.ScrolledWindow()
        sw.set_shadow_type(gtk.SHADOW_ETCHED_IN)
        sw.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        orders_status_v.pack_start(sw, True, True, 0)

        button_box = gtk.HBox()
        refresh_btn = gtk.Button("Refresh")
        refresh_btn.connect("clicked",self.refresh_btn_cb)
        button_box.pack_start(refresh_btn)

        completed_btn = gtk.Button("Mark Complete")
        completed_btn.connect("clicked",self.completed_btn_cb)
        button_box.pack_start(completed_btn)
        orders_status_v.pack_start(button_box, False, False)

        self.orders_list_store = gtk.ListStore(str, str, str, str, str)
        self.orders_list_store.append(["0","0","0","0", "#FFFFFF"])
        self.orders_view = gtk.TreeView(self.orders_list_store)
        self.orders_view.get_selection().set_mode(gtk.SELECTION_MULTIPLE)


        for num,i in enumerate(["Order Numer", "Station", "Order", "Customer Name"]):
            renderText = gtk.CellRendererText()
            column = gtk.TreeViewColumn(i, renderText, text=num, background=4)
            self.orders_view.append_column(column)
 
        sw.add(self.orders_view)
        orders_status.add(orders_status_v)
        vbox.pack_start(orders_status, True, True)

        
        drinks_status = gtk.Frame("Drink Sensors")
        self.drink_labels=[gtk.Label(),gtk.Label(),gtk.Label(),gtk.Label(),
                           gtk.Label(),gtk.Label(),gtk.Label(),gtk.Label()]
        drinks_v=gtk.VBox(False,False)
        drinks_top_row = gtk.HBox(False,False)
        drinks_bottom_row = gtk.HBox(False,False)
        drinks_v.pack_start(drinks_top_row)
        for i in range(0,4):
            drinks_top_row.pack_start(self.drink_labels[i], False, False)
        drinks_v.pack_start(drinks_bottom_row)
        for i in range(4,8):
            drinks_bottom_row.pack_start(self.drink_labels[i], False, False)
        drinks_status.add(drinks_v)
        vbox.pack_start(drinks_status, False, False)
        
        self.main_window.add(vbox)


    
        
    def run(self):
        self.main_window.show_all()

    def get_selected(self):
        (model, pathlist) = self.orders_view.get_selection().get_selected_rows()
        selected = []
        for i in pathlist:
            selected.append( self.orders[i[0]] )
        return selected
            

    def refresh_orders(self):
        try:
            rospy.wait_for_service("get_orders",1)
            get_orders = rospy.ServiceProxy("get_orders", GetOrders)
            resp = get_orders()
            self.orders = resp.orders
            rospy.wait_for_service("get_active_orders",1)
            get_active_orders = rospy.ServiceProxy("get_active_orders", GetActiveOrders)
            resp = get_active_orders()
            self.active_orders = resp.order_ids

            self.orders_list_store.clear()
            for i in self.orders:
                if i.order_id in self.active_orders:
                    self.orders_list_store.append([i.order_id, i.station_id, i.drinks, i.name, "#ffdddd"])
                else:
                    self.orders_list_store.append([i.order_id, i.station_id, i.drinks, i.name, "#ffffff"])
        except:
            rospy.logwarn("web connection services not available.")
    
        
    def refresh_btn_cb(self, btn):
        self.refresh_orders()

    def completed_btn_cb(self,btn):
        selected = self.get_selected()
        try:
            rospy.wait_for_service("get_orders",1)
            mark_order_complete = rospy.ServiceProxy("mark_order_complete", MarkOrderComplete)
            for i in selected:
                resp = mark_order_complete(i.order_id)
            self.refresh_orders()
        except:
            rospy.logwarn("web connection services not available.")

    def battery_cb(self, msg):
        self.battery_label.set_text("Battery voltage: %f"%msg.data)

    def update_drinks_indicator(self, drinks):
        for status, label in zip(drinks, self.drink_labels):
            if status:
                label.set_text("True")
            else:
                label.set_text("False")
            

    def drinks_cb(self, msg):
        self.update_drinks_indicator(msg.status)

    def brake_button_cb(self, btn):
        pass


        

if __name__ == '__main__':
    m = App()
    m.run()
    gtk.gdk.threads_init()
    gtk.main()

