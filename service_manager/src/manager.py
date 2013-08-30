#!/usr/bin/env python
import roslib; roslib.load_manifest('service_manager')
import rospy
from web_connector.srv import *
from std_msgs.msg import Float32


import pygtk
import gtk
import gobject

class App(object):
    def __init__(self):
        self.main_window = gtk.Window()
        self.main_window.set_title("Service Manager")
        self.main_window.connect('destroy', gtk.main_quit)
        self.main_window.set_size_request(400, 200)
        vbox = gtk.VBox(False, 8)

        self.battery_label = gtk.Label("Battery voltage=")
        vbox.pack_start(self.battery_label,False,False)
        
        sw = gtk.ScrolledWindow()
        sw.set_shadow_type(gtk.SHADOW_ETCHED_IN)
        sw.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        
        vbox.pack_start(sw, True, True, 0)
        self.button_box = gtk.HBox()

        vbox.pack_start(self.button_box, False, False)

        refresh_btn = gtk.Button("Refresh")
        refresh_btn.connect("clicked",self.refresh_btn_cb)
        self.button_box.pack_start(refresh_btn)

        completed_btn = gtk.Button("Mark Complete")
        completed_btn.connect("clicked",self.completed_btn_cb)
        self.button_box.pack_start(completed_btn)


        self.orders_list_store = gtk.ListStore(str, str, str, str, str)
        self.orders_list_store.append(["0","0","0","0", "#FFFFFF"])
        self.orders_view = gtk.TreeView(self.orders_list_store)
        self.orders_view.get_selection().set_mode(gtk.SELECTION_MULTIPLE)


        for num,i in enumerate(["Order Numer", "Station", "Order", "Customer Name"]):
            renderText = gtk.CellRendererText()
            column = gtk.TreeViewColumn(i, renderText, text=num, background=4)
            self.orders_view.append_column(column)
 
        sw.add(self.orders_view)

        self.main_window.add(vbox)

        self.node = rospy.init_node("service_manager")
        self.battery_sub = rospy.Subscriber("/b21/voltage",Float32,self.battery_cb)
 
        
    def run(self):
        self.main_window.show_all()

    def get_selected(self):
        (model, pathlist) = self.orders_view.get_selection().get_selected_rows()
        selected = []
        for i in pathlist:
            print self.orders[i[0]]
            selected.append( self.orders[i[0]] )
        return selected
            

    def refresh_orders(self):
        rospy.wait_for_service("get_orders")
        get_orders = rospy.ServiceProxy("get_orders", GetOrders)
        resp = get_orders()
        self.orders = resp.orders
        rospy.wait_for_service("get_active_orders")
        get_active_orders = rospy.ServiceProxy("get_active_orders", GetActiveOrders)
        resp = get_active_orders()
        self.active_orders = resp.order_ids
        
        self.orders_list_store.clear()
        for i in self.orders:
            if i.order_id in self.active_orders:
                self.orders_list_store.append([i.order_id, i.station_id, i.drinks, i.name, "#ffdddd"])
            else:
                self.orders_list_store.append([i.order_id, i.station_id, i.drinks, i.name, "#ffffff"])

    def refresh_btn_cb(self, btn):
        self.refresh_orders()

    def completed_btn_cb(self,btn):
        selected = self.get_selected()
        rospy.wait_for_service("get_orders")
        mark_order_complete = rospy.ServiceProxy("mark_order_complete", MarkOrderComplete)
        for i in selected:
            resp = mark_order_complete(i.order_id)
        self.refresh_orders()

    def battery_cb(self, msg):
        self.battery_label.set_text("Battery voltage=%f"%msg.data)

        

if __name__ == '__main__':
    m = App()
    m.run()
    gtk.gdk.threads_init()
    gtk.main()

