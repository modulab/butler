#!/usr/bin/env python
import roslib; roslib.load_manifest('service_manager')
import rospy
from web_connector.srv import *
from std_msgs.msg import Float32, Bool, String
from drink_sensor.msg import DrinksStatus
from drink_sensor.srv  import *

import pygtk
import gtk
import gobject

class App(object):
    def __init__(self):
        self.node = rospy.init_node("service_manager")
        
        self._init_gui()


        

    def _init_gui(self):
        self.main_window = gtk.Window()
        self.main_window.set_title("Service Manager")
        self.main_window.connect('destroy', gtk.main_quit)
        self.main_window.set_size_request(800, 600)
        vbox = gtk.VBox(False, 8)
        hbox = gtk.HBox()
        hbox.pack_start(vbox)

        robot_status = RobotStatusDisplay() 
        vbox.pack_start(robot_status, False, False)

        orders_status = OrdersDisplay() 
        vbox.pack_start(orders_status, True, True)

        
        drinks_status = DrinksSensorDisplay() #gtk.Frame("Drink Sensors")
        #self.drink_labels=[gtk.Label(),gtk.Label(),gtk.Label(),gtk.Label(),
                           #gtk.Label(),gtk.Label(),gtk.Label(),gtk.Label()]
        #drinks_v=gtk.VBox(False,False)
        #drinks_top_row = gtk.HBox(False,False)
        #drinks_bottom_row = gtk.HBox(False,False)
        #drinks_v.pack_start(drinks_top_row)
        #for i in range(0,4):
            #drinks_top_row.pack_start(self.drink_labels[i], False, False)
        #drinks_v.pack_start(drinks_bottom_row)
        #for i in range(4,8):
            #drinks_bottom_row.pack_start(self.drink_labels[i], False, False)
        #drinks_status.add(drinks_v)
        vbox.pack_start(drinks_status, False, False)

        # status messages
        vbox_r =  gtk.VBox()
        self.status_display =  StatusMessageDisplay("/buttler_status_messages")
        
        # state machine control buttons
        btn_box =  gtk.HBox()
        loaded =  BooleanPublishButton("Go", "/remote_buttons/go", False)
        btn_box.pack_start(loaded)
        cancel =  BooleanPublishButton("Cancel trip.", "/remote_buttons/cancel", False)
        btn_box.pack_start(cancel)
        done =  BooleanPublishButton("Flag done.", "/remote_buttons/mark_done", False)
        btn_box.pack_start(done)
        vbox_r.pack_start(self.status_display)
        vbox_r.pack_start(btn_box,  False,  False)
        hbox.pack_start(vbox_r)
                
        
        self.main_window.add(hbox)
    
        
    def run(self):
        self.main_window.show_all()

        
class DrinksSensorDisplay(gtk.Frame):
    def __init__(self):
        gtk.Frame.__init__(self, "Drink Sensors")
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
        self.add(drinks_v)
        
        self.drink_status_sub = rospy.Subscriber("/drinks_status",DrinksStatus, self.drinks_cb)
        
        # Update the drink sensor display
        try:
            rospy.wait_for_service("request_drinks_status", 1)
            self.get_drink_status_srv = rospy.ServiceProxy("request_drinks_status",RequestDrinksStatus)
            status = self.get_drink_status_srv().status.status
            self.update_drinks_indicator(status)
        except:
            rospy.logwarn("Couldn't call drink sensor service.")

   
    def update_drinks_indicator(self, drinks):
        for status, label in zip(drinks, self.drink_labels):
            if status:
                label.set_text("True")
            else:
                label.set_text("False")
            

    def drinks_cb(self, msg):
        self.update_drinks_indicator(msg.status)

class RobotStatusDisplay(gtk.Frame):
    def __init__(self):
        gtk.Frame.__init__(self,  "Robot status")
        robot_status_v = gtk.VBox()
        self.battery_label = gtk.Label("Battery voltage: no data!")
        self.brake_button = gtk.ToggleButton("Brakes disabled")
        self.brake_button.connect("clicked",self.brake_button_cb)
        robot_status_v.pack_start(self.battery_label,False,False)
        robot_status_v.pack_start(self.brake_button, False, False)
        self.add(robot_status_v)
        
        self.battery_sub = rospy.Subscriber("/b21/voltage",Float32,self.battery_cb)


    def brake_button_cb(self, btn):
        pass

    def battery_cb(self, msg):
        self.battery_label.set_text("Battery voltage: %f"%msg.data)
        
class OrdersDisplay(gtk.Frame):
    def __init__(self):
        gtk.Frame.__init__(self, "Orders")
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
        self.add(orders_status_v)
        
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


class StatusMessageDisplay(gtk.Frame):
    def __init__(self,  topic):
        gtk.Frame.__init__(self, "Status Messages")
        
        self.status_text =  gtk.TextBuffer()
        self.textbox =  gtk.TextView(self.status_text)
        self.textbox.set_editable(False)
        sw = gtk.ScrolledWindow()
        sw.set_shadow_type(gtk.SHADOW_ETCHED_IN)
        sw.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_AUTOMATIC)
        sw.add(self.textbox)
        self.add(sw)
        self.scrolling = True
        
        self._status_sub =  rospy.Subscriber(topic, String,  self.status_cb)
    
    def set_autoscroll(self, scroll_enable):
        self.scrolling =  scroll_enable
        
    def clear(self):
        self.status_text.set_text("")
    
    def add_message(self, message):
        ti = self.status_text.get_end_iter()
        self.status_text.insert(ti, message+"\n")
        if self.scrolling:
            self.textbox.scroll_to_iter(ti, 0.1)
            
    def status_cb(self, msg):
        self.add_message(msg.data)
#
# Buttons that connect to ROS
class RemoteEnabledDisabledButton(gtk.Button):
    def __init__(self,  text, enable_disable_topic, enabled=True):
        gtk.Button.__init__(self, text)
        self.set_sensitive(enabled)
        self._enable_disable_sub =  rospy.Subscriber(enable_disable_topic, Bool,
                                                     self._disable_cb)
        
    def _disable_cb(self, msg):
        self.set_sensitive(msg.data)
            
class BooleanPublishButton(RemoteEnabledDisabledButton):
    def __init__(self, text, button_topic, enabled=True):
        RemoteEnabledDisabledButton.__init__(self, text, button_topic+"/enable", enabled)
        self._publisher =  rospy.Publisher(button_topic+"/callback", Bool)
        self.connect("clicked", self._click_cb)
    
    def _click_cb(self, btn):
        self._publisher.publish(True)

if __name__ == '__main__':
    m = App()
    m.run()
    gtk.gdk.threads_init()
    gtk.main()

