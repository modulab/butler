#! /usr/bin/env python3.3

import sys
from PySide import QtGui, QtCore
from drink_sensor.msg import DrinksStatus
from drink_sensor.srv import *

import roslib
import rospy
import pymorse
from std_msgs.msg import Bool

class ControlGUI(QtGui.QWidget):
    
    def __init__(self):
        super(ControlGUI, self).__init__()
        self._drink_state = [False] * 8

        self._drinks_pub = rospy.Publisher('drinks_status', DrinksStatus)
        self._requested_digital_srv = rospy.Service("request_drinks_status",
                                                    RequestDrinksStatus,
                                                    self.request_digital)
        self._set_debounce_srv = rospy.Service("set_debounce",
                                                    SetDebounce,
                                                    self.set_debounce)

            
        self.initUI()

        
    def request_digital(self, req):
        state = DrinksStatus()
        state.status = self._drink_state
        resp = RequestDrinksStatusResponse(state)
        
        return resp

    def set_debounce(self,req):
        return SetDebounceResponse()
    

    def publish_state(self):
        self._drinks_pub.publish(self._drink_state)


        
    def initUI(self):
        sensor_buttons=[QtGui.QPushButton("{}".format(i)) for i in range(0,4)]
 
        hbox = QtGui.QHBoxLayout()
        hbox.addStretch(1)
        for i in sensor_buttons:
            i.setCheckable(True)
            i.clicked[bool].connect(self._button_click)
            hbox.addWidget(i)
        hbox.addStretch(1)
        
        self.setLayout(hbox)    
        
        self.setGeometry(300, 300, 300, 150)
        self.setWindowTitle('Drink Sensors Simulation GUI')    
        self.show()
        
    def _button_click(self,show):
         sender = self.sender()
         sensor=int(sender.text())
         self._drink_state[sensor]=not self._drink_state[sensor]
         self.publish_state()
         print ("Toggled sensor {}".format(sensor))


if __name__ == '__main__':
    rospy.init_node("DrinkSensorGui")
    app = QtGui.QApplication(sys.argv)
    ex = ControlGUI()
    sys.exit(app.exec_())
