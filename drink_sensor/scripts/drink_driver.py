#!/usr/bin/env python
import roslib; roslib.load_manifest('drink_sensor')
import rospy
from std_msgs.msg import String
from drink_sensor.msg import DrinksStatus
from drink_sensor.srv import *
import serial
import sys

# The actual caddy has some inputs tied low, others hight
# mask  not xor with this
MASK =  [True]*8
# They are also wired in a strange order, re order as
REORDER = [3, 4, 5, 1, 2, 0]

class DrinkSensor(object):
    def __init__(self):
        rospy.init_node('drink_sensor')
        self._drinks_pub = rospy.Publisher('drinks_status', DrinksStatus)

        serial_port = rospy.get_param("serial_port","/dev/ttyUSB0")
        self._serial = serial.Serial(serial_port)
        rospy.loginfo("Waiting for serial port")
        while not self._serial.isOpen():
            pass        
        rospy.loginfo("Opened serial "+ serial_port)
        
        # status = self._serial.read(2) # read the initial values
        # if status != "!G": # init string
        #     rospy.logerr("Error initialising drink sensor.")
        #     rospy.logerr("Got message "+status)
        #     sys.exit(1)
        
        self._serial.write("!G")
        status = self._serial.read(2) # read the initial values
        if status != "!G": # init string
            rospy.logerr("Error initialising drink sensor.")
            rospy.logerr("Got message "+status)
            sys.exit(1)

        self._requested_digital = False
        self._requested_digital_srv = rospy.Service("request_drinks_status",
                                                    RequestDrinksStatus,
                                                    self.request_digital)
        self._requested_digital_srv = rospy.Service("set_debounce",
                                                    SetDebounce,
                                                    self.set_debounce)

        rospy.loginfo("Setting default debounce period 800ms")
        #rospy.sleep(1)
        self._serial.write("B"+chr(80))
        
        rospy.loginfo("Driver going into main loop")
        self.main()

    def main(self):
        while not rospy.is_shutdown():
            status = self._serial.read(2)
            if status[0]=='E':
                # Error
                rospy.logerr("drink sensor device reported error, code " +
                             status[1])
            elif status[0] == 'D':
                # Some digital values
                self.digital_state = ord(status[1])
                if not self._requested_digital:
                    # not just requested so must be a change update
                    self.publish_state()
                else:
                    self._requested_digital = False
            elif status[0] == 'B':
                # Debounce set confirmation
                rospy.loginfo("Debounce set to " + str(ord(status[1])*10 )+"ms")

                        
    def publish_state(self):
        state = DrinksStatus()
        for n in range(0,8):
            state.status.append((self.digital_state >> n) & 1)
        #state.status =  map(lambda x: not (x[0] ^ x[1]), zip(MASK,state.status))
        #reorder =  map(lambda x: state.status[x],  REORDER)
        #state.status = reorder
        self._drinks_pub.publish(state)


    def request_digital(self, req):
        self._requested_digital=True
        self._serial.write("!R")
        while (self._requested_digital):
            rospy.sleep(0.1)
        state = DrinksStatus()
        for n in range(0,8):
            state.status.append((self.digital_state >> n) & 1)
        #state.status =  map(lambda x: not (x[0] ^ x[1]), zip(MASK,state.status)) 
        #state.status =  map(lambda x: state.status[x],  REORDER)

        resp = RequestDrinksStatusResponse(state)
        return resp


    def set_debounce(self, req):
        val =  req.value.data # in milliseconds, need in 10th
        val =  val / 10
        if val > 255:
            val = 255
        if val < 0:
            val = 0
        self._serial.write("B"+chr(val))
        return SetDebounceResponse()
        

if __name__ == '__main__':
    try:
        d = DrinkSensor()
    except rospy.ROSInterruptException:
        pass
