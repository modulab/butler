#!/usr/bin/env python
import roslib; roslib.load_manifest('drink_sensor')
import rospy
from std_msgs.msg import String
from drink_sensor.msg import DrinksStatus
from drink_sensor.srv import *
import serial

# The actual caddy has some inputs tied low, others hight
# mask  not xor with this
MASK =  [True, True, True, False, False, False,  True,  True] 


class DrinkSensor(object):
    def __init__(self):
        rospy.init_node('drink_sensor')
        self._drinks_pub = rospy.Publisher('drinks_status', DrinksStatus)

        serial_port = rospy.get_param("serial_port","/dev/ttyACM0")
        self._serial = serial.Serial(serial_port)

        self._requested_digital = False
        self._requested_digital_srv = rospy.Service("request_drinks_status",RequestDrinksStatus, self.request_digital)

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
                        
    def publish_state(self):
        state = DrinksStatus()
        for n in range(0,8):
            state.status.append((self.digital_state >> n) & 1)
        state.status =  map(lambda x: not (x[0] ^ x[1]), zip(MASK,state.status)) 
        self._drinks_pub.publish(state)


    def request_digital(self, req):
        self._requested_digital=True
        self._serial.write("!R")
        while (self._requested_digital):
            rospy.sleep(0.1)
        state = DrinksStatus()
        for n in range(0,8):
            state.status.append((self.digital_state >> n) & 1)
        state.status =  map(lambda x: not (x[0] ^ x[1]), zip(MASK,state.status)) 

        resp = RequestDrinksStatusResponse(state)
        return resp
        

if __name__ == '__main__':
    try:
        d = DrinkSensor()
    except rospy.ROSInterruptException:
        pass
