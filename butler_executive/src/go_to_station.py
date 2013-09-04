import rospy

import smach
import smach_ros

from monitor_states import *

from drink_sensor.srv import * 

import sm_global_data as application
from std_msgs.msg import String, Int32, Bool, Float32
from sensor_msgs.msg import Joy
from time import time

from order_states import GetAndMarkOrders


"""
Low level state that navigates to a station
"""
class GoToStation(smach.State):
    def __init__(self,  to_base=False):
        smach.State.__init__(self,
                             outcomes    = ['succeeded']
                             )
        self.going_to_base = to_base
        self.crowded_nav_go_pub =  rospy.Publisher("/crowded_nav/go", Int32)
        self.crowded_nav_stop_pub =  rospy.Publisher("/crowded_nav/stop", Bool)
        
        # Create a publisher for log status
        self.status_pub = rospy.Publisher("/buttler_status_messages",  String)
        
        
        self.crowded_nav_result_sub =  rospy.Subscriber("/crowded_nav/result",
                                                        Bool,
                                                        self.result_cb)
        self.crowded_nav_feedback_sub =  rospy.Subscriber("/crowded_nav/feedback",
                                                        String,
                                                        self.feedback_cb)
        
        self._executing = False # should we ignore the callbacks - is the state in exec


    def execute(self,userdata):
        if not self.going_to_base:
            station_id =  int(application.app_data.order_list[0].station_id)
            self.status_pub.publish("Traveling to QR code ." + str(station_id))
        else:
            self.status_pub.publish("Returning to base.")
            station_id =  0
            
        self.status =  None
        
        
        return_state = 'succeeded'
        self._executing = True
        self.crowded_nav_go_pub.publish(station_id)
        while True:
            if self.preempt_requested():
                self.service_preempt()
                self.crowded_nav_stop_pub.publish(True)
                #return_state = 'preempted'
                break
            if self.status is not None:
                if self.status:
                    return_state = 'succeeded'
                    break
                else:
                    self.status_pub.publish("MOVE FAILED: try to joystick me.")
                    rospy.sleep(0.5)
            rospy.sleep(0.1)

        self._executing = False
        return return_state
    
    def result_cb(self, msg):
        self.status = msg.data
    
    def feedback_cb(self, msg):
        if self._executing:
            self.status_pub.publish("[Crowded_Nav] " + msg.data)
        
    
"""
Concurrent state that GoToStation, but is prempted by a joystick overtake request
"""
class JoystickMonitoredGotoStation(smach.Concurrence):
    def __init__(self,  to_base=False):
        smach.Concurrence.__init__(self, outcomes=['succeeded',
                                                   'joystick_takeover'],
                                   default_outcome='succeeded',
                                   child_termination_cb=self.child_term_cb_stolen,
                                   outcome_cb = self.out_cb_stolen,
                                   )
        with self:
            smach.Concurrence.add('GO_TO_STATION', GoToStation(to_base))
            smach.Concurrence.add('JOYSTICK_TAKEOVER_MONITOR',
                                  ButtonMonitor("/remote_buttons/joystick"))

    def child_term_cb_stolen(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        if (outcome_map['JOYSTICK_TAKEOVER_MONITOR'] == 'invalid' or
            outcome_map["GO_TO_STATION"]=="succeeded"):
            return True
        return False
    
    def out_cb_stolen(self, outcome_map):
        # determine what the outcome of this machine is
        if outcome_map['JOYSTICK_TAKEOVER_MONITOR'] == 'invalid':
            return 'joystick_takeover'
        if  outcome_map["GO_TO_STATION"]=="succeeded":
            return "succeeded"
        
#"""
#State that robot is in when in joystick control mode. Waits for authorisation to
#'succeed' -> done GoToStation , or 'return_control' -> continue with GoToStation
#"""
#class JoystickControl(smach.State):
    #def __init__(self):
        #smach.State.__init__(self,
                             #outcomes    = ['succeeded', 'return_control']
                             #)

    #def execute(self,userdata):
        #application.app_data.status_publisher.publish("Under joystick control..")
        ## wait for button press to indicate return control
        ## or succeded
        #return 'succeeded'
    
"""
Monitor the joystick buttons, if user wants to handback control or mark sucess
then exit as desired.
"""
class JoystickModeMonitor(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                         outcomes    = ['succeeded', 'handback']
                         )
        self.joy_sub =  rospy.Subscriber('/joy', Joy,  self.joy_cb)
        self.should_succeed = False
        self.should_handback = False

    def execute(self,userdata):
        application.app_data.status_publisher.publish("Under joystick control..")
        # wait for button press to indicate return control
        # or succeded
        self.should_succeed = False
        self.should_handback = False
        while True:
            if self.preempt_requested():
                self.service_preempt()
                return 'succeeded'
            if self.should_succeed:
                return 'succeeded'
            if self.should_handback:
                return 'handback'
            rospy.sleep(0.1)
            
    def joy_cb(self, msg):
        if msg.buttons[0] == 1:    # Button 'A' on Rumblepad 710
            self.should_succeed = True
        elif msg.buttons[2] ==  1:  # Button 'X' on Rumblepad 710
            self.should_handback = True
        

""" State when in joystick control mode. Concurrently monitors some remote GUI buttons
and the joystic buttons to decide when to leave.
'succeed' -> done GoToStation , or 'return_control' -> continue with GoToStation
"""
class JoystickControl(smach.Concurrence):
    def __init__(self,  to_base=False):
        smach.Concurrence.__init__(self, outcomes=['succeeded',
                                                   'return_control'],
                                   default_outcome='succeeded',
                                   child_termination_cb=self.child_term_cb_stolen,
                                   outcome_cb = self.out_cb_stolen,
                                   )
        with self:
            smach.Concurrence.add('JOYSTICK_HANDBACK_MONITOR',
                                  ButtonMonitor('/remote_buttons/joystick_handback'))
            smach.Concurrence.add('JOYSTICK_SUCCESS_MONITOR',
                                  ButtonMonitor("/remote_buttons/joystick_success"))
            smach.Concurrence.add('JOYSTICK_MODE_MONITOR',
                                  JoystickModeMonitor())

    def child_term_cb_stolen(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        if (outcome_map['JOYSTICK_HANDBACK_MONITOR'] == 'invalid' or
            outcome_map['JOYSTICK_SUCCESS_MONITOR'] == 'invalid' or
            outcome_map["JOYSTICK_MODE_MONITOR"]=="succeeded" or
            outcome_map["JOYSTICK_MODE_MONITOR"]=="handback"):
            return True
        return False
    
    def out_cb_stolen(self, outcome_map):
        # determine what the outcome of this machine is
        if (outcome_map['JOYSTICK_HANDBACK_MONITOR'] == 'invalid' or
            outcome_map["JOYSTICK_MODE_MONITOR"]=="handback"):
            return 'return_control'
        if  (outcome_map["JOYSTICK_SUCCESS_MONITOR"]=="invalid" or
             outcome_map["JOYSTICK_MODE_MONITOR"]=="succeeded"):
            return "succeeded"
        
    
"""
The sub state machine implementing a joystick overidable GoToStation state.
"""
class JoystickOverideableGoToStation(smach.StateMachine):
    def __init__(self, to_base=False):
        smach.StateMachine.__init__(self, outcomes=['succeeded'])
        
        with self:
            smach.StateMachine.add('GOTO_STATION', JoystickMonitoredGotoStation(to_base),
                                   transitions={'joystick_takeover':'JOYSTICK_CONTROL',
                                                'succeeded':'succeeded'})
        
            smach.StateMachine.add('JOYSTICK_CONTROL', JoystickControl(),
                                   transitions={'return_control':'GOTO_STATION',
                                                'succeeded':'succeeded'})
        
        
"""
Low level state that says "Return drinks" and waits until drink sensor
has the correct number of drinks. Times out after 'timeout' seconds, default 15.
"""
class AskBottleBack(smach.State):
    def __init__(self, timeout=15):
        smach.State.__init__(self,
                             outcomes    = ['bottle_back', 'timeout']
                             )

        # service to query drinks status
        try:
            rospy.wait_for_service("request_drinks_status", 4)
        except:
            rospy.logerr("Can't find drink sensor services!")
            sys.exit(1)
            
        self.timeout = timeout
            
        self.request_drinks_status = rospy.ServiceProxy("request_drinks_status",
                                                        RequestDrinksStatus)
        

    def execute(self, userdata):
        application.app_data.status_publisher.publish("Trying to get bottle back")
        application.app_data.talk_service(String("Please return that drink!"))

        # 
        timed_out = False
        start_time = time()
        while not timed_out:
#        for i in range(int(self.timeout*10)):
            if self.preempt_requested():
                self.service_preempt()
                return 'timeout'
            response =  self.request_drinks_status()
            status =  response.status.status
            # check the number of drinks
            n_drinks = sum([1 for x in status if x ])
            if n_drinks == application.app_data.n_drinks:
                application.app_data.status_publisher.publish("Got it back!")
                return 'bottle_back'
            rospy.sleep(0.1)
            
            if (time() - start_time) >  self.timeout:
                timed_out = True
                
        
        # should adjust the current orders to only contain the ones
        # that still have enough drinks for
        application.app_data.status_publisher.publish("Timed out, lost drinks :-(")

        return 'timeout'
                
"""
A BottleMonitoredGoToStation state executes the traveling to the goal at the same time
as checking that a drink is not stolen.
Outcomes 'arrived_at_station' if good, if a bottle is stolen then JoystickOverideableGoToStation is
canceld and the outcome is 'stolen_bottle'.
"""
class BottleMonitoredGoToStation(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(self, outcomes=['arrived_to_station',
                                                   'stolen_bottle'],
                                   default_outcome='arrived_to_station',
                                   child_termination_cb=self.child_term_cb_stolen,
                                   outcome_cb = self.out_cb_stolen,
                                   )
        with self:
            smach.Concurrence.add('GO_TO_STATION', JoystickOverideableGoToStation())
            smach.Concurrence.add('STOLEN_BOTTLE_MONITOR',StolenBottleMonitor())
        
        # subscribe to crowded navigation distance to maintain knowledge of
        # goal distance
        self.distance_to_goal =  1000.0
        self.goal_distance_sub =  rospy.Subscriber('/crowded_nav/distance',
                                                   Float32,
                                                   self.goal_distance_cb)

    def child_term_cb_stolen(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        if outcome_map['STOLEN_BOTTLE_MONITOR'] == 'invalid' or outcome_map["GO_TO_STATION"]=="succeeded":
            return True
        return False
    
    def out_cb_stolen(self, outcome_map):
        # determine what the outcome of this machine is
        if outcome_map['STOLEN_BOTTLE_MONITOR'] == 'invalid':
            # if a bottle was stolen, but we are close to the station,
            # then go to 'arived' and proceed as normal; otherwise request return
            # threading issues?
            if self.distance_to_goal <  0.5:
                return 'arrived_to_station'
            else:
                return 'stolen_bottle'
        if  outcome_map["GO_TO_STATION"]=="succeeded":
            return "arrived_to_station"

    def goal_distance_cb(self, msg):
        self.distance_to_goal = msg.data

"""
Steal aware navigation to station. BottleMonitoredGoToStation <-> AskBottleBack
Outcome -> arrived_to_station
"""
class StealAwareGoToStation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['arrived_to_station',
                                                    'cancelled'])
        
        with self:
            smach.StateMachine.add('ASK_BOTTLE_BACK', AskBottleBack(),
                                   transitions={'bottle_back':'MONITORED_GO_TO_STATION',
                                                'timeout':'GET_AND_MARK_ORDERS'})
        
            monitored_go_to_station = BottleMonitoredGoToStation()
        
            smach.StateMachine.add('MONITORED_GO_TO_STATION', monitored_go_to_station,
                                   transitions={'stolen_bottle':'ASK_BOTTLE_BACK',
                                                'arrived_to_station':'arrived_to_station'})
            
            smach.StateMachine.add('GET_AND_MARK_ORDERS', GetAndMarkOrders(already_carrying=True),
                               transitions={'succeeded':'MONITORED_GO_TO_STATION',
                                            'no_orders': 'cancelled',
                                            'not_enough_beers': 'cancelled',}) 
        
        self.set_initial_state(["MONITORED_GO_TO_STATION"])
        
"""
CancelableGoToStation is concurrent StealAwareGoToStation and some monitors for
cancelation of motion, forced completion or joytick takeover.
"""
class CancelableGoToStation(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(self, outcomes=['arrived_to_station',
                                                   'cancelled',
                                                   'forced_completion'],
                                         default_outcome='arrived_to_station',
                                         child_termination_cb=self.child_term_cb_cancel,
                                         outcome_cb = self.out_cb_cancel,
                                         )
        with self:
            smach.Concurrence.add('STEAL_AWARE_GO_TO_STATION', StealAwareGoToStation())
            smach.Concurrence.add('CANCEL_MONITOR', ButtonMonitor("/remote_buttons/cancel") ) #cancel_monitor())
            smach.Concurrence.add('FORCE_COMPLETION_MONITOR', ButtonMonitor("/remote_buttons/mark_done") ) #force_completion_monitor())
            
    def child_term_cb_cancel(self, outcome_map):
         # decide if this state is done when one or more concurrent inner states 
        # stop
        if (outcome_map['CANCEL_MONITOR'] == 'invalid' or
            outcome_map['FORCE_COMPLETION_MONITOR'] == 'invalid' or
            outcome_map['STEAL_AWARE_GO_TO_STATION']=='arrived_to_station' or
            outcome_map['STEAL_AWARE_GO_TO_STATION']=='cancelled'):
            return True
        return False
    
    def out_cb_cancel(self, outcome_map):
        # determine what the outcome of this machine is
        if outcome_map['CANCEL_MONITOR'] == 'invalid':
            return 'cancelled'
        if  outcome_map['FORCE_COMPLETION_MONITOR'] == 'invalid':
            return "forced_completion"
        if  outcome_map['STEAL_AWARE_GO_TO_STATION']=='arrived_to_station':
            return "arrived_to_station"        
        if  outcome_map['STEAL_AWARE_GO_TO_STATION']=='cancelled':
            return "cancelled"        


    
    
    
    
    
    
    
    
