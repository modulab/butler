import rospy

import smach
import smach_ros

from monitor_states import *

from drink_sensor.srv import * 

import sm_global_data as application
from std_msgs.msg import String

"""
Low level state that navigates to a station
"""
class GoToStation(smach.State):
    def __init__(self,  to_base=False):
        smach.State.__init__(self,
                             outcomes    = ['succeeded']
                             )
        self.going_to_base = to_base
        
        #rospy.sleep(1)


    def execute(self,userdata):
        if not self.going_to_base:
            application.app_data.status_publisher.publish("Traveling to QR code.")
        else:
            application.app_data.status_publisher.publish("Returning to base.")
        for i in range(100):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.05)
        return 'succeeded'
        
"""
Low level state that says "Return drinks" and waits until drink sensor
has the correct number of drinks. Times out.
"""
class AskBottleBack(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes    = ['bottle_back', 'timeout']
                             )

        # service to query drinks status
        try:
            rospy.wait_for_service("request_drinks_status", 4)
        except:
            rospy.logerr("Can't find drink sensor services!")
            sys.exit(1)
            
        self.request_drinks_status = rospy.ServiceProxy("request_drinks_status",
                                                        RequestDrinksStatus)
        

    def execute(self, userdata):
        application.app_data.status_publisher.publish("Trying to get bottle back")
        application.app_data.talk_service(String("Please return that drink!"))

        # 
        for i in range(200): # 20 seconds timeout
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
                
        
        # should adjust the current orders to only contain the ones
        # that still have enough drinks for
        application.app_data.status_publisher.publish("Timed out, lost drinks :-(")

        return 'timeout'
                
"""
A MonitoredGoToStation state executes the traveling to the goal at the same time
as checking that a drink is not stolen.
Outcomes 'arrived_at_station' if good, if a bottle is stolen then GoToStation is
canceld and the outcome is 'stolen_bottle'.
"""
class MonitoredGoToStation(smach.Concurrence):
    def __init__(self):
        smach.Concurrence.__init__(self, outcomes=['arrived_to_station',
                                                   'stolen_bottle'],
                                   default_outcome='arrived_to_station',
                                   child_termination_cb=self.child_term_cb_stolen,
                                   outcome_cb = self.out_cb_stolen,
                                   )
        with self:
            smach.Concurrence.add('GO_TO_STATION', GoToStation())
            smach.Concurrence.add('STOLEN_BOTTLE_MONITOR',StolenBottleMonitor())

    def child_term_cb_stolen(self, outcome_map):
        # decide if this state is done when one or more concurrent inner states 
        # stop
        if outcome_map['STOLEN_BOTTLE_MONITOR'] == 'invalid' or outcome_map["GO_TO_STATION"]=="succeeded":
            return True
        return False
    
    def out_cb_stolen(self, outcome_map):
        # determine what the outcome of this machine is
        if outcome_map['STOLEN_BOTTLE_MONITOR'] == 'invalid':
            return 'stolen_bottle'
        if  outcome_map["GO_TO_STATION"]=="succeeded":
            return "arrived_to_station"


"""
Steal aware navigation to station. MonitoredGoToStation <-> AskBottleBack
Outcome -> arrived_to_station
"""
class StealAwareGoToStation(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['arrived_to_station'])
        
        with self:
            smach.StateMachine.add('ASK_BOTTLE_BACK', AskBottleBack(),
                                   transitions={'bottle_back':'MONITORED_GO_TO_STATION',
                                                'timeout':'MONITORED_GO_TO_STATION'})
        
            monitored_go_to_station = MonitoredGoToStation()
        
            smach.StateMachine.add('MONITORED_GO_TO_STATION', monitored_go_to_station,
                                   transitions={'stolen_bottle':'ASK_BOTTLE_BACK',
                                                'arrived_to_station':'arrived_to_station'})
        
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
        if outcome_map['CANCEL_MONITOR'] == 'invalid' or outcome_map['FORCE_COMPLETION_MONITOR'] == 'invalid' or outcome_map['STEAL_AWARE_GO_TO_STATION']=='arrived_to_station':
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


    
    
    
    
    
    
    
    
