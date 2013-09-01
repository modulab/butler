import rospy

import smach
import smach_ros

#import actionlib
#from actionlib_msgs.msg import *
#from move_base_msgs.msg import *



from monitor_states import *





class GoToStation(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['succeeded']
        )
        
        rospy.sleep(1)


    def execute(self,userdata):    
        rospy.sleep(1)
        return 'succeeded'
        

class AskBottleBack(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes    = ['bottle_back', 'timeout']
        )
        #subscribe to bottle button topic
        rospy.sleep(1)


    def execute(self,userdata):    
        rospy.sleep(1)
        return 'bottle_back'
                


#outcome maps for the concurrence container
#The cild termination callback decided when the concurrence container should be terminated, bases on the outcomes of its children. When it outputs True, the container terminates and the concurrence container outcome callback is called
def child_term_cb_stolen(outcome_map):
    if outcome_map['STOLEN_BOTTLE_MONITOR'] == 'invalid' or outcome_map["GO_TO_STATION"]=="succeeded":
        return True
    return False


#The concurrence container outcome callback maps the outcomes of the container's children into an outcome for the concurrence container itself   
def out_cb_stolen(outcome_map):
   # rospy.sleep(0.1) without this sleep, sometimes the concurrence container terminates before all its children terminate, and an error is printed. However, that does not affect the evolution, and I think that with the sleep sometimes the container blocks and never terminates
    if outcome_map['STOLEN_BOTTLE_MONITOR'] == 'invalid':
        return 'stolen_bottle'
    if  outcome_map["GO_TO_STATION"]=="succeeded":
        return "arrived_to_station"


        
    

    
    
#Move base + recovery behaviour. The number of move_base fails is sent from the move_base action state to the move_base recovery state.
def steal_aware_go_to_station():
    sm=smach.StateMachine(outcomes=['arrived_to_station'])
    
    with sm:
        
        
        

        smach.StateMachine.add('ASK_BOTTLE_BACK', AskBottleBack(),  transitions={'bottle_back':'MONITORED_GO_TO_STATION','timeout':'MONITORED_GO_TO_STATION'})
        
        monitored_go_to_station=smach.Concurrence(outcomes=['arrived_to_station','stolen_bottle'],
                                                    default_outcome='arrived_to_station',
                                                    child_termination_cb=child_term_cb_stolen,
                                                    outcome_cb = out_cb_stolen,
                                                    )
                                                    
        with monitored_go_to_station:
            smach.Concurrence.add('GO_TO_STATION', GoToStation())
            smach.Concurrence.add('STOLEN_BOTTLE_MONITOR',stolen_bottle_monitor())
            
        
        smach.StateMachine.add('MONITORED_GO_TO_STATION', monitored_go_to_station,  transitions={'stolen_bottle':'ASK_BOTTLE_BACK','arrived_to_station':'arrived_to_station'})
        
        
    return sm
        

        
        
        
        
        
        
        
        
#outcome maps for the concurrence container
#The cild termination callback decided when the concurrence container should be terminated, bases on the outcomes of its children. When it outputs True, the container terminates and the concurrence container outcome callback is called
def child_term_cb_cancel(outcome_map):
    if outcome_map['CANCEL_MONITOR'] == 'invalid' or outcome_map['FORCE_COMPLETION_MONITOR'] == 'invalid' or outcome_map['STEAL_AWARE_GO_TO_STATION']=='arrived_to_station':
        return True
    return False


#The concurrence container outcome callback maps the outcomes of the container's children into an outcome for the concurrence container itself   
def out_cb_cancel(outcome_map):
   # rospy.sleep(0.1) without this sleep, sometimes the concurrence container terminates before all its children terminate, and an error is printed. However, that does not affect the evolution, and I think that with the sleep sometimes the container blocks and never terminates
    if outcome_map['CANCEL_MONITOR'] == 'invalid':
        return 'cancelled'
    if  outcome_map['FORCE_COMPLETION_MONITOR'] == 'invalid':
        return "forced_completion"
    if  outcome_map['STEAL_AWARE_GO_TO_STATION']=='arrived_to_station':
        return "arrived_to_station"        


        
def cancelable_go_to_station():
    
    sm=smach.Concurrence(outcomes=['arrived_to_station','cancelled','forced_completion'],
                                                    default_outcome='arrived_to_station',
                                                    child_termination_cb=child_term_cb_cancel,
                                                    outcome_cb = out_cb_cancel,
                                                    )
                                                    
    with sm:
        smach.Concurrence.add('STEAL_AWARE_GO_TO_STATION', steal_aware_go_to_station())
        smach.Concurrence.add('CANCEL_MONITOR',cancel_monitor())
        smach.Concurrence.add('FORCE_COMPLETION_MONITOR',force_completion_monitor())
            
    return sm    

          

    
    
    
    
    
    
    
    
