import rospy

#class that can only have a single instance. It is used to "communicate" battery tresholds between long_term_patroller.py (which dynamically reconfigures the tresholds and uses them for point selection) and monitor_states.py(which uses them for the battery monitoring)
class GlobalData(object):
    _instance = None
    def __init__(self):
        self.n_orders=0
        self.order_list=0
        
      

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super(GlobalData, cls).__new__(
                cls, *args, **kwargs)

        return cls._instance
        
app_data =  GlobalData()
    
