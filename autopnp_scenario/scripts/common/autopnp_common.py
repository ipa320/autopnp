#!/usr/bin/python
import roslib
roslib.load_manifest('autopnp_scenario')
import rospy
import os
import threading

import tf

class ScreenFormat:
    # Convenient functions, which should be called when entering and leaving a state
    # to improve readability of the debug output on the console
    
    def __init__(self, text):
        self.text = text
        self.log_enter_state(text)
        
    def __del__(self):
        self.log_exit_state(self.text)
    
    def log_enter_state(self, text):
    
        #HEADER = '\033[95m'
        color = '\033[94m'
        #OKGREEN = '\033[92m'
        #WARNING = '\033[93m'
        #FAIL = '\033[91m'
        #ENDC = '\033[0m'
    
        out_str = ''
        for num in range(0, len(text)):
            out_str += '-'
    
        rospy.loginfo('%s\n', color)
        rospy.loginfo("%s----------------------------------------------", out_str)
        rospy.loginfo("- ENTERING \"%s\" --------------------------------\033[0m", text)
    
    
    def log_exit_state(self, text):
        color = '\033[94m'
        out_str = ''
        for num in range(0, len(text)):
            out_str += '-'
        rospy.loginfo('%s', color)
        rospy.loginfo("- LEAVING \"%s\" ---------------------------------", text)
        rospy.loginfo("%s----------------------------------------------\n\033[0m", out_str)


###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
_tl=None
_tl_creation_lock=threading.Lock()

def get_transform_listener():
	global _tl
	with _tl_creation_lock:
		if _tl==None:
			_tl=tf.TransformListener(True, rospy.Duration(40.0))
		return _tl
#################################################################################


def sign(val):
	if val<0:
		return -1.0
	elif val>0:
		return 1.0
	return 0
