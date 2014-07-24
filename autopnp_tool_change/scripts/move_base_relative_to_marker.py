#!/usr/bin/python

import sys, os

import roslib; roslib.load_manifest('autopnp_tool_change') # ; roslib.load_manifest('cob_navigation_global')
import rospy
import tf

from geometry_msgs.msg import PoseStamped
from cob_object_detection_msgs.msg import DetectionArray, Detection

from simple_script_server import simple_script_server
sss = simple_script_server()

class move_base_relative_to_marker():
	def __init__(self):
		self.last_fiducial_pose = PoseStamped()

		# subscribe to fiducials topic
		self.fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)

	def fiducial_callback(self, fiducials):
		for fiducial in fiducials.detections:
			if fiducial.label == "tag_73":
				print "-----------------------tag 73 found"
				print self.last_fiducial_pose
				self.last_fiducial_pose = fiducial.pose
				print self.last_fiducial_pose
				
	def execute(self):
		# move torso to right position
		# sss.move("torso","back_extreme")

		while True:
			print self.last_fiducial_pose
			
if __name__ == '__main__':
	try:
		rospy.init_node('move_base_relative_to_marker')
	
		mbrtm = move_base_relative_to_marker()
		mbrtm.execute()
		
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)



# /base_link   /fiducial/tag_board (tag_73)
# - Translation: [-0.688, -0.011, 1.002]
# - Rotation: in Quaternion [0.479, 0.520, 0.509, 0.491]
#            in RPY [1.571, 0.024, 1.629]