##!/usr/bin/python

import rospy
#import go_to_start_position_client
from go_to_start_position_client import GoToStartPosition
from go_to_slot_and_turn_client import GoToSlotAndTurn
from go_back_to_start_client import GoBackToStart


if __name__ == '__main__':
	try:
	# Initializes a rospy node so that the SimpleActionClient can
	# publish and subscribe over ROS.
		rospy.init_node('Vac_uncouple_client_py')

		result = GoToStartPosition().go_to_start_position_client("vac") 		
		result = GoToSlotAndTurn().go_to_slot_and_turn_client("default")
		result = GoBackToStart().go_back_to_start_client("upAndMove")     	


	except rospy.ROSInterruptException:
 		print "program interrupted before completion"
