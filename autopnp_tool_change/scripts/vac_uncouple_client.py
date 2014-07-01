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
		
		if result.result == True:
			result2 = GoToSlotAndTurn().go_to_slot_and_turn_client("default")
				
			if result2.result == True:
				result3 = ToolchnagerOpen.toolchnager_open_client()
				
				if result3 == 'yes':
					result4 = GoBackToStart().go_back_to_start_client("upAndMove")
					
					if result4.result == True:
						print "vac uncoupled OK!"
					else:
						print "vac_uncouple failed !"
				else:
					print "vac_uncouple failed !"
			else:
				print "vac_uncouple failed !"
		else:
			print "vac_uncouple failed !"

	except rospy.ROSInterruptException:
		print "program interrupted before completion"
