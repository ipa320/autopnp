#!/usr/bin/python

import rospy
#import go_to_start_position_client
from go_to_start_position_client import GoToStartPosition
from go_to_slot_client import GoToSlot
from go_back_to_start_client import GoBackToStart


if __name__ == '__main__':
	try:
	# Initializes a rospy node so that the SimpleActionClient can
	# publish and subscribe over ROS.
		rospy.init_node('Vac_couple_client_py')
		
		result = GoToStartPosition().go_to_start_position_client("vac", "couple")
		
		if result.result == True:
			result2 = GoToSlot().go_to_slot_client("vac", "couple")
					
			if result2.result == True:
				#result3 = ToolchnagerClose.toolchnager_close_client()
				
				if result3 == 'yes':
					#result4 = GoBackToStart().go_back_to_start_client("vac", "liftAndBack")
					
					if result4.result == True:
						print "vac coupled OK !" 
					else:
						print "vac_couple failed !"
				else:
					print "vac_couple failed !"	
			else:
				print "vac_couple failed !"
				
		else:
			print "vac_couple failed !"


	except rospy.ROSInterruptException:
		print "program interrupted before completion"
