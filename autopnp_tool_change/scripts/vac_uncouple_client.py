#!/usr/bin/python

import rospy
#import go_to_start_position_client
from go_to_start_position_client import GoToStartPosition
from go_to_slot_client import GoToSlot
from go_back_to_start_client import GoBackToStart
from toolchanger_open_client import ToolchangerOpen

from cob_phidgets.srv import SetDigitalSensor

if __name__ == '__main__':
	try:
	# Initializes a rospy node so that the SimpleActionClient can
	# publish and subscribe over ROS.
		rospy.init_node('Vac_uncouple_client_py')

		result = GoToStartPosition().go_to_start_position_client("vac", "uncouple") 
		if result.result == True:				
			result2 = GoToSlot().go_to_slot_client("vac", "uncouple")
			'''
			if result2.result == True:
				result3 = ToolchangerOpen.toolchanger_open_client()
				
				if result3 == 'yes':
					result4 = GoBackToStart().go_back_to_start_client("vac", "upAndMove")
					
					if result4 == True:
						print "arm_uncoupled OK!"
					else:
						print "arm_uncouple failed !"
						
				else:
					print "arm_uncouple failed !"
					
			else:
				print "arm_uncouple failed !"
			'''	
		else:
			print "arm_uncouple failed !"
			
 	
				
				
				
				
				
				
				
				
				
				
 			'''
		# detach   -> open changer with pin4=1, pin2=0
		service_name = '/cob_phidgets_toolchanger/ifk_toolchanger/set_digital'
		tool_change_successful = ''
		while tool_change_successful != 'yes':
			# wait for confirmation to release tool
			raw_input("Please hold the tool tightly with your hands and then press <Enter> and remove the tool quickly.")
			rospy.wait_for_service(service_name) 
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin4', state=0)
				print 'Resetting tool changer pin4: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			try:
				resp = req(uri='tool_changer_pin2', state=0)
				print 'Resetting tool changer pin2: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			try:
				resp = req(uri='tool_changer_pin4', state=1)
				print 'Opening tool changer response: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			try:
				resp = req(uri='tool_changer_pin4', state=0)
				print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			tool_change_successful = raw_input("If the tool was successfully removed type 'yes' and press <Enter>, otherwise just press enter to repeat. >>")
		'''
		'''		
		# attach	-> close changer with pin4=0, pin2=1
		tool_change_successful = ''
		while tool_change_successful != 'yes':
			# wait for confirmation to attach tool
			raw_input("Please attach the tool manually and then press <Enter>.")
			rospy.wait_for_service(service_name)
			try:
				resp = req(uri='tool_changer_pin2', state=0)
				print 'Resetting tool changer pin2: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin4', state=0)
				print 'Resetting tool changer pin4: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin2', state=1)
				print 'Closing tool changer response: uri=', resp.uri, '  state=', resp.state
				# keep power on closing the changer for safety 
				rospy.sleep(1.0)
				resp = req(uri='tool_changer_pin2', state=0)
				print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			tool_change_successful = raw_input("If the tool was successfully attached type 'yes' and press <Enter>, otherwise just press enter to repeat. >>")
		'''

	except rospy.ROSInterruptException:
 		print "program interrupted before completion"
