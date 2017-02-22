

#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_tool_change')
import rospy
import actionlib
import autopnp_tool_change.msg 

class ToolChangerClose:
	
	def toolchanger_close_client():
		
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
	
   		return tool_change_successful # State after waiting for GoToStartPositionAction


