

#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_tool_change')
import rospy
import actionlib
import autopnp_tool_change.msg 

class ToolchangerOpen:
	
	def toolchanger_open_client():
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
		
   		return tool_change_successful # State after waiting for GoToStartPositionAction


