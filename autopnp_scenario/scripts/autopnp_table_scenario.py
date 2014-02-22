#! /usr/bin/env python
#################################################################
##\file
## \note
#   Copyright (c) 2013 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: autopnp
# \note
#   ROS package name: autopnp_scenario
#
# \author: Richard Bormann(email:richard.bormann@ipa.fraunhofer.de)
#
# \author
# Supervised by: Richard Bormann(email:richard.bormann@ipa.fraunhofer.de) 
# 
# \date Date of creation: February 2014
#
# \brief
# smach representation of Autopnp project demo at Schunk Expert Days
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import smach
import smach_ros
import sys, os
import math

from ScreenFormatting import *
from autopnp_dirt_detection.srv import *
from cob_phidgets.srv import SetDigitalSensor
from cob_srvs.srv import Trigger

from simple_script_server import simple_script_server
sss = simple_script_server()


BASE_LINK_MAP = [0.2, 0.0]	# coordinates of base_link in measured in /map [in m]
CLEANING_REACH_MIN_MAX = [0.5, 0.7]		# distance from base_link in (x,y)-plane where vacuum cleaner can reach dirty locations [in m] 

class DirtDetectionOn(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['dirt_detection_on'])

	def execute(self, userdata ):
		sf = ScreenFormat("DirtDetectionOn")
		rospy.loginfo('Executing state Dirt_Detection_On')

		rospy.wait_for_service('/dirt_detection/activate_dirt_detection') 
		try:
			req = rospy.ServiceProxy('/dirt_detection/activate_dirt_detection',ActivateDirtDetection)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		return 'dirt_detection_on'


class DirtDetectionOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['dirt_detection_off'])

	def execute(self, userdata ):
		sf = ScreenFormat("DirtDetectionOff")
		rospy.loginfo('Executing state Dirt_Detection_Off')

		rospy.wait_for_service('/dirt_detection/deactivate_dirt_detection') 
		try:
			req = rospy.ServiceProxy('/dirt_detection/deactivate_dirt_detection',DeactivateDirtDetection)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		return 'dirt_detection_off'


class ReceiveDirtMap(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['dirt_found', 'no_dirt_found'],
							output_keys=['list_of_dirt_locations', 'last_visited_dirt_location'])

	def execute(self, userdata ):
		sf = ScreenFormat("ReceiveDirtMap")
		rospy.loginfo('Executing state ReceiveDirtMap')
		
		rospy.wait_for_service('/dirt_detection/get_dirt_map')
		try:
			req = rospy.ServiceProxy('/dirt_detection/get_dirt_map', GetDirtMap)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		# create list out of map
		list_of_dirt_locations = []
		map_resolution = resp.dirtMap.info.resolution
		map_offset = resp.dirtMap.info.origin
		for v in range(0, resp.dirtMap.info.height):
			for u in range(0, resp.dirtMap.info.width):
				if resp.dirtMap.data[v*resp.dirtMap.info.width + u] > 25:
					x = (u+0.5)*map_resolution+map_offset.position.x
					y = (v+0.5)*map_resolution+map_offset.position.y
					dist = math.sqrt((x-BASE_LINK_MAP[0])*(x-BASE_LINK_MAP[0]) + (y-BASE_LINK_MAP[1])*(y-BASE_LINK_MAP[1]))
					#if x>0.0 and y>0.0 and x<5.0 and y<2.5:		# todo: tune allowed area
					if dist>CLEANING_REACH_MIN_MAX[0] and dist<CLEANING_REACH_MIN_MAX[1]:
						list_of_dirt_locations.append([x,y])
						print "adding dirt location at (", u, ",", v ,")pix = (", x, ",", y, ")m"
		
		userdata.list_of_dirt_locations = list_of_dirt_locations
		userdata.last_visited_dirt_location = -1
		
		if (len(list_of_dirt_locations)==0):
			rospy.sleep(1);
			return 'no_dirt_found'
		
		return 'dirt_found'


class SelectNextUnprocssedDirtSpot(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['selected_next_dirt_location','no_dirt_spots_left'],
							input_keys=['list_of_dirt_locations', 'last_visited_dirt_location'],
							output_keys=['next_dirt_location', 'last_visited_dirt_location'])
	
	def execute(self, userdata ):
		sf = ScreenFormat("SelectNextUnprocssedDirtSpot")
		rospy.loginfo('Executing state Select_Next_Unprocssed_Dirt_Spot')
		
		if (len(userdata.list_of_dirt_locations)==0) or userdata.last_visited_dirt_location+1==len(userdata.list_of_dirt_locations):
			return 'no_dirt_spots_left'
		else:
			current_dirt_location = userdata.last_visited_dirt_location + 1
			userdata.last_visited_dirt_location = current_dirt_location
			userdata.next_dirt_location = userdata.list_of_dirt_locations[current_dirt_location]
			print "Next dirt location to clean: ", userdata.list_of_dirt_locations[current_dirt_location]
			return 'selected_next_dirt_location'

		print "Error: the script should newer visit this point."
		return 'no_dirt_spots_left'


class Clean(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['cleaning_done'],
							input_keys=['next_dirt_location'])
	
	def execute(self, userdata ):
		sf = ScreenFormat("Clean")
		#rospy.loginfo('Executing state Clean')

		#handle_arm = sss.move("arm",[[0.9865473596897948, -1.0831862403727208, 0.8702560716294125, -0.5028991706696462, 1.4975099515036547, -1.6986067879184412, -8.726646259971648e-05]]) # intermediate position with vacuum cleaner in air
		#handle_arm = sss.move("arm",[[]]) #
		#handle_arm = sss.move("arm",[[]]) #
		
		angle = math.atan2(userdata.next_dirt_location[1]-BASE_LINK_MAP[1], userdata.next_dirt_location[0]-BASE_LINK_MAP[0])
		print "angle=%f" %angle, "  (", (angle/math.pi*180.0), "deg )"
		
		raw_input("cleaning position ok?")
		
		vacuum_init_service_name = '/vacuum_cleaner_controller/init'
		vacuum_on_service_name = '/vacuum_cleaner_controller/vacuum_on'
		vacuum_off_service_name = '/vacuum_cleaner_controller/vacuum_off'
		
		# (re-)init vacuum cleaner
 		rospy.wait_for_service(vacuum_init_service_name) 
		try:
			req = rospy.ServiceProxy(vacuum_init_service_name, Trigger)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e

		carrying_position = [1.978714679571011, -0.9163502171745829, 0.08915141819187035, -1.796921184683282, 2.4326209849093216, -1.2165643018101275, 1.2519770323330925] # carrying position
		intermediate1_position = [1.4535276543533975, -0.3381749958664213, -0.07175048554948689, -1.937908881659384, 2.2285221821811047, -1.234576099690709, 1.000527447] # intermediate 1
		intermediate2_position = [0.7885223027585182, -0.14316935854109486, -0.07175048554948689, -1.937908881659384, 2.0185241665811468, -0.9095783396768448, 1.000527447] # intermediate 2
		above_cleaning_20cm_position = [-0.09950122065619672, -0.19219565722961557, 0.08124507668033604, -2.1109059171170617, 1.7055153453006288, -0.2646093678948603, 1.000527447] # ca. 20cm above cleaning position
		above_cleaning_5cm_position = [-0.09944886077863689, -0.7551690607529065, 0.08124507668033604, -1.562907438575882, 1.7055153453006288, -0.2646093678948603, 1.000527447] # just 5cm above cleaning position
		cleaning_position = [-0.09923942126839758, -0.909491073214245, 0.08154178265317508, -1.4209598105111834, 1.695622274897531, -0.24858724536155236, 1.066797598696494] #[-0.09944886077863689, -0.9291958404692611, 0.08124507668033604, -1.4179229376127134, 1.7055153453006288, -0.2646093678948603, 1.000527447] # cleaning position

		# move arm from storage position to cleaning position
		if JOURNALIST_MODE == False:
			handle_arm = sss.move("arm",[carrying_position, intermediate1_position, intermediate2_position, above_cleaning_20cm_position, above_cleaning_5cm_position, cleaning_position])
		else:
			handle_arm = sss.move("arm",[carrying_position])
			raw_input("enter")
			handle_arm = sss.move("arm",[intermediate1_position])
			raw_input("enter")
			handle_arm = sss.move("arm",[intermediate2_position])
			raw_input("enter")
			handle_arm = sss.move("arm",[above_cleaning_20cm_position])
			raw_input("enter")
			handle_arm = sss.move("arm",[above_cleaning_5cm_position])
			raw_input("enter")
			handle_arm = sss.move("arm",[cleaning_position])
			raw_input("enter")
		
		# turn vacuum cleaner on
		rospy.wait_for_service(vacuum_on_service_name) 
		try:
			req = rospy.ServiceProxy(vacuum_on_service_name, Trigger)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		# move base (if necessary)
		for i in range(0,1):
			for j in range(0,2):
				handle_base = sss.move_base_rel("base", (0.0, -0.1, 0.0), blocking=True)
			for j in range(0,5):
				handle_base = sss.move_base_rel("base", (0.0, 0.1, 0.0), blocking=True)
			for j in range(0,2):
				handle_base = sss.move_base_rel("base", (0.0, -0.1, 0.0), blocking=True)
			
		
		# turn vacuum cleaner off
		rospy.wait_for_service(vacuum_off_service_name) 
		try:
			req = rospy.ServiceProxy(vacuum_off_service_name, Trigger)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		# move arm back to storage position
		handle_arm = sss.move("arm",[above_cleaning_5cm_position, above_cleaning_20cm_position, intermediate2_position, intermediate1_position, carrying_position])
		
		rospy.sleep(2)
		
		#raw_input("Quit program")
		#handle_arm = sss.move("arm",[[]]) #
		#handle_arm = sss.move("arm",[[]]) #
		#handle_arm = sss.move("arm",[[]]) #
		#handle_arm = sss.move("arm",[[1.4605438779464148, -0.548173011466379, 0.08925613794699001, -1.751909143274348, 2.1865310336059762, -1.3275846955294868, -1.9370711236184264]]) #intermediate 1.5?
		
		return 'cleaning_done'


class ChangeToolManual(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['CTM_done'])
		# command line usage:
		# rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital '{uri: "tool_changer_pin2", state: 0}'
		# rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital '{uri: "tool_changer_pin4", state: 0}'
		
	def execute(self, userdata):
		sf = ScreenFormat("ChangeToolManual")
		
		# move arm with tool facing up (so it cannot fall down on opening)
		#handle_arm = sss.move("arm", "pregrasp")
		#handle_arm = sss.move("arm",[[1.064633390424021, -1.1901051103498934, 0.6336766915215812, -1.7237046225621198, -1.554041165975751, -1.7535846593562627, -0.00010471975511965978]])
		
		service_name = '/cob_phidgets_toolchanger/ifk_toolchanger/set_digital'
		tool_change_successful = ''
		while tool_change_successful!='yes':
			# wait for confirmation to release tool
			raw_input("Please hold the tool tightly with your hands and then press <Enter> and remove the tool quickly.")
			rospy.wait_for_service(service_name) 
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin4', state=0)
				print 'Resetting tool changer pin4: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			try:
				resp = req(uri='tool_changer_pin2', state=0)
				print 'Resetting tool changer pin2: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			try:
				resp = req(uri='tool_changer_pin4', state=1)
				print 'Opening tool changer response: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			try:
				resp = req(uri='tool_changer_pin4', state=0)
				print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			tool_change_successful = raw_input("If the tool was successfully removed type 'yes' and press <Enter>, otherwise just press enter to repeat. >>")
		
		# move arm with end facing down
		#handle_arm = sss.move("arm",[[1.3676400018627566, -0.882106857250454, 0.8536754437354664, -1.6116893911691237, 2.041947958370766, -1.7976018630915598, -0.00010471975511965978]])

		tool_change_successful = ''
		while tool_change_successful!='yes':
			# wait for confirmation to attach tool
			raw_input("Please attach the tool manually and then press <Enter>.")
			rospy.wait_for_service(service_name) 
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin2', state=1)
				print 'Closing tool changer response: uri=', resp.uri, '  state=', resp.state
				# keep power on closing the changer for safety 
				#rospy.sleep(1.0)
				#resp = req(uri='tool_changer_pin4', state=0)
				#print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			tool_change_successful = raw_input("If the tool was successfully attached type 'yes' and press <Enter>, otherwise just press enter to repeat. >>")
		
		carrying_position = [1.978714679571011, -0.9163502171745829, 0.08915141819187035, -1.796921184683282, 2.4326209849093216, -1.2165643018101275, 1.2519770323330925] # carrying position
		#handle_arm = sss.move("arm",[carrying_position])
		
		print 'Manual tool change successfully completed.'
		
		return 'CTM_done'


def main_cleaning():
	rospy.init_node('cleaning')
	
	# clean
	sm_scenario = smach.StateMachine(outcomes=['finished'])
	with sm_scenario:
		smach.StateMachine.add('DIRT_DETECTION_ON', DirtDetectionOn(),
					transitions={'dirt_detection_on':'GET_DIRT_MAP'})
		
		smach.StateMachine.add('GET_DIRT_MAP', ReceiveDirtMap(),
					transitions={'dirt_found':'DIRT_DETECTION_OFF',
								'no_dirt_found':'GET_DIRT_MAP'})
		
		smach.StateMachine.add('DIRT_DETECTION_OFF', DirtDetectionOff(),
					transitions={'dirt_detection_off':'SELECT_NEXT_UNPROCESSED_DIRT_SPOT'})
		
		smach.StateMachine.add('SELECT_NEXT_UNPROCESSED_DIRT_SPOT', SelectNextUnprocssedDirtSpot(),
								transitions={'selected_next_dirt_location':'CLEAN',
											'no_dirt_spots_left':'DIRT_DETECTION_ON'})
		
		smach.StateMachine.add('CLEAN', Clean(),
								transitions={'cleaning_done':'SELECT_NEXT_UNPROCESSED_DIRT_SPOT'})
	
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_scenario, '/START')
	sis.start()
	
	# Execute SMACH plan
	outcome = sm_scenario.execute()
	
	sis.stop()


def main_toolchange():
	rospy.init_node('toolchange')

	# manual tool change
	sm_scenario = smach.StateMachine(outcomes=['tool_change_done'])
	with sm_scenario:
		smach.StateMachine.add('CHANGE_TOOL_MANUAL_IMPLEMENTATION', ChangeToolManual(),
								transitions={'CTM_done':'tool_change_done'})
	
	
	'''
	# trash bin clearing stand alone
	sm_scenario = smach.StateMachine(outcomes=['CWB_done', 'failed'],input_keys=['detection_pose'])
	with sm_scenario:
		smach.StateMachine.add('INITIALIZE_AUTOPNP_SCENARIO', InitAutoPnPScenario(),
					transitions={'initialized':'GRASP_TRASH_BIN',
								'failed':'failed'})
		
# 		smach.StateMachine.add('MOVE_TO_TRASH_BIN_LOCATION', MoveToTrashBinLocation(),
# 							transitions={'MTTBL_success':'APPROACH_PERIMETER'},
# 								remapping = {'trash_bin_pose_':'detection_pose'})
		
# 		smach.StateMachine.add('APPROACH_PERIMETER', ApproachPerimeter(),
# 							transitions={'reached':'GRASP_TRASH_BIN', 
# 										 'not_reached':'failed',
# 										 'failed':'failed'},
# 								remapping = {'trash_bin_pose_':'detection_pose'})
		
		smach.StateMachine.add('GRASP_TRASH_BIN', GraspTrashBin(),
							transitions={'GTB_success':'MOVE_TO_TOOL_WAGON_FRONTAL',
										 'failed':'failed'})
		
		smach.StateMachine.add('MOVE_TO_TOOL_WAGON_FRONTAL', MoveToToolWaggonFrontFrontalFar(),
							transitions={'arrived':'MOVE_TO_TOOL_WAGON_TURN180'})
		
		smach.StateMachine.add('MOVE_TO_TOOL_WAGON_TURN180', Turn180(),
							transitions={'arrived':'CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART1'})
		
		smach.StateMachine.add('CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART1', ClearTrashBinIntoToolWagonPart1(),
							transitions={'finished':'MOVE_TO_TOOL_WAGON_FRONTAL_TRASH_BIN_CLEARING'})
		
		smach.StateMachine.add('MOVE_TO_TOOL_WAGON_FRONTAL_TRASH_BIN_CLEARING', MoveToToolWaggonFrontTrashClearing(),
							transitions={'arrived':'CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART2'})
		
		smach.StateMachine.add('CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART2', ClearTrashBinIntoToolWagonPart2(),
							transitions={'finished':'RELEASE_TRASH_BIN'})
		
# 		smach.StateMachine.add('MOVE_TO_TRASH_BIN_PICKING_LOCATION', MoveToTrashBinPickingLocation(),
# 							transitions={'MTTBPL_done':'APPROACH_PERIMETER_2'},
# 							remapping = {'trash_bin_pose_':'detection_pose'})
		
# 		smach.StateMachine.add('APPROACH_PERIMETER_2', ApproachPerimeter(),
# 							transitions={'reached':'RELEASE_TRASH_BIN', 
# 										 'not_reached':'failed',
# 										 'failed':'failed'},
# 							remapping = {'trash_bin_pose_':'detection_pose'})
		
		smach.StateMachine.add('RELEASE_TRASH_BIN', ReleaseTrashBin(),
							transitions={'RTB_finished':'CWB_done'})
	'''
	
	
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_scenario, '/START')
	sis.start()
	
	# Execute SMACH plan
	outcome = sm_scenario.execute()
	
	sis.stop()


def start_prompt():
	print "---------------------------------------------------------"
	print "plese start script with one of the following arguments:"
	print "clean - starts the cleaning scenario"
	print "tool - changes the tool"
	print "---------------------------------------------------------"
	sys.exit(1)

if __name__ == '__main__':
	try:
		if len(sys.argv)<2:
			start_prompt()
		else:
			flag=sys.argv[1]
		if len(flag)==0:
			start_promt()

		if flag=="clean":
			print "Starting cleaning scenario..."
			main_cleaning()
		elif flag=="tool":
			print "Starting tool change..."
			main_toolchange()
		else:
			start_prompt()
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)