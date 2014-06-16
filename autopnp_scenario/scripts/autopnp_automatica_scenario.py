#! /usr/bin/env python
#################################################################
##\file
## \note
#   Copyright (c) 2014 \n
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
# Supervised by: 
# 
# \date Date of creation: May 2014
#
# \brief
# smach representation of Autopnp project
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
import sys, os
os.system("rosservice call /say 'Lets clean this room.' &")

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import smach
import smach_ros


from exploration_detection_cleaning import *

def h():
	sys.stderr.write('shutdown requested\n')

#wagon: 1.513, -0.890 0.7
#trash: 1.352. 0.287. 0.000). Orientation(0.000. 0.000. 0.970. 0.244) = Angle: 2.649
#dirt1 -0.208. -0.690. 0.000). Orientation(0.000. 0.000. 0.492. 0.871) = Angle: 1,028
#dirt2 1.5

def main(confirm):
	rospy.init_node('exploration_detection_cleaning')
	rospy.on_shutdown(h)
	

	#sss.move("arm",[ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_20cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate2_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate1_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])
	#exit()

	# todo: parameters
	tool_wagon_map_pose = Pose2D(x=1.513, y=-0.890, theta=-0.7+math.pi)  # the map coordinates of the tool wagon center
	trash_bin_inspection_map_poses = [ [[1.352, 0.1, 2.649], "linear"], [[1.352, 0.1, 2.3], "linear"] ]   # list of inspection positions (x,y,theta) of robot with movement mode
	#,									   [[1.2, -0.0, 2.4], "linear"]
	dirt_inspection_map_poses = [ [[-0.208, -0.690, 1.], "linear"],
								  [[-0.208, -0.690, 1.5], "linear"] ]	  # list of inspection positions (x,y,theta) of robot with movement mode
	# deactivated at the moment: valid_rectangle_for_dirt_detections = [-1.0, -4.8, 3.0, 0.0]   # dirt detections outside of this rectangle ([min_x, min_y, max_x, max_y]) will not be attended to during the script
	
	# full Automatica scenario (i.e. let the operator attach/change the tool, do the job according to the attached tool)
	# just fill history of global transform listener
	listener = get_transform_listener()
	rospy.sleep(5.0)
	
	sm_scenario = smach.StateMachine(outcomes=['finished', 'failed'])
	sm_scenario.userdata.sm_trash_bin_counter = 0

	with sm_scenario:

		smach.StateMachine.add('INITIALIZE_AUTOPNP_SCENARIO', InitAutoPnPScenario(confirm_mode=confirm, tool_wagon_pose=tool_wagon_map_pose),
							transitions={'initialized':'DETERMINE_ATTACHED_TOOL',#CHANGE_TOOL_MANUAL', #'ANALYZE_MAP',
										'failed':'failed'})
		
		#smach.StateMachine.add('ANALYZE_MAP', AnalyzeMap(),
		#					transitions={'list_of_rooms': 'CHANGE_TOOL_MANUAL'},   #'GO_TO_NEXT_UNPROCESSED_ROOM'},
		#					remapping={'analyze_map_data_img_':'sm_img'})
		
		
		smach.StateMachine.add('DETERMINE_ATTACHED_TOOL', DetermineAttachedTool(),
							transitions={'sdh':'TRASH_BIN_DETECTION_ON',
										 'vacuum':'DIRT_DETECTION_ON',
										 'none':'CHANGE_TOOL_MANUAL',
										 'failed':'failed'})

		sm_sub_change_tool_manual = smach.StateMachine(outcomes=['sdh_attached', 'vacuum_attached', 'failed'],
														input_keys=['tool_wagon_pose'])
		with sm_sub_change_tool_manual:
			smach.StateMachine.add('GO_TO_TOOL_WAGON_LOCATION', MoveToToolWaggonFrontFar(),
								transitions={'arrived':'CHANGE_TOOL_MANUAL_IMPLEMENTATION'})
			
			smach.StateMachine.add('CHANGE_TOOL_MANUAL_IMPLEMENTATION', ChangeToolManualPnP(release_confirmation_device='joystick'),
								transitions={'CTM_done_sdh':'sdh_attached',
											 'CTM_done_vacuum':'vacuum_attached',
											 'failed':'failed'})
		
		smach.StateMachine.add('CHANGE_TOOL_MANUAL', sm_sub_change_tool_manual,
							transitions={'sdh_attached':'TRASH_BIN_DETECTION_ON',
										 'vacuum_attached':'DIRT_DETECTION_ON',
										 'failed':'failed'})


		### trash bin clearing sub-script
		#smach.StateMachine.add('TRASH_BIN_DETECTION_ON', MoveToToolWaggonFrontFrontalFar(),
		#					transitions={'arrived':'finished'})
		smach.StateMachine.add('TRASH_BIN_DETECTION_ON', TrashBinDetectionOn(),
							transitions={'trash_bin_detection_on':'INSPECT_ROOM_FOR_TRASH_BINS'})
		
		smach.StateMachine.add('INSPECT_ROOM_FOR_TRASH_BINS', InspectRoomShowcase(inspection_poses=trash_bin_inspection_map_poses,
																				  inspection_time=2.0,
																				  search_target='trash bins to clear.'),
							transitions={'finished':'TRASH_BIN_DETECTION_OFF'})
		
		smach.StateMachine.add('TRASH_BIN_DETECTION_OFF', TrashBinDetectionOff(),
							transitions={'trash_can_found':'GO_TO_NEXT_UNPROCESSED_WASTE_BIN',
										'trash_can_not_found':'TRASH_BIN_DETECTION_ON'},
							remapping={'detected_waste_bin_poses_':'trash_detection_poses'})

		
		smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_WASTE_BIN', GoToNextUnprocessedWasteBin(),
							transitions={'go_to_trash_location':'CLEAR_TRASH_BIN',
										'all_trash_bins_cleared':'TRASH_BIN_DETECTION_ON'},
							remapping={'go_to_next_unprocessed_waste_bin_in_':'trash_detection_poses',
										'go_to_next_unprocessed_waste_bin_out_':'detection_pose',
										'number_of_unprocessed_trash_bin_in_':'sm_trash_bin_counter',
										'number_of_unprocessed_trash_bin_out_':'sm_trash_bin_counter'})


		sm_sub_clear_waste_bin = smach.StateMachine(outcomes=['clear_trash_bin_done', 'failed'],input_keys=['detection_pose', 'tool_wagon_pose'])

		with sm_sub_clear_waste_bin:
			smach.StateMachine.add('MOVE_TO_TRASH_BIN_LOCATION', MoveToTrashBinLocation(),
								transitions={'MTTBL_success':'APPROACH_PERIMETER'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('APPROACH_PERIMETER', ApproachPerimeter(tf_listener=get_transform_listener()),
								transitions={'reached':'MOVE_TO_TRASH_BIN_LOCATION_LINEAR', 
											 'not_reached':'MOVE_TO_TRASH_BIN_LOCATION_LINEAR',
											 'failed':'failed'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('MOVE_TO_TRASH_BIN_LOCATION_LINEAR', MoveToTrashBinLocationLinear(),
								transitions={'MTTBL_success':'APPROACH_PERIMETER_LINEAR'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('APPROACH_PERIMETER_LINEAR', ApproachPerimeter(mode='linear', tf_listener=get_transform_listener()),
								transitions={'reached':'CHECK_POSITION_TO_TRASH_BIN', 
											 'not_reached':'CHECK_POSITION_TO_TRASH_BIN',
											 'failed':'failed'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('CHECK_POSITION_TO_TRASH_BIN', CheckPositionToTrashBinLocation(),
								transitions={'success':'GRASP_TRASH_BIN',
											 'failed':'MOVE_TO_TRASH_BIN_LOCATION_LINEAR'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
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
								transitions={'finished':'MOVE_TO_TRASH_BIN_PICKING_LOCATION'})
			
			smach.StateMachine.add('MOVE_TO_TRASH_BIN_PICKING_LOCATION', MoveToTrashBinPickingLocation(),
								transitions={'MTTBPL_done':'APPROACH_PERIMETER_2'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('APPROACH_PERIMETER_2', ApproachPerimeter(tf_listener=get_transform_listener()),
								transitions={'reached':'RELEASE_TRASH_BIN', 
											'not_reached':'RELEASE_TRASH_BIN',
											'failed':'failed'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('RELEASE_TRASH_BIN', ReleaseTrashBin(),
								transitions={'RTB_finished':'clear_trash_bin_done'})

		smach.StateMachine.add('CLEAR_TRASH_BIN', sm_sub_clear_waste_bin,
							transitions={'clear_trash_bin_done':'GO_TO_NEXT_UNPROCESSED_WASTE_BIN'})
		
		
		
		
		### dirt cleaning sub-script
		smach.StateMachine.add('DIRT_DETECTION_ON', DirtDetectionOn(),
							transitions={'dirt_detection_on':'INSPECT_ROOM_FOR_DIRT'})

		smach.StateMachine.add('INSPECT_ROOM_FOR_DIRT', InspectRoomShowcase(inspection_poses=dirt_inspection_map_poses,
																		  inspection_time=3.0,
																		  search_target='dirt spots on the ground.'),
							transitions={'finished':'DIRT_DETECTION_OFF'})

		smach.StateMachine.add('DIRT_DETECTION_OFF', DirtDetectionOff(),
							transitions={'dirt_detection_off':'GET_DIRT_MAP'})

		smach.StateMachine.add('GET_DIRT_MAP', ReceiveDirtMap(valid_rectangle_for_dirt_detections=0),  #valid_rectangle_for_dirt_detections),
							transitions={'list_of_dirt_location':'GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION',
										 'failed': 'DIRT_DETECTION_ON'})
		
		
		
		sm_sub_go_to_next_unprocessed_dirt_location = smach.StateMachine(outcomes=['no_dirt_spots_left','arrived_dirt_location', 'failed'],
																		input_keys=['list_of_dirt_locations', 'last_visited_dirt_location'],
																		output_keys=['next_dirt_location'])

# 		with sm_sub_go_to_next_unprocessed_dirt_location:
# 			smach.StateMachine.add('SELECT_NEXT_UNPROCESSED_DIRT_SPOT', SelectNextUnprocssedDirtSpot(),
# 								transitions={'selected_next_dirt_location':'MOVE_TO_DIRT_LOCATION_PERIMETER_CLEANING',
# 											'no_dirt_spots_left':'no_dirt_spots_left'},
# 								remapping = {'last_visited_dirt_location_in':'last_visited_dirt_location',
# 											 'last_visited_dirt_location_out':'last_visited_dirt_location'})
# 			
# 			smach.StateMachine.add('MOVE_TO_DIRT_LOCATION_PERIMETER_CLEANING', MoveLocationPerimeterCleaning(),
# 								transitions={'movement_prepared':'APPROACH_PERIMETER_CLEANING'})
# 			
# 			smach.StateMachine.add('APPROACH_PERIMETER_CLEANING', ApproachPerimeter(tf_listener=get_transform_listener()),
# 								transitions={'reached':'arrived_dirt_location', 
# 											 'not_reached':'SELECT_NEXT_UNPROCESSED_DIRT_SPOT',
# 											 'failed':'failed'})
# 
# 		smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION', sm_sub_go_to_next_unprocessed_dirt_location,
# 							transitions={'arrived_dirt_location':'CLEAN',
# 										'no_dirt_spots_left':'DIRT_DETECTION_ON'})

		smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION', SelectNextUnprocssedDirtSpot(),
							transitions={'selected_next_dirt_location':'CLEAN_CELL_GROUP',
										'no_dirt_spots_left':'DIRT_DETECTION_ON'},
							remapping = {'last_visited_dirt_location_in':'last_visited_dirt_location',
										 'last_visited_dirt_location_out':'last_visited_dirt_location'})
		
		
		
#		smach.StateMachine.add('CLEAN', Clean(), transitions={'cleaning_done':'GO_TO_INSPECT_LOCATION'})
		
		smach.StateMachine.add('CLEAN_CELL_GROUP', CleanCellGroup(), transitions={'cleaning_done':'GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION'})#'GO_TO_INSPECT_LOCATION'})
		
		
		
		sm_sub_go_to_inspect_location = smach.StateMachine(outcomes=['arrived_cleaning_inspection_location', 'not_reached', 'failed'],
														input_keys=['next_dirt_location'])

		with sm_sub_go_to_inspect_location:
			smach.StateMachine.add('MOVE_TO_DIRT_LOCATION_PERIMETER_VALIDATION', MoveLocationPerimeterValidation(),
								transitions={'movement_prepared':'APPROACH_PERIMETER_VALIDATION'})
			
			smach.StateMachine.add('APPROACH_PERIMETER_VALIDATION', ApproachPerimeter(tf_listener=get_transform_listener()),
								transitions={'reached':'arrived_cleaning_inspection_location', 
											 'not_reached':'not_reached',
											 'failed':'failed'})

		smach.StateMachine.add('GO_TO_INSPECT_LOCATION', sm_sub_go_to_inspect_location,
							transitions={'arrived_cleaning_inspection_location':'VERIFY_CLEANING_SUCCESS',
										'not_reached':'GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION',
										'failed':'failed'})
		
		
		
		smach.StateMachine.add('VERIFY_CLEANING_SUCCESS', VerifyCleaningProcess(),
							transitions={'verify_cleaning_done':'GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION'})
		
	
	# Create and start the introspection server
	#sis = smach_ros.IntrospectionServer('server_name', sm_scenario, '/START')
	#sis.start()
	
	# Execute SMACH plan
	outcome = sm_scenario.execute()
	
	rospy.spin()
	
	#sis.stop()


if __name__ == '__main__':
	try:
		flag = ""
		if len(sys.argv)<2:
			CONFIRM_MODE = True
		else:
			flag=sys.argv[1]

		if flag=="auto":
			CONFIRM_MODE = False  # in auto mode the connection to the control pc may interrupted, the scenario will stil work
			# necessitates that all nodes are started with & and the end, e.g. bringup &
		elif flag=="special":
			CONFIRM_MODE = False
		else:
			CONFIRM_MODE = True

		CONFIRM_MODE = False
		print "CONFIRM_MODE ", CONFIRM_MODE
		main(CONFIRM_MODE)
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
