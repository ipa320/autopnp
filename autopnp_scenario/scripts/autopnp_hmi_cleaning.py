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
# \author: Mohammad Muinul Islam(email-> mohammad.islam@ipa.fraunhofer.de)
#
# \author
# Supervised by: Richard Bormann(email:richard.bormann@ipa.fraunhofer.de) 
# 
# \date Date of creation: August 2013
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

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import smach
import smach_ros
import sys, os

from exploration_detection_cleaning import *


def main(confirm):
	rospy.init_node('exploration_detection_cleaning')
	
	'''
	# approach tool waggon
	sm_scenario = smach.StateMachine(outcomes=['finished'])
	with sm_scenario:
		smach.StateMachine.add('MOVE_TO_TOOL_WAGON_FRONTAL_TRASH_BIN_CLEARING', MoveToToolWaggonFrontTrashClearing(),
								transitions={'arrived':'finished'})
	'''
	'''
	# clean
	sm_scenario = smach.StateMachine(outcomes=['cleaning_done'])
	with sm_scenario:
		smach.StateMachine.add('CLEAN', Clean(),
								transitions={'cleaning_done':'cleaning_done'})
	'''
	
	'''
	# manual tool change
	sm_scenario = smach.StateMachine(outcomes=['tool_change_done'])
	with sm_scenario:
		smach.StateMachine.add('CHANGE_TOOL_MANUAL_IMPLEMENTATION', ChangeToolManual(),
								transitions={'CTM_done':'tool_change_done'})
	'''
	
	
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
		
		smach.StateMachine.add('GRASP_TRASH_BI	"front_trash_clearing":Pose2D(x=-1.05, y=0.0, theta=0.0)}
N', GraspTrashBin(),
							transitions={'GTB_success':'MOVE_TO_TOOL_WAGON_FRONTAL',
										 'failed':'failed'})
		
		smach.StateMachine.add('MOVE_TO_TOOL_WAGON_FRONTAL', MoveToToolWaggonFrontFrontalFar(),
							transitions={'arrived':'MOVE_TO_TOOL_WAGON_TURN180'})
		
		smach.StateMachine.add('MOVE_TO_TOOL_WAGON_TURN180', Turn180(),
							transitions={'arrived':'CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART1'})
		
		smach.StateMachine.add('CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART1', ClearTrashBinIntoToolWagonPart1(),
							transitions={'finished':'MOVE_TO_TOOL_WAGON_FRONTAL_TRASH_BIN_CLEARING'})
			"front_trash_clearing":Pose2D(x=-1.05, y=0.0, theta=0.0)}

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
	
	
	
	# full scenario
	sm_scenario = smach.StateMachine(outcomes=['finished', 'failed'])
	sm_scenario.userdata.sm_trash_bin_counter = 0

	with sm_scenario:

		smach.StateMachine.add('INITIALIZE_AUTOPNP_SCENARIO', InitAutoPnPScenario(confirm_mode=confirm),
							transitions={'initialized':'ANALYZE_MAP',
										'failed':'failed'})
		
		smach.StateMachine.add('ANALYZE_MAP', AnalyzeMap(),
							transitions={'list_of_rooms':'GO_TO_NEXT_UNPROCESSED_ROOM'},
							remapping={'analyze_map_data_img_':'sm_img'})

		sm_sub_go_to_next_unproccessed_room = smach.StateMachine(outcomes=['arrived','no_more_rooms_left'],
																input_keys=['sm_img',
																			'analyze_map_data_map_resolution_',
																			'analyze_map_data_map_origin_x_',
																			'analyze_map_data_map_origin_y_',
																			'analyze_map_data_room_center_x_', 
																			'analyze_map_data_room_center_y_',
																			'analyze_map_data_room_min_x_',
																			'analyze_map_data_room_max_x_',
																			'analyze_map_data_room_min_y_',
																			'analyze_map_data_room_max_y_',
																			'tool_wagon_pose'],
																output_keys=['sm_RoomNo'])
		
		sm_sub_go_to_next_unproccessed_room.userdata.sm_counter = 1
		sm_sub_go_to_next_unproccessed_room.userdata.sm_location_counter = 0
		
		with sm_sub_go_to_next_unproccessed_room:
			smach.StateMachine.add('FIND_NEXT_ROOM', NextUnprocessedRoom(),
								transitions={'location':'VERIFY_TOOL_CAR_LOCATION',
											'no_rooms':'no_more_rooms_left'},
								remapping={'find_next_unprocessed_room_data_img_':'sm_img',
											'find_next_unprocessed_room_number_in':'sm_RoomNo',
											'find_next_unprocessed_room_number_out_':'sm_RoomNo',
											'find_next_unprocessed_room_loop_counter_in_':'sm_counter',
											'find_next_unprocessed_room_loop_counter_out_':'sm_counter'})
			
			smach.StateMachine.add('VERIFY_TOOL_CAR_LOCATION', VerifyToolCarLocation(),
								transitions={'no_need_to_move_it':'arrived', #'GO_TO_LOCATION',
											'tool_wagon_needs_to_be_moved':'MOVE_TOOL_WAGON'})

			sm_sub_move_tool_wagon = smach.StateMachine(outcomes=['finished'],
														input_keys=['tool_wagon_pose',
																	'tool_wagon_goal_pose'])
			with sm_sub_move_tool_wagon:
				smach.StateMachine.add('MOVE_BASE_TO_LAST_TOOL_WAGGON_LOCATION', MoveToToolWaggonFront(),
									transitions={'arrived':'GRASP_HANDLE'})
				
				smach.StateMachine.add('GRASP_HANDLE', GraspHandle(),
									transitions={'grasped':'GO_TO_NEXT_TOOL_WAGGON_LOCATION'})
				
				smach.StateMachine.add('GO_TO_NEXT_TOOL_WAGGON_LOCATION', GoToNextToolWaggonLocation(),
									transitions={'arrived':'RELEASE_GRASP'})
				
				smach.StateMachine.add('RELEASE_GRASP', ReleaseGrasp(),
									transitions={'released':'finished'})


			smach.StateMachine.add('MOVE_TOOL_WAGON', sm_sub_move_tool_wagon,
								transitions={'finished':'GO_TO_LOCATION'})
			
			smach.StateMachine.add('GO_TO_LOCATION', GoToRoomLocation(),
								transitions={'successful':'arrived',
											'unsuccessful':'RANDOM_LOCATION_FINDER'},
								remapping={'go_to_room_location_data_img_':'sm_img',
										'go_to_room_location_loop_counter_in_':'sm_location_counter',
										'go_to_room_location_loop_counter_out_':'sm_location_counter'}) 

			smach.StateMachine.add('RANDOM_LOCATION_FINDER', FindRandomLocation(),
								transitions={'re_locate':'GO_TO_LOCATION',
											'unsuccessful_five_times':'FIND_NEXT_ROOM'},
								remapping={'random_location_finder_data_img_in_':'sm_img',
										'random_location_finder_data_img_out_':'sm_img',
										'random_location_finder_counter_in_':'sm_location_counter',
										'random_location_finder_counter_out_':'sm_location_counter',
										'random_location_finder_room_number_':'sm_RoomNo'}) 

		smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_ROOM', sm_sub_go_to_next_unproccessed_room,
							transitions={'arrived':'DIRT_DETECTION_ON',
										'no_more_rooms_left': 'DIRT_DETECTION_ON'})#'CHANGE_TOOL_MANUAL'}) #'CHANGE_TOOL_HAND'})
		
		smach.StateMachine.add('DIRT_DETECTION_ON', DirtDetectionOn(),
							transitions={'dirt_detection_on':'INSPECT_ROOM'})
		
		#smach.StateMachine.add('TRASH_BIN_DETECTION_ON', TrashBinDetectionOn(),
		#					transitions={'trash_bin_detection_on':'INSPECT_ROOM'})
		
		smach.StateMachine.add('INSPECT_ROOM', InspectRoom(),
							transitions={'finished':'DIRT_DETECTION_OFF'},
							remapping={'inspect_room_data_img_in_':'sm_img',
										'inspect_room_img_out_':'sm_img',
										'inspect_room_room_number_':'sm_RoomNo'})

		smach.StateMachine.add('DIRT_DETECTION_OFF', DirtDetectionOff(),
							transitions={'dirt_detection_off':'GET_DIRT_MAP'})
		
		#smach.StateMachine.add('TRASH_BIN_DETECTION_OFF', TrashBinDetectionOff(),
		#					transitions={'trash_can_found':'GO_TO_NEXT_UNPROCESSED_WASTE_BIN',
		#								'trash_can_not_found':'GO_TO_NEXT_UNPROCESSED_ROOM'},
		#					remapping={'detected_waste_bin_poses_':'trash_detection_poses'})

		smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_WASTE_BIN', GoToNextUnprocessedWasteBin(),
							transitions={'go_to_trash_location':'CLEAR_TRASH_BIN',
										'all_trash_bins_cleared':'GO_TO_NEXT_UNPROCESSED_ROOM'},
							remapping={'go_to_next_unprocessed_waste_bin_in_':'trash_detection_poses',
										'go_to_next_unprocessed_waste_bin_out_':'detection_pose',
										'number_of_unprocessed_trash_bin_in_':'sm_trash_bin_counter',
										'number_of_unprocessed_trash_bin_out_':'sm_trash_bin_counter'})


		sm_sub_clear_waste_bin = smach.StateMachine(outcomes=['clear_trash_bin_done', 'failed'],input_keys=['detection_pose', 'tool_wagon_pose'])

		with sm_sub_clear_waste_bin:
			smach.StateMachine.add('MOVE_TO_TRASH_BIN_LOCATION', MoveToTrashBinLocation(),
								transitions={'MTTBL_success':'APPROACH_PERIMETER'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('APPROACH_PERIMETER', ApproachPerimeter(),
								transitions={'reached':'MOVE_TO_TRASH_BIN_LOCATION_LINEAR', 
											 'not_reached':'MOVE_TO_TRASH_BIN_LOCATION_LINEAR',
											 'failed':'failed'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('MOVE_TO_TRASH_BIN_LOCATION_LINEAR', MoveToTrashBinLocationLinear(),
								transitions={'MTTBL_success':'APPROACH_PERIMETER_LINEAR'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('APPROACH_PERIMETER_LINEAR', ApproachPerimeter(mode='linear'),
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
			
			smach.StateMachine.add('APPROACH_PERIMETER_2', ApproachPerimeter(),
								transitions={'reached':'RELEASE_TRASH_BIN', 
											'not_reached':'RELEASE_TRASH_BIN',
											'failed':'failed'},
								remapping = {'trash_bin_pose_':'detection_pose'})
			
			smach.StateMachine.add('RELEASE_TRASH_BIN', ReleaseTrashBin(),
								transitions={'RTB_finished':'clear_trash_bin_done'})

		smach.StateMachine.add('CLEAR_TRASH_BIN', sm_sub_clear_waste_bin,
							transitions={'clear_trash_bin_done':'GO_TO_NEXT_UNPROCESSED_WASTE_BIN'})
		
		
		
		sm_sub_change_tool_hand = smach.StateMachine(outcomes=['CTH_done'],
													input_keys=['tool_wagon_pose'])
		with sm_sub_change_tool_hand:
			smach.StateMachine.add('GO_TO_TOOL_WAGON_LOCATION', MoveToToolWaggonFrontFar(),
								transitions={'arrived':'DETECT_SLOT_FOR_CURRENT_TOOL'})
			#smach.StateMachine.add('GO_TO_TOOL_WAGON_LOCATION',GoToToolWagonLocation() ,transitions={'GTTWL_done':'DETECT_SLOT_FOR_CURRENT_TOOL'})
			
			smach.StateMachine.add('DETECT_SLOT_FOR_CURRENT_TOOL',DetectSlotForCurrentTool() ,transitions={'slot_pose':'MOVE_ARM_TO_SLOT'})
			
			smach.StateMachine.add('MOVE_ARM_TO_SLOT',MoveArmToSlot() ,transitions={'MATS_done':'RELEASE_TOOL_CHANGER'})
			
			smach.StateMachine.add('RELEASE_TOOL_CHANGER',ReleaseToolChanger() ,transitions={'RTC_done':'LIFT_ARM'})
			
			smach.StateMachine.add('LIFT_ARM',LiftArm() ,transitions={'LA_done':'DETECT_SLOT_FOR_NEW_DEVICE'})
			
			smach.StateMachine.add('DETECT_SLOT_FOR_NEW_DEVICE',DetectSlotForNewDevice() ,transitions={'DSFND_slot_pose':'MOVE_ARM_TO_SLOT'})
			
			smach.StateMachine.add('MOVE_ARM_TO_SLOT_2',MoveArmToSlot2() ,transitions={'MATS2_done':'CLOSE_TOOL_CHANGER'})
			
			smach.StateMachine.add('CLOSE_TOOL_CHANGER',CloseToolChanger() ,transitions={'CTC_done':'MOVE_ARM_TO_STANDARD_LOCATION'})
			
			smach.StateMachine.add('MOVE_ARM_TO_STANDARD_LOCATION',MoveArmToStandardLocation() ,transitions={'MATSL_done':'CTH_done'})

		smach.StateMachine.add('CHANGE_TOOL_HAND', sm_sub_change_tool_hand,
							transitions={'CTH_done':'GET_DIRT_MAP'})


		sm_sub_change_tool_manual = smach.StateMachine(outcomes=['tool_change_done'],
														input_keys=['tool_wagon_pose'])
		with sm_sub_change_tool_manual:
			smach.StateMachine.add('GO_TO_TOOL_WAGON_LOCATION', MoveToToolWaggonFrontFar(),
								transitions={'arrived':'CHANGE_TOOL_MANUAL_IMPLEMENTATION'})
			
			smach.StateMachine.add('CHANGE_TOOL_MANUAL_IMPLEMENTATION', ChangeToolManualPnP(),
								transitions={'CTM_done':'tool_change_done'})
		
		smach.StateMachine.add('CHANGE_TOOL_MANUAL', sm_sub_change_tool_manual,
							transitions={'tool_change_done':'GET_DIRT_MAP'})
		
		
		smach.StateMachine.add('GET_DIRT_MAP', ReceiveDirtMap(),
							transitions={'list_of_dirt_location':'GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION'})
		
		
		
		sm_sub_go_to_next_unprocessed_dirt_location = smach.StateMachine(outcomes=['no_dirt_spots_left','arrived_dirt_location', 'failed'],
																		input_keys=['list_of_dirt_locations', 'last_visited_dirt_location'],
																		output_keys=['next_dirt_location'])

		with sm_sub_go_to_next_unprocessed_dirt_location:
			smach.StateMachine.add('SELECT_NEXT_UNPROCESSED_DIRT_SPOT', SelectNextUnprocssedDirtSpot(),
								transitions={'selected_next_dirt_location':'MOVE_TO_DIRT_LOCATION_PERIMETER_CLEANING',
											'no_dirt_spots_left':'no_dirt_spots_left'})
			
			smach.StateMachine.add('MOVE_TO_DIRT_LOCATION_PERIMETER_CLEANING', MoveLocationPerimeterCleaning(),
								transitions={'movement_prepared':'APPROACH_PERIMETER_CLEANING'})
			
			smach.StateMachine.add('APPROACH_PERIMETER_CLEANING', ApproachPerimeter(),
								transitions={'reached':'arrived_dirt_location', 
											 'not_reached':'SELECT_NEXT_UNPROCESSED_DIRT_SPOT',
											 'failed':'failed'})

		smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION', sm_sub_go_to_next_unprocessed_dirt_location,
							transitions={'arrived_dirt_location':'CLEAN',
										'no_dirt_spots_left':'DIRT_DETECTION_ON'})
		
		
		
		smach.StateMachine.add('CLEAN', Clean(), transitions={'cleaning_done':'GO_TO_INSPECT_LOCATION'})
		
		
		
		sm_sub_go_to_inspect_location = smach.StateMachine(outcomes=['arrived_cleaning_inspection_location', 'not_reached', 'failed'],
														input_keys=['next_dirt_location'])

		with sm_sub_go_to_inspect_location:
			smach.StateMachine.add('MOVE_TO_DIRT_LOCATION_PERIMETER_VALIDATION', MoveLocationPerimeterValidation(),
								transitions={'movement_prepared':'APPROACH_PERIMETER_VALIDATION'})
			
			smach.StateMachine.add('APPROACH_PERIMETER_VALIDATION', ApproachPerimeter(),
								transitions={'reached':'arrived_cleaning_inspection_location', 
											 'not_reached':'not_reached',
											 'failed':'failed'})

		smach.StateMachine.add('GO_TO_INSPECT_LOCATION', sm_sub_go_to_inspect_location,
							transitions={'arrived_cleaning_inspection_location':'VERIFY_CLEANING_SUCCESS',
										'not_reached':'GO_TO_NEXT_UNPROCESSED_DIRT_LOCATION',
										'failed':'failed'})
		
		
		
		smach.StateMachine.add('VERIFY_CLEANING_SUCCESS', VerifyCleaningProcess(),
							transitions={'verify_cleaning_done':'DIRT_DETECTION_ON'})
		
		
		
		smach.StateMachine.add('PROCESS_CLEANING_VERIFICATION_RESULTS', ProcessCleaningVerificationResults(),
							transitions={'finished':'GO_TO_TOOL_WAGON_LOCATION_2'})

		smach.StateMachine.add('GO_TO_TOOL_WAGON_LOCATION_2', MoveToToolWaggonFrontFrontalFar(),
								transitions={'arrived':'CHANGE_TOOL_MANUAL_IMPLEMENTATION_2'})
			
		smach.StateMachine.add('CHANGE_TOOL_MANUAL_IMPLEMENTATION_2', ChangeToolManualPnP(current_tool='vacuum'),
								transitions={'CTM_done':'finished'})
		
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_scenario, '/START')
	sis.start()
	
	# Execute SMACH plan
	outcome = sm_scenario.execute()
	
	#rospy.spin()
	
	sis.stop()


if __name__ == '__main__':
	try:
		flag = ""
		if len(sys.argv)<2:
			CONFIRM_MODE = True
		else:
			flag=sys.argv[1]
		if len(flag)==0:
			CONFIRM_MODE = True

		if flag=="auto":
			CONFIRM_MODE = False  # in auto mode the connection to the control pc may interrupted, the scenario will stil work
			# necessitates that all nodes are started with & and the end, e.g. bringup &
		else:
			CONFIRM_MODE = True

		print "CONFIRM_MODE ", CONFIRM_MODE
		main(CONFIRM_MODE)
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
