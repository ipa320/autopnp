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

from autopnp_scenario_states_sim import *


def main(confirm):
	rospy.init_node('exploration_detection_cleaning')
		
	# full scenario
	sm_scenario = smach.StateMachine(outcomes=['finished', 'failed'])
	sm_scenario.userdata.sm_trash_bin_counter = 0

	with sm_scenario:

		smach.StateMachine.add('INITIALIZE_AUTOPNP_SCENARIO', InitAutoPnPScenario(confirm_mode=confirm),
							transitions={'initialized':'ANALYZE_MAP',
										'failed':'failed'})
		
		smach.StateMachine.add('ANALYZE_MAP', AnalyzeMap(),
							transitions={'list_of_rooms':'GO_TO_NEXT_UNPROCESSED_ROOM',
										 'failed':'ANALYZE_MAP'},
							remapping={'analyze_map_segmented_map_':'segmented_map',
										'analyze_map_map_resolution_': 'map_resolution',
										'analyze_map_room_information_in_meter_': 'room_information_in_meter',
										'analyze_map_room_sequence_': 'room_sequence'})




		sm_sub_go_to_next_unproccessed_room = smach.StateMachine(outcomes=['arrived','no_more_rooms_left'],
																input_keys=['room_sequence',
																			'room_information_in_meter',
																			'current_clique_index',
																			'current_room_index',
																			'segmented_map',
																			'tool_wagon_pose',
																			'map_resolution'],
																output_keys=['current_clique_index',
																			'current_room_index',
																			'tool_wagon_pose'])
		
		#sm_sub_go_to_next_unproccessed_room.userdata.sm_counter = 1
		#sm_sub_go_to_next_unproccessed_room.userdata.sm_location_counter = 0
		
		with sm_sub_go_to_next_unproccessed_room:
			smach.StateMachine.add('FIND_NEXT_ROOM', NextUnprocessedRoom(),
								transitions={'next_room':'COMPUTE_NAVIGATION_GOALS',
											'move_trolley': 'MOVE_TOOL_WAGON',
											'no_rooms_left':'no_more_rooms_left'})

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
								transitions={'finished':'FIND_NEXT_ROOM'})
			
			smach.StateMachine.add('COMPUTE_NAVIGATION_GOALS', ComputeNavigationGoals(),
								transitions={'successful':'APPROACH_POSES'})
			
			smach.StateMachine.add('APPROACH_POSES', ApproachPoses(),
								transitions={'reached':'arrived', 
											 'not_reached':'FIND_NEXT_ROOM',
											 'failed':'FIND_NEXT_ROOM'})

		smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_ROOM', sm_sub_go_to_next_unproccessed_room,
							transitions={'arrived':'DIRT_DETECTION_ON',
										'no_more_rooms_left':'PROCESS_CLEANING_VERIFICATION_RESULTS'})
		
		
		
		
		
		smach.StateMachine.add('DIRT_DETECTION_ON', DirtDetectionOn(),
							transitions={'dirt_detection_on':'TRASH_BIN_DETECTION_ON'})
		
		smach.StateMachine.add('TRASH_BIN_DETECTION_ON', TrashBinDetectionOn(),
							transitions={'trash_bin_detection_on':'INSPECT_ROOM'})
		
		smach.StateMachine.add('INSPECT_ROOM', InspectRoom(),
							transitions={'finished':'DIRT_DETECTION_OFF'},
							remapping={'inspect_room_data_img_in_':'segmented_map',
										'inspect_room_img_out_':'segmented_map',
										'inspect_room_room_number_':'sm_RoomNo'})

		smach.StateMachine.add('DIRT_DETECTION_OFF', DirtDetectionOff(),
							transitions={'dirt_detection_off':'TRASH_BIN_DETECTION_OFF'})
		
		smach.StateMachine.add('TRASH_BIN_DETECTION_OFF', TrashBinDetectionOff(),
							transitions={'trash_can_found':'CLEAR_WASTE_BIN_SIM',
										'trash_can_not_found':'GO_TO_NEXT_UNPROCESSED_ROOM'})

		smach.StateMachine.add('GO_TO_WASTE_BIN_SIM', ApproachPoses(),
							transitions={'reached':'CLEAR_WASTE_BIN_SIM', 
										 'not_reached':'CLEAR_WASTE_BIN_SIM',
										 'failed':'CLEAR_WASTE_BIN_SIM'})
		
		smach.StateMachine.add('CLEAR_WASTE_BIN_SIM', MoveToToolWaggonFront(),
								transitions={'arrived':'RETURN_WASTE_BIN_SIM'})

		smach.StateMachine.add('RETURN_WASTE_BIN_SIM', ApproachPoses(),
							transitions={'reached':'GO_TO_NEXT_UNPROCESSED_ROOM', 
										 'not_reached':'GO_TO_NEXT_UNPROCESSED_ROOM',
										 'failed':'GO_TO_NEXT_UNPROCESSED_ROOM'})
		
# 		smach.StateMachine.add('GO_TO_NEXT_UNPROCESSED_WASTE_BIN', GoToNextUnprocessedWasteBin(),
# 							transitions={'go_to_trash_location':'CLEAR_TRASH_BIN',
# 										'all_trash_bins_cleared':'GO_TO_NEXT_UNPROCESSED_ROOM'},
# 							remapping={'go_to_next_unprocessed_waste_bin_in_':'trash_detection_poses',
# 										'go_to_next_unprocessed_waste_bin_out_':'detection_pose',
# 										'number_of_unprocessed_trash_bin_in_':'sm_trash_bin_counter',
# 										'number_of_unprocessed_trash_bin_out_':'sm_trash_bin_counter'})
# 
# 
# 		sm_sub_clear_waste_bin = smach.StateMachine(outcomes=['clear_trash_bin_done', 'failed'],input_keys=['detection_pose', 'tool_wagon_pose'])
# 
# 		with sm_sub_clear_waste_bin:
# 			smach.StateMachine.add('MOVE_TO_TRASH_BIN_LOCATION', MoveToTrashBinLocation(),
# 								transitions={'MTTBL_success':'APPROACH_PERIMETER'},
# 								remapping = {'trash_bin_pose_':'detection_pose'})
# 			
# 			smach.StateMachine.add('APPROACH_PERIMETER', ApproachPerimeter(),
# 								transitions={'reached':'MOVE_TO_TRASH_BIN_LOCATION_LINEAR', 
# 											 'not_reached':'MOVE_TO_TRASH_BIN_LOCATION_LINEAR',
# 											 'failed':'failed'},
# 								remapping = {'trash_bin_pose_':'detection_pose'})
# 			
# 			smach.StateMachine.add('MOVE_TO_TRASH_BIN_LOCATION_LINEAR', MoveToTrashBinLocationLinear(),
# 								transitions={'MTTBL_success':'APPROACH_PERIMETER_LINEAR'},
# 								remapping = {'trash_bin_pose_':'detection_pose'})
# 			
# 			smach.StateMachine.add('APPROACH_PERIMETER_LINEAR', ApproachPerimeter(mode='linear'),
# 								transitions={'reached':'CHECK_POSITION_TO_TRASH_BIN', 
# 											 'not_reached':'CHECK_POSITION_TO_TRASH_BIN',
# 											 'failed':'failed'},
# 								remapping = {'trash_bin_pose_':'detection_pose'})
# 			
# 			smach.StateMachine.add('CHECK_POSITION_TO_TRASH_BIN', CheckPositionToTrashBinLocation(),
# 								transitions={'success':'GRASP_TRASH_BIN',
# 											 'failed':'MOVE_TO_TRASH_BIN_LOCATION_LINEAR'},
# 								remapping = {'trash_bin_pose_':'detection_pose'})
# 			
# 			smach.StateMachine.add('GRASP_TRASH_BIN', GraspTrashBin(),
# 								transitions={'GTB_success':'MOVE_TO_TOOL_WAGON_FRONTAL',
# 											 'failed':'failed'})
# 			
# 			smach.StateMachine.add('MOVE_TO_TOOL_WAGON_FRONTAL', MoveToToolWaggonFrontFrontalFar(),
# 								transitions={'arrived':'MOVE_TO_TOOL_WAGON_TURN180'})
# 			
# 			smach.StateMachine.add('MOVE_TO_TOOL_WAGON_TURN180', Turn180(),
# 								transitions={'arrived':'CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART1'})
# 			
# 			smach.StateMachine.add('CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART1', ClearTrashBinIntoToolWagonPart1(),
# 								transitions={'finished':'MOVE_TO_TOOL_WAGON_FRONTAL_TRASH_BIN_CLEARING'})
# 			
# 			smach.StateMachine.add('MOVE_TO_TOOL_WAGON_FRONTAL_TRASH_BIN_CLEARING', MoveToToolWaggonFrontTrashClearing(),
# 								transitions={'arrived':'CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART2'})
# 			
# 			smach.StateMachine.add('CLEAR_TRASH_BIN_INTO_TOOL_WAGON_PART2', ClearTrashBinIntoToolWagonPart2(),
# 								transitions={'finished':'MOVE_TO_TRASH_BIN_PICKING_LOCATION'})
# 			
# 			smach.StateMachine.add('MOVE_TO_TRASH_BIN_PICKING_LOCATION', MoveToTrashBinPickingLocation(),
# 								transitions={'MTTBPL_done':'APPROACH_PERIMETER_2'},
# 								remapping = {'trash_bin_pose_':'detection_pose'})
# 			
# 			smach.StateMachine.add('APPROACH_PERIMETER_2', ApproachPerimeter(),
# 								transitions={'reached':'RELEASE_TRASH_BIN', 
# 											'not_reached':'RELEASE_TRASH_BIN',
# 											'failed':'failed'},
# 								remapping = {'trash_bin_pose_':'detection_pose'})
# 			
# 			smach.StateMachine.add('RELEASE_TRASH_BIN', ReleaseTrashBin(),
# 								transitions={'RTB_finished':'clear_trash_bin_done'})
# 
# 		smach.StateMachine.add('CLEAR_TRASH_BIN', sm_sub_clear_waste_bin,
# 							transitions={'clear_trash_bin_done':'GO_TO_NEXT_UNPROCESSED_WASTE_BIN'})


		
		smach.StateMachine.add('PROCESS_CLEANING_VERIFICATION_RESULTS', ProcessCleaningVerificationResults(),
							transitions={'finished':'finished'})
	
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

		if flag=="auto":
			CONFIRM_MODE = False  # in auto mode the connection to the control pc may interrupted, the scenario will stil work
			# necessitates that all nodes are started with & and the end, e.g. bringup &
		elif flag=="special":
			CONFIRM_MODE = False
		else:
			CONFIRM_MODE = True

		print "CONFIRM_MODE ", CONFIRM_MODE
		main(CONFIRM_MODE)
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
