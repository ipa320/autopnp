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

import roslib; roslib.load_manifest('autopnp_scenario') #; roslib.load_manifest('cob_navigation_global')
import rospy
import smach
import smach_ros
import threading
import tf

import dynamic_reconfigure.client

from map_segmentation_action_client import MapSegmentationActionClient
from find_next_unprocessed_room_action_client import find_next_unprocessed_room
from go_to_room_location_action_client import go_to_room_location
from random_location_finder_action_client import random_location_finder_client
from inspect_room_action_client import inspect_room

from autopnp_scenario.srv import *
from autopnp_scenario.msg import *
from autopnp_dirt_detection.srv import *
from cob_object_detection_msgs.msg import DetectionArray, Detection
from geometry_msgs import *
from cob_phidgets.srv import SetDigitalSensor

from ApproachPerimeter import *

from simple_script_server import simple_script_server
sss = simple_script_server()


#-------------------------------------------------------- Global Definitions ---------------------------------------------------------------------------------------
global MAX_TOOL_WAGON_DISTANCE_TO_NEXT_ROOM
MAX_TOOL_WAGON_DISTANCE_TO_NEXT_ROOM = 0.20000 # maximum allowed distance of tool wagon to next target room center, if exceeded, tool wagon needs to be moved [in m]

global TOOL_WAGON_LOCATIONS
TOOL_WAGON_LOCATIONS=[]  # valid parking positions of tool wagon in current map [in m and rad]
TOOL_WAGON_LOCATIONS.append(Pose2D(x=0, y=0, theta=0))
TOOL_WAGON_LOCATIONS.append(Pose2D(x=3, y=3, theta=0))

global TOOL_WAGON_MARKER_OFFSETS
TOOL_WAGON_MARKER_OFFSETS={ #todo: measure translational offsets
						"front":	Transform(translation=Vector3(x=0.055, y=-0.975, z=0.0), rotation=Quaternion(x=-0.5, y=-0.5, z=-0.5, w=0.5)),   # quaternion_from_euler(-90.0/180.0*math.pi, -90.0/180.0*math.pi, 0.0, 'rzyx')
						"rear":		Transform(translation=Vector3(x=0.06, y=-1.025, z=-0.46), rotation=Quaternion(x=-0.5, y=0.5, z=0.5, w=0.5)),   # quaternion_from_euler(90.0/180.0*math.pi, 90.0/180.0*math.pi, 0.0, 'rzyx')
						"left":		Transform(translation=Vector3(x=-0.17, y=-1.0, z=-0.275), rotation=Quaternion(x=0.0, y=0.707106781, z=0.707106781, w=0.0)),   # quaternion_from_euler(math.pi, 0.0, 90.0/180.0*math.pi, 'rzyx')
						"right":	Transform(translation=Vector3(x=0.29, y=-1.0, z=-0.275), rotation=Quaternion(x=-0.70710678, y=0.0, z=0.0, w=0.70710678))   # quaternion_from_euler(0.0, 0.0, -90.0/180.0*math.pi, 'rzyx')
						}  # offset transformations from respective markers to tool wagon base/center coordinate system

global TOOL_WAGON_ROBOT_OFFSETS
TOOL_WAGON_ROBOT_OFFSETS={
						"front":	Pose2D(x=-1.1, y=0.0, theta=0.0),
						"rear":		Pose2D(x=-1.0, y=0.0, theta=math.pi)
						}  # describes the offset of the tool wagon center with respect to base_link (x-axis in tool wagon is directed to the front, y-axis to the left)

global FIDUCIALS_MARKER_DICTIONARY
FIDUCIALS_MARKER_DICTIONARY={
						"tag_tool_wagon_front_l":		"tag_48",  # left = +y of tool wagon base frame
						"tag_tool_wagon_front_c":		"tag_55",
						"tag_tool_wagon_front_r":		"tag_36",  # right = -y
						"tag_tool_wagon_rear_l":		"tag_38",  # left = +y
						"tag_tool_wagon_rear_c":		"tag_79",
						"tag_tool_wagon_rear_r":		"tag_73",  # right = -y
						"tag_tool_wagon_right":			"tag_64",
						"tag_tool_wagon_left":			"tag_69",
						"trash_bin":					"tag_25"
						}

#-------------------------------------------------------- Exploration Algorithm ---------------------------------------------------------------------------------------

###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
_tl=None
_tl_creation_lock=threading.Lock()

def get_transform_listener():
	global _tl
	with _tl_creation_lock:
		if _tl==None:
			_tl=tf.TransformListener(True, rospy.Duration(40.0))
		return _tl
#################################################################################

class InitAutoPnPScenario(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['initialized', 'failed'],
			input_keys=[],
			output_keys=['tool_wagon_pose'])
		
	def execute(self, userdata):
		sf = ScreenFormat("InitAutoPnPScenario")
		
		tool_wagon_pose = Pose2D()
		try:
			listener = get_transform_listener()
			t = rospy.Time(0)
			listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
			(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			print "Could not lookup robot pose: %s" %e
			return 'failed'
		tool_wagon_offset = TOOL_WAGON_ROBOT_OFFSETS["front"]   # in meter
		robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx') # yields yaw, pitch, roll
		tool_wagon_pose.theta = robot_pose_rotation_euler[0] + tool_wagon_offset.theta
		tool_wagon_pose.x = robot_pose_translation[0] + tool_wagon_offset.x*math.cos(robot_pose_rotation_euler[0]) - tool_wagon_offset.y*math.sin(robot_pose_rotation_euler[0])
		tool_wagon_pose.y = robot_pose_translation[1] + tool_wagon_offset.x*math.sin(robot_pose_rotation_euler[0]) + tool_wagon_offset.y*math.cos(robot_pose_rotation_euler[0])
		
		print 'robot_pose: ', (robot_pose_translation[0], robot_pose_translation[1], robot_pose_rotation_euler[0])
		print 'tool_wagon_pose ', tool_wagon_pose
		
		userdata.tool_wagon_pose = tool_wagon_pose
		
		return 'initialized'
			
			

# The AnalyzeMap class defines a state machine of smach which basically 
# call the map segmentation action client object and execute the function
# of action client to communicate with action server
class AnalyzeMap(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['list_of_rooms'], output_keys=['analyze_map_data_img_',
																			'analyze_map_data_map_resolution_',  # in m/cell
																			'analyze_map_data_map_origin_x_',    # in m (pose of cell 0,0 in real world)
																			'analyze_map_data_map_origin_y_',
																			'analyze_map_data_room_center_x_', 
																			'analyze_map_data_room_center_y_',
																			'analyze_map_data_room_min_x_',
																			'analyze_map_data_room_max_x_',
																			'analyze_map_data_room_min_y_',
																			'analyze_map_data_room_max_y_'])

	def execute(self, userdata):
		sf = ScreenFormat("AnalyzeMap")
		map_segmentation_action_client_object_ = MapSegmentationActionClient()
		
		rospy.sleep(1)
		rospy.loginfo('Executing state analyze map.....')
		
		map_segmentation_action_server_result_ = map_segmentation_action_client_object_.map_segmentation_action_client_()
		
		userdata.analyze_map_data_img_ = map_segmentation_action_server_result_.output_map
		userdata.analyze_map_data_map_resolution_ = map_segmentation_action_server_result_.map_resolution
		userdata.analyze_map_data_map_origin_x_ = map_segmentation_action_server_result_.map_origin_x
		userdata.analyze_map_data_map_origin_y_ = map_segmentation_action_server_result_.map_origin_y
		userdata.analyze_map_data_room_center_x_ = map_segmentation_action_server_result_.room_center_x_in_pixel
		userdata.analyze_map_data_room_center_y_ = map_segmentation_action_server_result_.room_center_y_in_pixel
		userdata.analyze_map_data_room_min_x_ = map_segmentation_action_server_result_.room_min_x_in_pixel
		userdata.analyze_map_data_room_max_x_ = map_segmentation_action_server_result_.room_max_x_in_pixel
		userdata.analyze_map_data_room_min_y_ = map_segmentation_action_server_result_.room_min_y_in_pixel
		userdata.analyze_map_data_room_max_y_ = map_segmentation_action_server_result_.room_max_y_in_pixel
		
		return 'list_of_rooms'  

# The NextUnprocessedRoom class defines a state machine of smach which basically 
# call the find next unprocessed room action client object and execute the function
# of action client to communicate with action server
class NextUnprocessedRoom(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['location','no_rooms','arrived'],input_keys=['find_next_unprocessed_room_data_img_',
																						'analyze_map_data_map_resolution_',
																						'analyze_map_data_map_origin_x_',
																						'analyze_map_data_map_origin_y_',
																						'analyze_map_data_room_center_x_',	# in pixel
																						'analyze_map_data_room_center_y_',    # in pixel
																						'find_next_unprocessed_room_number_in_',
																						'find_next_unprocessed_room_loop_counter_in_'],
																			output_keys=['find_next_unprocessed_room_number_out_',   # a vector with next target room at back
																						'find_next_unprocessed_room_loop_counter_out_',
																						'find_next_unprocessed_room_center_x_',   # in pixel
																						'find_next_unprocessed_room_center_y_'])  # in pixel

	def execute(self, userdata ):
		sf = ScreenFormat("NextUnprocessedRoom")
		#rospy.sleep(10)
		rospy.loginfo('Executing state next unprocessed room.....')
		if userdata.find_next_unprocessed_room_loop_counter_in_ <= len(userdata.analyze_map_data_room_center_x_):
			find_next_unprocessed_room_action_server_result_ = find_next_unprocessed_room( userdata.find_next_unprocessed_room_data_img_,
																							userdata.analyze_map_data_room_center_x_,   # in pixel
																							userdata.analyze_map_data_room_center_y_,   # in pixel
																							userdata.analyze_map_data_map_resolution_,
																							userdata.analyze_map_data_map_origin_x_,
																							userdata.analyze_map_data_map_origin_y_ )
			#rospy.sleep(10)
			userdata.find_next_unprocessed_room_number_out_ = find_next_unprocessed_room_action_server_result_.room_number
			rospy.loginfo('Current room No: %d'%userdata.find_next_unprocessed_room_loop_counter_in_)  
			userdata.find_next_unprocessed_room_loop_counter_out_ = userdata.find_next_unprocessed_room_loop_counter_in_ + 1  
			userdata.find_next_unprocessed_room_center_x_ = find_next_unprocessed_room_action_server_result_.center_position_x
			userdata.find_next_unprocessed_room_center_y_ = find_next_unprocessed_room_action_server_result_.center_position_y
			return 'location'
		else:
			return 'no_rooms'


# The GoToRoomLocation class defines a state machine of smach which basically 
# call the go to room location action client object and execute the function
# of action client to communicate with action server     
class GoToRoomLocation(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['successful','unsuccessful'],input_keys=['go_to_room_location_data_img_',
																					'analyze_map_data_map_resolution_',
																					'analyze_map_data_map_origin_x_',
																					'analyze_map_data_map_origin_y_',
																					'find_next_unprocessed_room_center_x_',   # in pixel
																					'find_next_unprocessed_room_center_y_',   # in pixel
																					'random_location_finder_random_location_x_',
																					'random_location_finder_random_location_y_',
																					'go_to_room_location_loop_counter_in_'],
																		output_keys=['go_to_room_location_loop_counter_out_'])

	def execute(self, userdata):
		sf = ScreenFormat("GoToRoomLocation")
		#rospy.sleep(10)
		rospy.loginfo('Executing state go to room location')
		if userdata.go_to_room_location_loop_counter_in_ == 0 :
			go_to_room_location_action_server_result_ = go_to_room_location(userdata.go_to_room_location_data_img_,
																			userdata.find_next_unprocessed_room_center_x_ , 
																			userdata.find_next_unprocessed_room_center_y_,
																			userdata.analyze_map_data_map_resolution_,
																			userdata.analyze_map_data_map_origin_x_,
																			userdata.analyze_map_data_map_origin_y_ )
		else:
			go_to_room_location_action_server_result_ = go_to_room_location(userdata.go_to_room_location_data_img_,
																			userdata.random_location_finder_random_location_x_,
																			userdata.random_location_finder_random_location_y_,
																			userdata.analyze_map_data_map_resolution_,
																			userdata.analyze_map_data_map_origin_x_,
																			userdata.analyze_map_data_map_origin_y_)

		if go_to_room_location_action_server_result_.output_flag == 'True':
			return 'successful'
		else:
			userdata.go_to_room_location_loop_counter_out_ = userdata.go_to_room_location_loop_counter_in_ + 1
			return 'unsuccessful'


# The FindRandomLocation class defines a state machine of smach which basically 
# call the random location finder action client object and execute the function
# of action client to communicate with action server        
class FindRandomLocation(smach.State):
	def __init__(self):
		smach.State.__init__(self,outcomes=['re_locate','unsuccessful_five_times'],input_keys=['random_location_finder_data_img_in_',
																								'analyze_map_data_map_resolution_',
																								'analyze_map_data_map_origin_x_',
																								'analyze_map_data_map_origin_y_',
																								'random_location_finder_room_number_',
																								'analyze_map_data_room_min_x_',
																								'analyze_map_data_room_max_x_',
																								'analyze_map_data_room_min_y_',
																								'analyze_map_data_room_max_y_',
																								'random_location_finder_counter_in_'],
																					output_keys=['random_location_finder_random_location_x_',
																								'random_location_finder_random_location_y_',
																								'random_location_finder_counter_out_',
																								'random_location_finder_data_img_out_'])

	def execute(self, userdata):
		sf = ScreenFormat("FindRandomLocation")
		#rospy.sleep(10)
		rospy.loginfo('Executing state find random location')
		
		random_location_finder_action_server_result_ = random_location_finder_client(userdata.random_location_finder_data_img_in_,
																					userdata.analyze_map_data_map_resolution_,
																					userdata.analyze_map_data_map_origin_x_,
																					userdata.analyze_map_data_map_origin_y_ ,
																					userdata.random_location_finder_room_number_,
																					userdata.analyze_map_data_room_min_x_,
																					userdata.analyze_map_data_room_max_x_,
																					userdata.analyze_map_data_room_min_y_,
																					userdata.analyze_map_data_room_max_y_,
																					userdata.random_location_finder_counter_in_)

		userdata.random_location_finder_random_location_x_ = random_location_finder_action_server_result_.random_location_x
		userdata.random_location_finder_random_location_y_ = random_location_finder_action_server_result_.random_location_y
		if userdata.random_location_finder_counter_in_ < 6:
			return 're_locate'
		else:
			rospy.loginfo("unsuccessful rate: %d",userdata.random_location_finder_counter_in_)
			userdata.random_location_finder_counter_out_ = 0 ;
			userdata.random_location_finder_data_img_out_ = random_location_finder_action_server_result_.output_img
			return 'unsuccessful_five_times'


# The InspectRoom class defines a state machine of smach which basically 
# call the inspect room action client object and execute the function
# of action client to communicate with action server   
class InspectRoom(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished'],input_keys=['inspect_room_data_img_in_',
																	'inspect_room_room_number_',
																	'analyze_map_data_room_center_x_',
																	'analyze_map_data_room_center_y_',
																	'analyze_map_data_room_min_x_',
																	'analyze_map_data_room_max_x_',
																	'analyze_map_data_room_min_y_',
																	'analyze_map_data_room_max_y_',
																	'analyze_map_data_map_resolution_',
																	'analyze_map_data_map_origin_x_',
																	'analyze_map_data_map_origin_y_'],
														output_keys=['inspect_room_img_out_'])

	def execute(self, userdata):
		sf = ScreenFormat("InspectRoom")
		#rospy.sleep(10)
		rospy.loginfo('Executing state inspect_room')
		inspect_room_action_server_result_= inspect_room( userdata.inspect_room_data_img_in_,
														userdata.inspect_room_room_number_,
														userdata.analyze_map_data_room_center_x_,
														userdata.analyze_map_data_room_center_y_,
														userdata.analyze_map_data_room_min_x_,
														userdata.analyze_map_data_room_max_x_,
														userdata.analyze_map_data_room_min_y_,
														userdata.analyze_map_data_room_max_y_,
														userdata.analyze_map_data_map_resolution_,
														userdata.analyze_map_data_map_origin_x_,
														userdata.analyze_map_data_map_origin_y_)
		#rospy.sleep(10)
		userdata.inspect_room_img_out_ = inspect_room_action_server_result_.output_img
		return 'finished'


#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Move Tool Wagon ---------------------------------------------------------------------------------------

class VerifyToolCarLocation(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['no_need_to_move_it','tool_wagon_needs_to_be_moved'],
							input_keys=['find_next_unprocessed_room_center_x_',  # in pixel
										'find_next_unprocessed_room_center_y_',  # in pixel
										'tool_wagon_pose',  # in m
										'analyze_map_data_map_resolution_',
										'analyze_map_data_map_origin_x_',
										'analyze_map_data_map_origin_y_',],
							output_keys=['tool_wagon_goal_pose'])  # in m

	def execute(self, userdata ):
		sf = ScreenFormat("VerifyToolCarLocation")
		rospy.loginfo('Executing state Verify_Tool_Car_Location')

		# compare tool wagon location with next goal position (find_next_unprocessed_room_center_x_/y_) -> if to far, move tool wagon
		next_room_center_x_m = userdata.find_next_unprocessed_room_center_x_*userdata.analyze_map_data_map_resolution_ + userdata.analyze_map_data_map_origin_x_
		next_room_center_y_m = userdata.find_next_unprocessed_room_center_y_*userdata.analyze_map_data_map_resolution_ + userdata.analyze_map_data_map_origin_y_
		dist = math.sqrt((userdata.tool_wagon_pose.x-next_room_center_x_m)*(userdata.tool_wagon_pose.x-next_room_center_x_m) + (userdata.tool_wagon_pose.y-next_room_center_y_m)*(userdata.tool_wagon_pose.y-next_room_center_y_m))
		
		# compare to maximally allowed tool wagon distance
		if (dist > MAX_TOOL_WAGON_DISTANCE_TO_NEXT_ROOM):
			# select closest position where to move the tool wagon
			tool_wagon_goal_pose = Pose2D
			closest_goal_pose = 1e10
			for location in TOOL_WAGON_LOCATIONS:
				dist = math.sqrt((next_room_center_x_m-location.x)*(next_room_center_x_m-location.x) + (next_room_center_y_m-location.y)*(next_room_center_y_m-location.y))
				if closest_goal_pose > dist:
					closest_goal_pose = dist
					tool_wagon_goal_pose = location
			userdata.tool_wagon_goal_pose = tool_wagon_goal_pose
			return 'tool_wagon_needs_to_be_moved'
		
		return 'no_need_to_move_it'


def computeToolWagonPoseFromFiducials(fiducials):
	trROS = tf.TransformerROS(True, rospy.Duration(10.0))
	averaged_tool_wagon_pose = None
	averaged_tool_wagon_markers = 0.0
	for fiducial in fiducials.detections:
		# transform to base pose of tool wagon and then convert to map coordinate system
		offset = None
		if fiducial.label==FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_front_l"] or fiducial.label==FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_front_c"] or fiducial.label==FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_front_r"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['front']
		elif fiducial.label==FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_rear_l"] or fiducial.label==FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_rear_c"] or fiducial.label==FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_rear_r"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['rear']
		elif fiducial.label==FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_left"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['left']
		elif fiducial.label==FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_right"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['right']
		
		if offset != None:  # i.e. if a tool wagon marker was detected
			# compute tool wagon center
			offset_mat = trROS.fromTranslationRotation((offset.translation.x, offset.translation.y, offset.translation.z), (offset.rotation.x, offset.rotation.y, offset.rotation.z, offset.rotation.w))
			fiducial_pose_mat = trROS.fromTranslationRotation((fiducial.pose.pose.position.x, fiducial.pose.pose.position.y, fiducial.pose.pose.position.z), (fiducial.pose.pose.orientation.x, fiducial.pose.pose.orientation.y, fiducial.pose.pose.orientation.z, fiducial.pose.pose.orientation.w))
			tool_wagon_pose_mat = numpy.dot(fiducial_pose_mat, offset_mat)
			q = quaternion_from_matrix(tool_wagon_pose_mat)
			tool_wagon_pose = PoseStamped(header=fiducial.pose.header, pose=Pose(position=Point(x=tool_wagon_pose_mat[0,3], y=tool_wagon_pose_mat[1,3], z=tool_wagon_pose_mat[2,3]),
									orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
			#print "fidpose, offset: ", fiducial.pose, offset
			#print "twpose: ", tool_wagon_pose
			#print "twpose ypr: ", euler_from_quaternion([tool_wagon_pose.pose.orientation.x, tool_wagon_pose.pose.orientation.y, tool_wagon_pose.pose.orientation.z, tool_wagon_pose.pose.orientation.w], "rzyx")
			
			# transform to map system
			try:
				listener = get_transform_listener()
				listener.waitForTransform('/map', tool_wagon_pose.header.frame_id, tool_wagon_pose.header.stamp, rospy.Duration(10))
				tool_wagon_pose_map = listener.transformPose('/map', tool_wagon_pose)
				print 'tool_wagon_pose = ', tool_wagon_pose_map
				print "tool_wagon_pose ypr: ", euler_from_quaternion([tool_wagon_pose_map.pose.orientation.x, tool_wagon_pose_map.pose.orientation.y, tool_wagon_pose_map.pose.orientation.z, tool_wagon_pose_map.pose.orientation.w], "rzyx")
				# average position when multiple detections are present
				if averaged_tool_wagon_pose==None:
					averaged_tool_wagon_pose = tool_wagon_pose_map
					averaged_tool_wagon_markers = 1.0
				else:
					averaged_tool_wagon_pose.pose.position.x = averaged_tool_wagon_pose.pose.position.x + tool_wagon_pose_map.pose.position.x
					averaged_tool_wagon_pose.pose.position.y = averaged_too/cob_phidgets_toolchanger/ifk_toolchanger/set_digitall_wagon_pose.pose.position.y + tool_wagon_pose_map.pose.position.y
					averaged_tool_wagon_pose.pose.position.z = averaged_tool_wagon_pose.pose.position.z + tool_wagon_pose_map.pose.position.z
					averaged_tool_wagon_pose.pose.orientation.x = averaged_tool_wagon_pose.pose.orientation.x + tool_wagon_pose_map.pose.orientation.x
					averaged_tool_wagon_pose.pose.orientation.y = averaged_tool_wagon_pose.pose.orientation.y + tool_wagon_pose_map.pose.orientation.y
					averaged_tool_wagon_pose.pose.orientation.z = averaged_tool_wagon_pose.pose.orientation.z + tool_wagon_pose_map.pose.orientation.z
					averaged_tool_wagon_pose.pose.orientation.w = averaged_tool_wagon_pose.pose.orientation.w + tool_wagon_pose_map.pose.orientation.w
					averaged_tool_wagon_markers = averaged_tool_wagon_markers + 1.0
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
				print "computeToolWagonPoseFromFiducials: Could not lookup robot pose: %s" %e
	# finalize averaging and normalizationroslib.load_manifest(PACKAGE)
	if averaged_tool_wagon_markers > 0.0:
		averaged_tool_wagon_pose.pose.position.x = averaged_tool_wagon_pose.pose.position.x/averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.position.y = averaged_tool_wagon_pose.pose.position.y/averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.position.z = averaged_tool_wagon_pose.pose.position.z/averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.x = averaged_tool_wagon_pose.pose.orientation.x/averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.y = averaged_tool_wagon_pose.pose.orientation.y/averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.z = averaged_tool_wagon_pose.pose.orientation.z/averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.w = averaged_tool_wagon_pose.pose.orientation.w/averaged_tool_wagon_markers
		norm = math.sqrt(averaged_tool_wagon_pose.pose.orientation.x*averaged_tool_wagon_pose.pose.orientation.x + averaged_tool_wagon_pose.pose.orientation.y*averaged_tool_wagon_pose.pose.orientation.y + averaged_tool_wagon_pose.pose.orientation.z*averaged_tool_wagon_pose.pose.orientation.z + averaged_tool_wagon_pose.pose.orientation.w*averaged_tool_wagon_pose.pose.orientation.w)
		averaged_tool_wagon_pose.pose.orientation.x = averaged_tool_wagon_pose.pose.orientation.x/norm
		averaged_tool_wagon_pose.pose.orientation.y = averaged_tool_wagon_pose.pose.orientation.y/norm
		averaged_tool_wagon_pose.pose.orientation.z = averaged_tool_wagon_pose.pose.orientation.z/norm
		averaged_tool_wagon_pose.pose.orientation.w = averaged_tool_wagon_pose.pose.orientation.w/norm
		
		return averaged_tool_wagon_pose

class MoveToToolWaggonFront(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'], input_keys=['tool_wagon_pose'])
		self.tool_wagon_pose = None

	def fiducial_callback(self, fiducials):
		self.tool_wagon_pose = computeToolWagonPoseFromFiducials(fiducials)

	def execute(self, userdata):
		sf = ScreenFormat("MoveToToolWaggonFront")
		#rospy.loginfo('Executing state Move_Base_To_Last_Tool_Waggon_Location')

		sss.move("head", "back")
		sss.move("torso", "home")

		self.tool_wagon_pose = None
		fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)
		
		# 1. move to last known position (i.e. move_base to tool_wagon_pose + robot offset)
		robot_offset = TOOL_WAGON_ROBOT_OFFSETS["front"]
		dx = -(robot_offset.x*math.cos(robot_offset.theta)+robot_offset.y*math.sin(robot_offset.theta))
		dy = -(-robot_offset.x*math.sin(robot_offset.theta)+robot_offset.y*math.cos(robot_offset.theta))
		robot_pose = Pose2D(x = userdata.tool_wagon_pose.x + dx*math.cos(userdata.tool_wagon_pose.theta)-dy*math.sin(userdata.tool_wagon_pose.theta),
							y = userdata.tool_wagon_pose.y + dx*math.sin(userdata.tool_wagon_pose.theta)+dy*math.cos(userdata.tool_wagon_pose.theta),
							theta = userdata.tool_wagon_pose.theta - robot_offset.theta)
		handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta])
		
		# 2. detect fiducials and move to corrected pose
		# wait until wagon is detected
		robot_approximately_well_positioned = False
		while robot_approximately_well_positioned==False:
			while self.tool_wagon_pose == None:
				rospy.sleep(0.2)
			# move to corrected pose
			tool_wagon_pose_3d = self.tool_wagon_pose.pose
			tool_wagon_pose_3d_euler = tf.transformations.euler_from_quaternion([tool_wagon_pose_3d.orientation.x,tool_wagon_pose_3d.orientation.y,tool_wagon_pose_3d.orientation.z,tool_wagon_pose_3d.orientation.w], 'rzyx') # yields yaw, pitch, roll
			tool_wagon_pose = Pose2D(x=tool_wagon_pose_3d.position.x, y=tool_wagon_pose_3d.position.y, theta=tool_wagon_pose_3d_euler[0])
			robot_goal_pose = Pose2D(x=tool_wagon_pose.x + dx*math.cos(tool_wagon_pose.theta)-dy*math.sin(tool_wagon_pose.theta),
									y=tool_wagon_pose.y + dx*math.sin(tool_wagon_pose.theta)+dy*math.cos(tool_wagon_pose.theta),
									theta=tool_wagon_pose.theta - robot_offset.theta)
			handle_base = sss.move("base", [float(robot_goal_pose.x), float(robot_goal_pose.y), float(robot_goal_pose.theta)])
			
			# read out current robot pose
			try:
				listener = get_transform_listener()
				t = rospy.Time(0)
				listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
				(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
				print "Could not lookup robot pose: %s" %e
				continue
			robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx') # yields yaw, pitch, roll
			
			# verify distance to goal pose
			dist = math.sqrt((robot_pose_translation[0]-robot_goal_pose.x)*(robot_pose_translation[0]-robot_goal_pose.x) + (robot_pose_translation[1]-robot_goal_pose.y)*(robot_pose_translation[1]-robot_goal_pose.y))
			print "(x,y)-dist: ", dist
			if dist > 0.02:		# in m
				self.tool_wagon_pose = None
			else:
				robot_approximately_well_positioned = True
		
		# 3. unsubscribe to fiducials
		fiducials_sub.unregister()
		
		return 'arrived'

class MoveToToolWaggonRear(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'], input_keys=['tool_wagon_pose'])
		self.tool_wagon_pose = None

	def fiducial_callback(self, fiducials):
		self.tool_wagon_pose = computeToolWagonPoseFromFiducials(fiducials)

	def execute(self, userdata):
		sf = ScreenFormat("MoveToToolWaggonFront")
		#rospy.loginfo('Executing state Move_Base_To_Last_Tool_Waggon_Location')

		sss.move("head", "back")
		sss.move("torso", "home")

		self.tool_wagon_pose = None
		fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)
		
		# 1. move to last known position (i.e. move_base to tool_wagon_pose + robot offset)
		robot_offset = TOOL_WAGON_ROBOT_OFFSETS["rear"]
		dx = -(robot_offset.x*math.cos(robot_offset.theta)+robot_offset.y*math.sin(robot_offset.theta))
		dy = -(-robot_offset.x*math.sin(robot_offset.theta)+robot_offset.y*math.cos(robot_offset.theta))
		robot_pose = Pose2D(x = userdata.tool_wagon_pose.x + dx*math.cos(userdata.tool_wagon_pose.theta)-dy*math.sin(userdata.tool_wagon_pose.theta),
							y = userdata.tool_wagon_pose.y + dx*math.sin(userdata.tool_wagon_pose.theta)+dy*math.cos(userdata.tool_wagon_pose.theta),
							theta = userdata.tool_wagon_pose.theta - robot_offset.theta)
		handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta])
		
		# 2. detect fiducials and move to corrected pose
		# wait until wagon is detected
		robot_approximately_well_positioned = False
		while robot_approximately_well_positioned==False:
			while self.tool_wagon_pose == None:
				rospy.sleep(0.2)
			# move to corrected pose
			tool_wagon_pose_3d = self.tool_wagon_pose.pose
			tool_wagon_pose_3d_euler = tf.transformations.euler_from_quaternion([tool_wagon_pose_3d.orientation.x,tool_wagon_pose_3d.orientation.y,tool_wagon_pose_3d.orientation.z,tool_wagon_pose_3d.orientation.w], 'rzyx') # yields yaw, pitch, roll
			tool_wagon_pose = Pose2D(x=tool_wagon_pose_3d.position.x, y=tool_wagon_pose_3d.position.y, theta=tool_wagon_pose_3d_euler[0])
			robot_goal_pose = Pose2D(x=tool_wagon_pose.x + dx*math.cos(tool_wagon_pose.theta)-dy*math.sin(tool_wagon_pose.theta),
									y=tool_wagon_pose.y + dx*math.sin(tool_wagon_pose.theta)+dy*math.cos(tool_wagon_pose.theta),
									theta=tool_wagon_pose.theta - robot_offset.theta)
			handle_base = sss.move("base", [float(robot_goal_pose.x), float(robot_goal_pose.y), float(robot_goal_pose.theta)])
			
			# read out current robot pose
			try:
				listener = get_transform_listener()
				t = rospy.Time(0)
				listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
				(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
				print "Could not lookup robot pose: %s" %e
				continue
			robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx') # yields yaw, pitch, roll
			
			# verify distance to goal pose
			dist = math.sqrt((robot_pose_translation[0]-robot_goal_pose.x)*(robot_pose_translation[0]-robot_goal_pose.x) + (robot_pose_translation[1]-robot_goal_pose.y)*(robot_pose_translation[1]-robot_goal_pose.y))
			print "(x,y)-dist: ", dist
			if dist > 0.02:		# in m
				self.tool_wagon_pose = None
			else:
				robot_approximately_well_positioned = True
		
		# 3. unsubscribe to fiducials
		fiducials_sub.unregister()
		
		return 'arrived'


class GraspHandle(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['grasped'])

	def execute(self, userdata ):
		sf = ScreenFormat("GraspHandle")
		rospy.loginfo('Executing state Grasp_Handle')
		
		# 1. move arm in position over wagon
		
		# 2. open hand
		
		# 3. lower arm into handle
		
		# 4. close hand
		
		return 'grasped' 


class GoToNextToolWaggonLocation(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'], input_keys=['tool_wagon_goal_pose'])
		self.navigation_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")

	def execute(self, userdata ):
		sf = ScreenFormat("GoToNextToolWaggonLocation")
		rospy.loginfo('Executing state Go_To_Next_Tool_Waggon_Location')
		
		# 1. adjust base movement speeds
		self.navigation_dynamic_reconfigure_client.update_configuration({"max_vel_y": 0.01, "min_vel_y":-0.01,"max_rot_vel":0.1})
		
		# 2. move robot to new position (robot offset to tool_wagon_goal_pose)
		robot_offset = TOOL_WAGON_ROBOT_OFFSETS["front"]
		dx = -(robot_offset.x*math.cos(robot_offset.theta)+robot_offset.y*math.sin(robot_offset.theta))
		dy = -(-robot_offset.x*math.sin(robot_offset.theta)+robot_offset.y*math.cos(robot_offset.theta))
		robot_pose = Pose2D(x = userdata.tool_wagon_goal_pose.x + dx*math.cos(userdata.tool_wagon_goal_pose.theta)-dy*math.sin(userdata.tool_wagon_goal_pose.theta),
							y = userdata.tool_wagon_goal_pose.y + dx*math.sin(userdata.tool_wagon_goal_pose.theta)+dy*math.cos(userdata.tool_wagon_goal_pose.theta),
							theta = userdata.tool_wagon_goal_pose.theta - robot_offset.theta)
		handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta])
		
		# 3. reset base movement speeds
		self.navigation_dynamic_reconfigure_client.update_configuration({"max_vel_y": 0.2, "min_vel_y":-0.2,"max_rot_vel":0.6})
		
		return 'arrived' 



class ReleaseGrasp(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['released'])

	def execute(self, userdata ):
		sf = ScreenFormat("ReleaseGrasp")
		rospy.loginfo('Executing state Release_Grasp')
		
		# 1. open hand
		
		# 2. lift arm
		
		# 3. hand to home
		
		# 4. arm to folded
		
		return 'released'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Dirt Detection ----------------------------------------------------------------------------------------



class DirtDetectionOn(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['dd_On'])

	def execute(self, userdata ):
		sf = ScreenFormat("DirtDetectionOn")
#		rospy.sleep(2)
		rospy.loginfo('Executing state Dirt_Detection_On')

		# move torso and head to frontal inspection perspective
		sss.move("torso","front_extreme")
		sss.move("head","front")

		rospy.wait_for_service('/dirt_detection/activate_dirt_detection') 
		try:
			req = rospy.ServiceProxy('/dirt_detection/activate_dirt_detection',ActivateDirtDetection)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		return 'dd_On'


# The TrashBinDetectionOn class defines a state machine of smach which basically 
# use the ActivateTrashBinDetection service to activate Trash Bin Detection
class TrashBinDetectionOn(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['TBD_On'])

	def execute(self, userdata ):
		sf = ScreenFormat("TrashBinDetectionOn")
#		rospy.sleep(2)
		rospy.loginfo('Executing state Trash_Bin_Detection_On')
		rospy.wait_for_service('activate_trash_bin_detection_service')
		try:
			req = rospy.ServiceProxy('activate_trash_bin_detection_service', ActivateTrashBinDetection)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		return 'TBD_On'



class DirtDetectionOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['dd_Off'])

	def execute(self, userdata ):
		sf = ScreenFormat("DirtDetectionOff")
#		rospy.sleep(2)
		rospy.loginfo('Executing state Dirt_Detection_Off')

		# move torso back to normal position
		sss.move("torso","home")

		rospy.wait_for_service('/dirt_detection/deactivate_dirt_detection') 
		try:
			req = rospy.ServiceProxy('/dirt_detection/deactivate_dirt_detection',DeactivateDirtDetection)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		return 'dd_Off'


# The TrashBinDetectionOff class defines a state machine of smach which basically 
# use the DeactivateTrashBinDetection service to deactivate Trash Bin Detection
class TrashBinDetectionOff(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['trash_can_found','trash_can_not_found'],output_keys=['detected_waste_bin_poses_'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("TrashBinDetectionOff")
#         rospy.sleep(2)                               
        rospy.loginfo('Executing state Trash_Bin_Detection_Off')     
        rospy.wait_for_service('deactivate_trash_bin_detection_service') 
        try:
            req = rospy.ServiceProxy('deactivate_trash_bin_detection_service',DeactivateTrashBinDetection)
            resp = req()            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
                        
        userdata.detected_waste_bin_poses_ = resp.detected_trash_bin_poses.detections
        
        if len(resp.detected_trash_bin_poses.detections)==0:                                    
            return 'trash_can_not_found'   
        else:
            return 'trash_can_found'
    
    
# The GoToNextUnprocessedWasteBin class defines a state machine of smach which basically 
# give the goal position to go to the next unprocessed trash bin location   
class GoToNextUnprocessedWasteBin(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_trash_location','All_the_trash_bin_is_cleared'],
							input_keys=['go_to_next_unprocessed_waste_bin_in_',
                                        'number_of_unprocessed_trash_bin_in_'],
                            output_keys= ['go_to_next_unprocessed_waste_bin_out_',
                                          'number_of_unprocessed_trash_bin_out_'])
              
    def execute(self, userdata ):
    	sf = ScreenFormat("GoToNextUnprocessedWasteBin")
#         rospy.sleep(2)                               
        rospy.loginfo('Executing state Go_To_Next_Unprocessed_Waste_Bin')
        if (len(userdata.go_to_next_unprocessed_waste_bin_in_)==0 or
            userdata.number_of_unprocessed_trash_bin_in_ == len(userdata.go_to_next_unprocessed_waste_bin_in_)):
            rospy.loginfo('Total Number of Trash Bin: %d',len(userdata.go_to_next_unprocessed_waste_bin_in_))
            return 'All_the_trash_bin_is_cleared'
        else:
            rospy.loginfo('Total Number of Trash Bin: %d',len(userdata.go_to_next_unprocessed_waste_bin_in_))
            rospy.loginfo('Current Trash Bin Number: %d',userdata.number_of_unprocessed_trash_bin_in_)
            userdata.go_to_next_unprocessed_waste_bin_out_ = userdata.go_to_next_unprocessed_waste_bin_in_[userdata.number_of_unprocessed_trash_bin_in_]
            userdata.number_of_unprocessed_trash_bin_out_ = userdata.number_of_unprocessed_trash_bin_in_ + 1
            return 'go_to_trash_location'                    
    
        

#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Clear Waste Bin ---------------------------------------------------------------------------------------


# Here you can use the 'trash_bin_pose_' input key to move the robot 
# to desire trash bin position     
class MoveToTrashBinLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MTTBL_success'],input_keys=['trash_bin_pose_'],
                             output_keys=['center', 'radius', 'rotational_sampling_step', 'goal_pose_theta_offset', 'new_computation_flag', 'invalidate_other_poses_radius', 'goal_pose_selection_strategy'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("MoveToTrashBinLocation")
        rospy.loginfo('Executing state Move_To_Trash_Bin_Location') 
        #try:
            #sm = ApproachPerimeter()
        center = Pose2D()
        center.x = userdata.trash_bin_pose_.pose.pose.position.x 
        center.y = userdata.trash_bin_pose_.pose.pose.position.y
        center.theta = 0
        userdata.center = center
        userdata.radius = 0.75		# adjust this for right distance to trash bin
        userdata.rotational_sampling_step = 10.0/180.0*math.pi
        userdata.goal_pose_theta_offset = math.pi/2.0		# todo: adjust this rotation angle for the right position relative to the trash bin
        userdata.new_computation_flag = True
        userdata.invalidate_other_poses_radius = 1.0 #in meters, radius the current goal covers
        userdata.goal_pose_selection_strategy = 'closest_to_robot'  #'closest_to_target_gaze_direction', 'closest_to_robot'             
            # introspection -> smach_viewer
#             sis = smach_ros.IntrospectionServer('map_accessibility_analysis_introspection', sm, '/MAP_ACCESSIBILITY_ANALYSIS')             
#             sis.start()
#             sm.execute()
#             sis.stop()
#         except:
#             print('EXCEPTION THROWN')
#             print('Aborting cleanly')
#             os._exit(1)                                              
        return 'MTTBL_success'  
        
    
    
class GraspTrashBin(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['GTB_success','failed'])
# 		self.client = actionlib.SimpleActionClient('grasp_trash_bin', autopnp_scenario.msg.GraspTrashBinAction)
# 		result = self.client.wait_for_server()
# 		if result == True:
# 			rospy.loginfo("Grasp trash bin server connected ...")
# 		else:
# 			rospy.logerror("Grasp trash bin server not found ...")

	def execute(self, userdata ):
		sf = ScreenFormat("GraspTrashBin")							
		rospy.loginfo('Executing state Grasp_Trash_Bin')

		# 1. arm: folded -> over trash bin
		handle_arm = sss.move("arm",[[1.2847840785980225, -0.6864653825759888, 2.384225845336914, 1.362023115158081, 0.159407377243042, 0.9801371097564697, -1.39732344150543213]]) # large trash bin
		handle_arm = sss.move("arm",[[0.6609504818916321, -0.46957021951675415, 2.051220178604126, 1.7225379943847656, 1.0994664430618286, 0.6991068720817566, -3.7607481479644775]])
		#handle_arm = sss.move("arm",[[1.2847840785980225, -0.6864653825759888, 2.384225845336914, 1.362023115158081, 0.159407377243042, 0.9801371097564697, -1.39732344150543213]]) # small trash bin
		#handle_arm = sss.move("arm",[[0.10646567493677139, -0.9334499835968018, 3.0960710048675537, 0.7012139558792114, -0.05262168124318123, 0.738262951374054, -2.813400983810425]]) # small trash bin

		# 2. open hand
		sss.move("sdh", [[0.0, 0.0, 0.0, -1.4, 0.0, -1.4, 0.0]])

		# 3.a) arm: over trash bin -> into trash bin
		handle_arm = sss.move("arm",[[0.5458570122718811, -1.0000877380371094, 2.07297420501709, 1.4942553043365479, 1.6078239679336548, 0.7354173064231873, -3.9213883876800537]]) # large trash bin
		#handle_arm = sss.move("arm",[[0.10646567493677139, -1.277030110359192, 3.0960710048675537, 0.5529675483703613, -0.05258183926343918, 0.46139299869537354, -2.8133485317230225]]) # small trash bin

		# 3.b) robot: move left
		handle_base = sss.move_base_rel("base", (0.0, 0.1, 0.0), blocking=True)
		handle_base = sss.move_base_rel("base", (0.0, 0.1, 0.0), blocking=True)
		#rospy.sleep(5)

		# 4. close hand
		handle_sdh = sss.move("sdh",[[0.47,0,0,0.45,-0,0.45,-0]])

		# 5. arm: lift up
		handle_arm = sss.move("arm",[[0.6609504818916321, -0.46957021951675415, 2.051220178604126, 1.7225379943847656, 1.0994664430618286, 0.6991068720817566, -3.7607481479644775]])
		#handle_arm = sss.move("arm",[[0.10628669708967209, -0.21421051025390625, 3.096407413482666, 1.2974236011505127, -0.05254769325256348, 0.7705268859863281, -2.813359022140503]]) # small trash bin
		#handle_arm = sss.move("arm",[[1.162199854850769, -0.21367885172367096, 2.4674811363220215, 1.1584975719451904, -0.3269248604774475, 1.0696042776107788, -2.8133485317230225]]) # small trash bin
		#suggested new carry pos
		handle_arm = sss.move("arm",[[2.1978673934936523, -0.8611428141593933, 2.0929908752441406, 1.7526683807373047, -0.1037636250257492, 0.7569588422775269, -2.8135266304016113]]) # large waste bin
		#handle_arm = sss.move("arm",[[2.5890188217163086, -1.3564121723175049, 2.3780744075775146, 1.9312158823013306, 0.43163323402404785, 0.4533853530883789, -2.814291000366211]]) # small trash bin
		##handle_arm = sss.move("arm",[[1.7155265808105469, -0.5807472467422485, 2.374333143234253, 0.20792463421821594, -0.19672517478466034, 1.9377082586288452, -2.8133485317230225]])
		

# 		goal = autopnp_scenario.msg.GraspTrashBinGoal()
# 		self.client.send_goal(goal)	
# 		finished_before_timeout = self.client.wait_for_result()
# 		if finished_before_timeout:
# 			state = self.client.get_state()
# 			if state is 3:
# 				state = 'SUCCEEDED'
# 				rospy.loginfo("action finished: %s " % state)
# 				return 'GTB_success'
# 			else:
# 				rospy.loginfo("action finished: %s " % state)
# 		else:
# 			rospy.logwarn("action did not finish before the time out.")		

		
		
		
# 		rospy.loginfo('setting robot head and torso position')
# 		handle_head = sss.move("head","back",False)
# 		handle_head.wait()
# 		handle_torso = sss.move("torso","back",False)
# 		handle_torso.wait()        
# 		rospy.wait_for_service('detect_trash_bin_again_service')
# 		try:
# 			while 1:
# 				req = rospy.ServiceProxy('detect_trash_bin_again_service', DetectFiducials)
# 				resp = req('tag_0')
# #                 print'\nseq: ',resp.waste_bin_location.header.seq 
# #                 print'frame id: ',resp.waste_bin_location.header.frame_id
# #                 print'sec: ',resp.waste_bin_location.header.stamp.secs
# #                 print'nsec: ',resp.waste_bin_location.header.stamp.nsecs
# #                 print'position.x: ',resp.waste_bin_location.pose.position.x    
# #                 print'position.y: ',resp.waste_bin_location.pose.position.y  
# #                 print'position.z: ',resp.waste_bin_location.pose.position.z  
# #                 print'orientation.x: ',resp.waste_bin_location.pose.orientation.x  
# #                 print'orientation.y: ',resp.waste_bin_location.pose.orientation.y  
# #                 print'orientation.z: ',resp.waste_bin_location.pose.orientation.z  
# #                 print'orientation.w: ',resp.waste_bin_location.pose.orientation.x   
# 				if resp.waste_bin_location.header.seq != 0:
# 					break
# 		except rospy.ServiceException, e:
# 			print "Service call failed: %s"%e  
#         
# 		handle_torso = sss.move("torso","home",False)
# 		handle_torso.wait()     

		#return 'failed'
		
		return 'GTB_success'



class MoveToToolWagon(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MTTW_success'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("MoveToToolWagon")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_To_Tool_Wagon')
        
        # todo: go to the big trash bin for clearing the smaller one
        #use movetotoolwagonfront
#        sss.move('base',[3.2, 2.8, -1.2])                                               
        return 'MTTW_success'
    
    
    
class ClearTrashBinIntoToolWagon(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['CTBITW_done'])
             
	def execute(self, userdata ):
		sf = ScreenFormat("ClearTrashBinIntoToolWagon")
#         rospy.sleep(2)                              
		rospy.loginfo('Executing state Clear_Trash_Bin_Into_Tool_Wagon')
        
		# 6. arm: move up, turn around
		handle_arm = sss.move("arm",[[2.7794463634490967, -0.5230057239532471, 2.12442684173584, 0.8296095132827759, -0.4476940631866455, 1.7776577472686768, -2.813380002975464]])
		handle_arm = sss.move("arm",[[2.9369261264801025, -0.11826770752668381, -0.3997747302055359, 0.5886469483375549, -0.48628100752830505, 1.5983763933181763, -3.4693655967712402]])
		handle_arm = sss.move("arm",[[3.951626777648926, 0.46264269948005676, -1.3414390087127686, 1.3531497716903687, -1.7254003286361694, 1.7960236072540283, -3.771000385284424]]) # trash bin head over

		#handle_arm = sss.move("arm",[[2.222482204437256, -1.1973689794540405, 2.3105621337890625, 0.8730173110961914, 0.06216597557067871, 1.4420934915542603, -2.8141443729400635]]) # small trash bin
		#handle_arm = sss.move("arm",[[1.7032876014709473, -0.5807051062583923, 2.3603627681732178, 1.1806684732437134, -2.2949676513671875, 1.7723535299301147, -2.81252121925354]]) # small trash bin

		# 7. arm: back to upright trash bin pose
		#handle_arm = sss.move("arm",[[2.5890188217163086, -1.3564121723175049, 2.3780744075775146, 1.9312158823013306, 0.43163323402404785, 0.4533853530883789, -2.814291000366211]]) # small trash bin


		raw_input("Please abort script now.")
		

		return 'CTBITW_done'   
    
    
    
class MoveToTrashBinPickingLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MTTBPL_done'],input_keys=['trash_bin_pose_'])
        smach.State.__init__(self, outcomes=['MTTBPL_done'],input_keys=['trash_bin_pose_'],
                             output_keys=['center', 'radius', 'rotational_sampling_step', 'goal_pose_theta_offset', 'new_computation_flag', 'invalidate_other_poses_radius', 'goal_pose_selection_strategy'])

    def execute(self, userdata ):
    	sf = ScreenFormat("MoveToTrashBinPickingLocation")
        rospy.loginfo('Executing state Move_To_Trash_Bin_Picking_Location') 
        #try:
            #sm = ApproachPerimeter()
#        center = Pose2D()
#        center.x = userdata.trash_bin_pose_.pose.pose.position.x 
#        center.y = userdata.trash_bin_pose_.pose.pose.position.y
#        center.theta = 0
#        userdata.center = center
#        userdata.radius = 0.75		# adjust this for right distance to trash bin
#        userdata.rotational_sampling_step = 10.0/180.0*math.pi
#        userdata.goal_pose_theta_offset = math.pi/2.0		# todo: adjust this rotation angle for the right position relative to the trash bin
#        userdata.new_computation_flag = True
#        userdata.invalidate_other_poses_radius = 1.0 #in meters, radius the current goal covers
#        userdata.goal_pose_selection_strategy = 'closest_to_robot'  #'closest_to_target_gaze_direction', 'closest_to_robot'          
#        rospy.loginfo('Executing state Move_To_Trash_Bin_Picking_Location')
        
        # todo: move back to location where trash bin was grabbed
        #sss.move('base',[userdata.trash_bin_pose_.x, userdata.trash_bin_pose_.y, userdata.trash_bin_pose_.theta])
                         
        return 'MTTBPL_done'
    
    
    
class ReleaseTrashBin(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['RTB_finished'])
             
	def execute(self, userdata ):
		sf = ScreenFormat("ReleaseTrashBin")
#         rospy.sleep(2)                              
		rospy.loginfo('Executing state Release_Trash_Bin')
        
		# 8. arm: put down
		handle_arm = sss.move("arm",[[0.10628669708967209, -0.21421051025390625, 3.096407413482666, 1.2974236011505127, -0.05254769325256348, 0.7705268859863281, -2.813359022140503]])
		handle_arm = sss.move("arm",[[0.10646567493677139, -1.277030110359192, 3.0960710048675537, 0.5529675483703613, -0.05258183926343918, 0.46139299869537354, -2.8133485317230225]])

		# 9. open hand
		sss.move("sdh", "cylopen")

		# 10. arm: trash bin -> over trash bin
		handle_arm = sss.move("arm",[[0.10646567493677139, -0.9334499835968018, 3.0960710048675537, 0.7012139558792114, -0.05262168124318123, 0.738262951374054, -2.813400983810425]])
		handle_arm = sss.move("arm",[[1.2847840785980225, -0.6864653825759888, 2.384225845336914, 1.362023115158081, 0.159407377243042, 0.9801371097564697, -1.39732344150543213]])
	   
		# 11. close hand
		sss.move("sdh", "home")

		# 12. arm: over trash bin -> folded
		handle_arm = sss.move("arm",["folded"])		

		return 'RTB_finished'  
    
 
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
 
 
#--------------------------------------------------------Change Tool Hand --> Vacuum Cleaner---------------------------------------------------------------------

class ChangeToolManual(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['CTM_done'])
		
	def execute(self, userdata):
		sf = ScreenFormat("ChangeToolManual")
		
		# move arm with tool facing up (so it cannot fall down on opening)
		#handle_arm = sss.move("arm",[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
		
		service_name = '/cob_phidgets_toolchanger/ifk_toolchanger/set_digital'
		tool_change_successful = ''
		while tool_change_successful!='yes':
			# wait for confirmation to release tool
			raw_input("Please hold the tool tightly with your hands and then press <Enter> and remove the tool quickly.")
			rospy.wait_for_service(service_name) 
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin2', state=1)
				print 'Opening tool changer response: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
				resp = req(uri='tool_changer_pin2', state=0)
				print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			tool_change_successful = raw_input("If the tool was successfully removed type 'yes' and press <Enter>, otherwise just press enter to repeat.")
		
		# move arm with end facing down
		#handle_arm = sss.move("arm",[[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

		tool_change_successful = ''
		while tool_change_successful!='yes':
			# wait for confirmation to attach tool
			raw_input("Please attach the tool manually and then press <Enter>.")
			rospy.wait_for_service(service_name) 
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin4', state=1)
				print 'Closing tool changer response: uri=', resp.uri, '  state=', resp.state
				rospy.sleep(1.0)
				resp = req(uri='tool_changer_pin4', state=0)
				print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			tool_change_successful = raw_input("If the tool was successfully attached type 'yes' and press <Enter>, otherwise just press enter to repeat.")
		
		print 'Manual tool change successfully completed.'
		
		return 'CTM_done'



class GoToToolWagonLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['GTTWL_done'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("GoToToolWagonLocation")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Go_To_Tool_Wagon_Location')
        return 'GTTWL_done'
    
    
    
class DetectSlotForCurrentTool(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['slot_pose'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("DetectSlotForCurrentTool")
#         rospy.sleep(2)
        rospy.loginfo('Executing state Detect_Slot_For_Current_Tool')
        return 'slot_pose'
    
    
    
class MoveArmToSlot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MATS_done'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("MoveArmToSlot")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Arm_To_Slot')
        return 'MATS_done'
    
    
    
class ReleaseToolChanger(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RTC_done'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("ReleaseToolChanger")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Release_Tool_Changer')
        return 'RTC_done'
    
    
    
class LiftArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['LA_done'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("LiftArm")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Lift-Arm')
        return 'LA_done'
    
    
    
class DetectSlotForNewDevice(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DSFND_slot_pose'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("DetectSlotForNewDevice")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Detect_Slot_For_New_Device')
        return 'DSFND_slot_pose'
    
    
    
class MoveArmToSlot2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MATS2_done'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("MoveArmToSlot2")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Arm_To_Slot_2')
        return 'MATS2_done'
      
    
    
class CloseToolChanger(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CTC_done'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("CloseToolChanger")  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Close_Tool_Changer')
        return 'CTC_done'
    
    
    
class MoveArmToStandardLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MATSL_done'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("MoveArmToStandardLocation")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Arm_To_Standard_Location')
        return 'MATSL_done'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Get Dirt Map ------------------------------------------------------------------------------------------



class GetDirtMap(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['list_of_dirt_location'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("GetDirtMap")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Get_Dirt_Map')   
        rospy.wait_for_service('/dirt_detection/get_dirt_map') 
        try:
            rospy.ServiceProxy('/dirt_detection/get_dirt_map',GetDirtMap)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e                                             
        return 'list_of_dirt_location'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Go To Next Unprocessed dirt Location ------------------------------------------------------------------


class SelectNextUnprocssedDirtSpot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SNUDS_location','no_dirt_spots_left'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("SelectNextUnprocssedDirtSpot")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Select_Next_Unprocssed_Dirt_Spot')                                                
        return 'no_dirt_spots_left'
    
    
    
class Move_Location_Perimeter_60cm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MLP60_arrived_dirt_location','MLP60_unsuccessful'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("Move_Location_Perimeter_60cm")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Location_Perimeter_60cm')                                                
        return 'MLP60_arrived_dirt_location'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Clean -------------------------------------------------------------------------------------------------



class Clean(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['clean_done'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("Clean")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Clean')                                                
        return 'clean_done'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Go To Inspect Location --------------------------------------------------------------------------------



class Move_Location_Perimeter_180cm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MLP180_arrived_dirt_location'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("Move_Location_Perimeter_180cm")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Move_Location_Perimeter_180cm')                                                
        return 'MLP180_arrived_dirt_location'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Verify Cleaning Success -------------------------------------------------------------------------------



class verifyCleaningProcess(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['VCP_done'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("verifyCleaningProcess")
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state verify_Cleaning_Process')     
        rospy.wait_for_service('/dirt_detection/validate_cleaning_result') 
        try:
            rospy.ServiceProxy('/dirt_detection/validate_cleaning_result',ValidateCleaningResult)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e                                              
        return 'VCP_done'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Process Cleaning Verification Results -----------------------------------------------------------------



class ProcessCleaningVerificationResults(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['PCVR_finish'])
             
    def execute(self, userdata ):
    	sf = ScreenFormat("ProcessCleaningVerificationResults")  
#         rospy.sleep(2)                              
        rospy.loginfo('Executing state Process_Cleaning_Verification_Results')                                                
        return 'PCVR_finish'
    
    
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------