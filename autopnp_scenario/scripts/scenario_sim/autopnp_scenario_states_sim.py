#! /usr/bin/env python
#################################################################
# #\file
# # \note
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
# \author: Mohammad Muinul Islam(email:mohammad.islam@ipa.fraunhofer.de),
#          Richard Bormann(email:richard.bormann@ipa.fraunhofer.de)
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

import roslib; roslib.load_manifest('autopnp_scenario')  # ; roslib.load_manifest('cob_navigation_global')
import rospy
import smach
import smach_ros
import tf
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import actionlib

import dynamic_reconfigure.client

from common.autopnp_common import *

from inspect_room_action_client import inspect_room

from autopnp_scenario.srv import *
from autopnp_scenario.msg import *
from ipa_room_segmentation.msg import *
from ipa_building_navigation.msg import *
from autopnp_dirt_detection.srv import *
from cob_object_detection_msgs.msg import DetectionArray, Detection
from geometry_msgs import *
from geometry_msgs.msg import Pose2D, Pose, Transform, Vector3, Quaternion, Point32
from cob_phidgets.srv import SetDigitalSensor
from std_srvs.srv import Trigger, Empty
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Joy
from nav_msgs.msg import OccupancyGrid

from cob_generic_states_experimental.ApproachPerimeter import *
from cob_generic_states_experimental.ApproachPoses import *
from cob_map_accessibility_analysis.srv import CheckPointAccessibility

from simple_script_server import simple_script_server
sss = simple_script_server()


#-------------------------------------------------------- Global Definitions ---------------------------------------------------------------------------------------
global JOURNALIST_MODE
JOURNALIST_MODE = False  # set to true to have breaks within all movements for photography

global CONFIRM_MODE  # safety confirmation necessary during script?
CONFIRM_MODE = 0

global ROBOT_RADIUS
ROBOT_RADIUS = 0.6

global MAX_CLIQUE_PATH_LENGTH
MAX_CLIQUE_PATH_LENGTH = 12.0

global MAP_DOWNSAMPLING_FACTOR
MAP_DOWNSAMPLING_FACTOR = 0.25

global MAX_TOOL_WAGON_DISTANCE_TO_NEXT_ROOM
MAX_TOOL_WAGON_DISTANCE_TO_NEXT_ROOM = 20.0  # maximum allowed distance of tool wagon to next target room center, if exceeded, tool wagon needs to be moved [in m]

global TOOL_WAGON_LOCATIONS
TOOL_WAGON_LOCATIONS = []  # valid parking positions of tool wagon in current map [in m and rad]
#TOOL_WAGON_LOCATIONS.append(Pose2D(x=5.3, y=0, theta=math.pi))
#TOOL_WAGON_LOCATIONS.append(Pose2D(x=0, y=0, theta=0))
TOOL_WAGON_LOCATIONS.append(Pose2D(x=1.513, y=-0.890, theta=0.7))

global TOOL_WAGON_MARKER_OFFSETS
TOOL_WAGON_MARKER_OFFSETS = {  # todo: measure translational offsets
						"front":	Transform(translation=Vector3(x=0.055, y=-0.975, z=0.0), rotation=Quaternion(x=-0.5, y=-0.5, z=-0.5, w=0.5)),  # quaternion_from_euler(-90.0/180.0*math.pi, -90.0/180.0*math.pi, 0.0, 'rzyx')
						"rear":		Transform(translation=Vector3(x=0.06, y=-1.025, z=-0.46), rotation=Quaternion(x=-0.5, y=0.5, z=0.5, w=0.5)),  # quaternion_from_euler(90.0/180.0*math.pi, 90.0/180.0*math.pi, 0.0, 'rzyx')
						"left":		Transform(translation=Vector3(x=-0.17, y=-1.0, z=-0.275), rotation=Quaternion(x=0.0, y=0.707106781, z=0.707106781, w=0.0)),  # quaternion_from_euler(math.pi, 0.0, 90.0/180.0*math.pi, 'rzyx')
						"right":	Transform(translation=Vector3(x=0.29, y=-1.0, z=-0.275), rotation=Quaternion(x=-0.70710678, y=0.0, z=0.0, w=0.70710678))  # quaternion_from_euler(0.0, 0.0, -90.0/180.0*math.pi, 'rzyx')
						}  # offset transformations from respective markers to tool wagon base/center coordinate system

global TOOL_WAGON_ROBOT_OFFSETS
TOOL_WAGON_ROBOT_OFFSETS = {
						"front":	Pose2D(x=-1.1, y=0.0, theta=0.0),
						"front_far":	Pose2D(x=-1.4, y=0.0, theta=0.0),
						"rear":		Pose2D(x=-1.45, y=0.0, theta=math.pi),
						"front_frontal_far":	Pose2D(x=1.4, y=0.0, theta=math.pi),
						"front_trash_clearing":	Pose2D(x=-0.95, y=0.0, theta=0.0)  # Pose2D(x=-1.05, y=0.0, theta=0.0)
						}  # describes the offset of the tool wagon center with respect to base_link (x-axis in tool wagon is directed to the front, y-axis to the left)

global FIDUCIALS_MARKER_DICTIONARY
FIDUCIALS_MARKER_DICTIONARY = {
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

global ARM_JOINT_CONFIGURATIONS_TRASH
ARM_JOINT_CONFIGURATIONS_TRASH = {
		"intermediate1_carry2clear": [1.6047953406237458, -0.9250419568495145, 0.5463578690443048, -1.2438961578963585, 2.005610203344244, -1.6329125948733747, -1.0737614624119514],
		"intermediate2_carry2clear": [1.4538243603262366, -1.0659947472405766, 0.6813939932711062, -0.22266910596943656, 1.4894814369444809, -1.7758899671967503, -1.1778005391233333],
		"intermediate3_carry2clear": [1.7098641615938048, -0.9947504071741682, 0.8471130057479679, 0.7007671479682432, 0.8387877852159548, -1.7758027007341504, 2.283187367581422],  # [1.8488796365151532, -1.091738353707493, 0.8273733319079118, 0.7019365185670795, 0.8387005187533552, -1.7758201540266705, 2.2837284196495404],   #[1.8468026947052798, -1.305960066097277, 0.684221426659337, 0.702058691614719, 0.8387005187533552, -1.7758201540266705, 2.2837284196495404]
		"intermediate4_carry2clear_large": [1.7169851049419416, -1.6298582686823848, 0.3803596038871242, 0.7778408877363129, 0.9197187026309318, -1.754806389832659, 2.2837109663570203],  # [1.8468026947052798, -1.6619548736265604, 0.3801850709619248, 0.702058691614719, 0.8387005187533552, -1.7758201540266705, 2.2837284196495404]
		"intermediate4_carry2clear_small": [1.760915042214639, -1.775104569033353, 0.3801850709619248, 0.7269645400406781, 0.919701249338412, -1.7547889365401388, 2.2837109663570203],
		"intermediate5_carry2clear_large": [1.5208275503102988, -1.85292880037978, -0.10279989294246601, 0.6450562382445842, 0.8517032216807128, -1.793814498614732, 2.3077266968644623],
		"intermediate5_carry2clear_small": [1.5206704706776195, -1.9089887759538378, -0.10300933245270533, 0.4019144201492542, 0.8616166918320407, -1.8318626763082082, 2.3077441501569824],
		"intermediate6_carry2clear_large": [1.4417641351949557, -1.910803918375912, -0.15067427432467048, 0.6390523056177237, 0.8522093671637911, -1.7937795920296922, 2.3077266968644623],  # [1.327776681747206, -1.8629469902862275, -0.15077899407979012, 0.46406559481277226, 0.8517206749732329, -1.7937970453222118, 2.3077266968644623]
		"intermediate6_carry2clear_small": [1.5316311383801438, -1.942795803564968, -0.31066860685499065, 0.4072725809528768, 0.850149878646438, -1.8257365706337085, 2.3077266968644623],
		"intermediate7_carry2clear_large": [1.3697693035501897, -1.9599872966971121, -0.16287412579611082, 0.3929957876715632, 0.5164603789576421, -1.8158056471898605, 2.3077266968644623],  # [1.327811588332246, -1.8629295369937076, -0.16280431262603104, 0.30407126228245207, 0.5157098873792845, -1.8158056471898605, 2.3077266968644623]
		"intermediate7_carry2clear_small": [1.414624265326444, -1.959969843404592, -0.1627868593335111, 0.3280695394973741, 0.42593015065669615, -1.8157881938973404, 2.3077266968644623],
		"clear_large": [1.327811588332246, -1.8629295369937076, -0.16280431262603104, 0.2030865117620602, -0.21528636323350056, -1.8157881938973404, 2.3077266968644623],
		"clear_small": [1.4147813449591236, -1.8628597238236275, -0.16273449945595128, 0.07897614865274341, -0.21584486859413876, -1.8157881938973404, 2.3077441501569824],
		"carry": [2.0313014499336, -0.9694605863127703, 0.2622706266971879, -1.5307061138765867, 2.422900974203568, -1.4726564629552554, -0.407516927048156]
		}

global ARM_JOINT_CONFIGURATIONS_VACUUM
ARM_JOINT_CONFIGURATIONS_VACUUM = {
	"carrying_position": [1.978714679571011, -0.9163502171745829, 0.08915141819187035, -1.796921184683282, 2.4326209849093216, -1.2165643018101275, 1.2519770323330925],  # carrying position
	"intermediate1_position": [1.4535276543533975, -0.3381749958664213, -0.07175048554948689, -1.937908881659384, 2.2285221821811047, -1.234576099690709, 1.000527447],  # intermediate 1
	"intermediate2_position": [0.7885223027585182, -0.14316935854109486, -0.07175048554948689, -1.937908881659384, 2.0185241665811468, -0.9095783396768448, 0.5],  # 7:1.000527447 # intermediate 2
	"above_cleaning_20cm_position": [-0.09950122065619672, -0.19219565722961557, 0.08124507668033604, -2.1109059171170617, 1.7055153453006288, -0.2646093678948603, -0.500527447],  # 7:1.000527447 # ca. 20cm above cleaning position
	"above_cleaning_5cm_position": [-0.09944886077863689, -0.7551690607529065, 0.08124507668033604, -1.562907438575882, 1.7055153453006288, -0.2646093678948603, -0.520801552756769],
		# hmi: [-0.09944886077863689, -0.7551690607529065, 0.08124507668033604, -1.562907438575882, 1.7055153453006288, -0.2646093678948603, -1.3], #7:1.000527447 # just 5cm above cleaning position
	"cleaning_position": [-0.09939650090107707, -0.9171356153379803, 0.08103563717009671, -1.3722302177955017, 1.6754986286220364, -0.20853193902828246, -0.520806248795108]
#[-0.09951867394871666, -0.9302779446054975, 0.08077383778229758, -1.3722127645029818, 1.6754811753295165, -0.20858429890584235, -0.520801552756769]  # hard ground ipa
	     # hmi: [-0.09946631407115684, -0.921219685787647, 0.08096582400001694, -1.341006277477323, 1.705518291756339, -0.2665815899496139, -2.085563736378104]
	     # [-0.09944886077863689, -0.9020385173082293, 0.08121017009529616, -1.401132870208528, 1.705518291756339, -0.2665815899496139, -2.0844467256568278] #[-0.09944886077863689, -0.9020385173082293, 0.08121017009529616, -1.401132870208528, 1.705518291756339, -0.2665815899496139, 1.0595544823007175] #[-0.09944886077863689, -0.9291958404692611, 0.08124507668033604, -1.4179229376127134, 1.7055153453006288, -0.2646093678948603, 1.000527447] # cleaning position #[-0.09943140748611694, -0.8705527776022516, 0.0813497964354557, -1.4487105456178933, 1.6995143591294783, -0.22661355007894374, 0.997525480684839]
	}
# [-0.09944886077863689, -0.9020385173082293, 0.08121017009529616, -1.401132870208528, 1.705518291756339, -0.2665815899496139, 1.0595544823007175] hmi cleaning - soft
# [-0.09944886077863689, -0.9110269629560002, 0.0812276233878161, -1.401132870208528, 1.7055357450488586, -0.2665815899496139, 1.0595544823007175] hmi cleaning -hard

# put down
# sss.move("arm",[ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate1_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate2_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_20cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["cleaning_position"]])
# lift up:
#sss.move("arm",[ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_20cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate2_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate1_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])

class RoomProtocol():
	def __init__(self):
		self.visited = False
		self.room_center = Point32()

#-------------------------------------------------------- Exploration Algorithm ---------------------------------------------------------------------------------------

class InitAutoPnPScenario(smach.State):
	def __init__(self, confirm_mode, tool_wagon_pose=0):
		smach.State.__init__(self,
			outcomes=['initialized', 'failed'],
			input_keys=[],
			output_keys=['tool_wagon_pose', 'current_clique_index', 'current_room_index'])
#		self.local_costmap_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/local_costmap_node/costmap")
		self.dwa_planner_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")
		print "confirm_mode", confirm_mode
		self.CONFIRM_MODE = confirm_mode
		self.tool_wagon_pose = tool_wagon_pose
		
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		CONFIRM_MODE = self.CONFIRM_MODE
		print "CONFIRM_MODE =", CONFIRM_MODE

		# reset torso and head
# 		sss.move("head", "front", False)
# 		sss.move("torso", "home", False)
		sss.move("arm_right", "folded", False)
		sss.move("arm_left", "folded", False)
		
		# todo: check arm position?

		# clear dirt map
# 		clear_dirt_map_service_name = "/dirt_detection/reset_dirt_maps"
# 		rospy.wait_for_service(clear_dirt_map_service_name) 
# 		try:
# 			req = rospy.ServiceProxy(clear_dirt_map_service_name, Empty)
# 			resp = req()
# 		except rospy.ServiceException, e:
# 			print "Service call to /dirt_detection/reset_dirt_maps failed: %s" % e

# 		# just fill history of global transform listener
# 		dummylistener = get_transform_listener()
# 		rospy.sleep(5.0)

		# adjust base footprint -> not necessary at this place
# 		self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.45,0.25],[0.45,-0.25],[0.25,-0.45],[-0.25,-0.45],[-0.45,-0.25],[-0.45,0.25],[-0.25,0.45],[0.25,0.45]]"})
		
		# set acceleration
#		self.dwa_planner_dynamic_reconfigure_client.update_configuration({"acc_lim_x": 1.0, "acc_lim_y": 1.0, "acc_lim_theta": 1.0, "xy_goal_tolerance": 0.05, "yaw_goal_tolerance": 0.06})

		# compute tool wagon pose from robot starting point
		tool_wagon_pose = Pose2D()
		try:
			listener = get_transform_listener()
			t = rospy.Time(0)
			listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
			(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			print "Could not lookup robot pose: %s" % e
			return 'failed'
		tool_wagon_offset = TOOL_WAGON_ROBOT_OFFSETS["front"]  # in meter
		robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx')  # yields yaw, pitch, roll
		tool_wagon_pose.theta = robot_pose_rotation_euler[0] + tool_wagon_offset.theta
		tool_wagon_pose.x = robot_pose_translation[0] + tool_wagon_offset.x * math.cos(robot_pose_rotation_euler[0]) - tool_wagon_offset.y * math.sin(robot_pose_rotation_euler[0])
		tool_wagon_pose.y = robot_pose_translation[1] + tool_wagon_offset.x * math.sin(robot_pose_rotation_euler[0]) + tool_wagon_offset.y * math.cos(robot_pose_rotation_euler[0])
		
		print 'robot_pose: ', (robot_pose_translation[0], robot_pose_translation[1], robot_pose_rotation_euler[0])
		print 'tool_wagon_pose ', tool_wagon_pose
		
		# hack: fixed position if provided by caller
		if self.tool_wagon_pose != 0:
			tool_wagon_pose = self.tool_wagon_pose
		
		userdata.tool_wagon_pose = tool_wagon_pose
	
		# reset indices for room visiting sequence	
		userdata.current_clique_index = 0
		userdata.current_room_index = -1

		return 'initialized'
			
			

# The AnalyzeMap class defines a state machine of smach which basically 
# call the map segmentation action client object and execute the function
# of action client to communicate with action server
class AnalyzeMap(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['list_of_rooms', 'failed'], output_keys=['analyze_map_segmented_map_',
																			'analyze_map_map_resolution_',  # in m/cell
																			'analyze_map_map_origin_',  # in m (pose of cell 0,0 in real world)
																			'analyze_map_room_information_in_pixel_',
																			'analyze_map_room_information_in_meter_',
																			'analyze_map_room_sequence_'])
		#Subscribe to the map topic to get the navigation map
		rospy.Subscriber('/map', OccupancyGrid, self.update_map)
		self.map_ = None
		self.map_resolution_ = None
		self.map_origin_ = None
		self.map_origin_x_ = None
		self.map_origin_y_ = None
		
	#receive data and creates a cv map for the goal of map segmentation action server		  
	def update_map(self, map_msg):
		self.map_resolution_ = map_msg.info.resolution
		self.map_origin_ = map_msg.info.origin
		
		print "self.map_origin_: ", self.map_origin_
		
		#accessible areas are white color and have a value of 255
		self.map_ = 255*np.ones((map_msg.info.height, map_msg.info.width), np.uint8)
		i = 0
		#obstacles are black color and have a value of 0
		for v in range(0,map_msg.info.height):
			for u in range(0,map_msg.info.width):
				if map_msg.data[i] != 0:
					self.map_[v][u] = 0
				i += 1

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		# wait until map received
		while self.map_ == None:
			rospy.sleep(1.)
		mat = self.map_
#		cv.ShowImage( "map_image", mat )
#		cv.WaitKey()   

		# creates an action client object and triggers a room segmentation on the current map
		client_seg = actionlib.SimpleActionClient('/room_segmentation/room_segmentation_server', ipa_room_segmentation.msg.MapSegmentationAction)
		cv_bridge = CvBridge()
		#filling the goal msg format for map segmentation action server		
		goal = ipa_room_segmentation.msg.MapSegmentationGoal(input_map = cv_bridge.cv2_to_imgmsg(mat , "mono8"),
														map_resolution = self.map_resolution_,
														map_origin = self.map_origin_,
														return_format_in_pixel = True,
														return_format_in_meter = True)
		rospy.loginfo("waiting for the map segmentation action server to start.....")
		client_seg.wait_for_server()
		rospy.loginfo("map segmentation action server started, sending goal.....")
		client_seg.send_goal(goal)
		finished_before_timeout = client_seg.wait_for_result(rospy.Duration(200.0))
		if finished_before_timeout:
			state = client_seg.get_state()
			if state is 3:
				state = 'SUCCEEDED'
				rospy.loginfo("action finished: %s " % state)
			else:
				rospy.loginfo("action finished: %s " % state)
		else:
			rospy.loginfo("Action did not finish before the time out.")
			return 'failed'
		map_segmentation_action_server_result = client_seg.get_result()
		
		# copy data to userdata
		userdata.analyze_map_segmented_map_ = map_segmentation_action_server_result.segmented_map
		userdata.analyze_map_map_resolution_ = map_segmentation_action_server_result.map_resolution
		userdata.analyze_map_map_origin_ = map_segmentation_action_server_result.map_origin
		userdata.analyze_map_room_information_in_pixel_ = map_segmentation_action_server_result.room_information_in_pixel
		userdata.analyze_map_room_information_in_meter_ = map_segmentation_action_server_result.room_information_in_meter
		
		# creates an action client object and triggers planning for the room sequence on the segmented map
		try:
			listener = get_transform_listener()
			t = rospy.Time(0)
			listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
			(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
			robot_start_coordinate = Pose()
			robot_start_coordinate.position.x = robot_pose_translation[0]
			robot_start_coordinate.position.y = robot_pose_translation[1]
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			print "Could not lookup robot pose: %s" % e
			return 'failed'
		client_seq = actionlib.SimpleActionClient('/room_sequence_planning/room_sequence_planning_server', ipa_building_navigation.msg.FindRoomSequenceWithCheckpointsAction)
		#filling the goal msg format for map segmentation action server		
		goal = ipa_building_navigation.msg.FindRoomSequenceWithCheckpointsGoal(input_map = cv_bridge.cv2_to_imgmsg(mat , "mono8"),
														map_resolution = self.map_resolution_,
														map_origin = self.map_origin_,
														room_information_in_pixel = map_segmentation_action_server_result.room_information_in_pixel,
														max_clique_path_length = MAX_CLIQUE_PATH_LENGTH,
														map_downsampling_factor = MAP_DOWNSAMPLING_FACTOR,
														robot_radius = ROBOT_RADIUS,
														robot_start_coordinate = robot_start_coordinate)
		rospy.loginfo("waiting for the map segmentation action server to start.....")
		client_seq.wait_for_server()
		rospy.loginfo("map segmentation action server started, sending goal.....")
		client_seq.send_goal(goal)
		finished_before_timeout = client_seq.wait_for_result(rospy.Duration(200.0))
		if finished_before_timeout:
			state = client_seq.get_state()
			if state is 3:
				state = 'SUCCEEDED'
				rospy.loginfo("action finished: %s " % state)
			else:
				rospy.loginfo("action finished: %s " % state)
		else:
			rospy.loginfo("Action did not finish before the time out.")
			return 'failed'
		room_sequence_planning_action_server_result = client_seq.get_result()
		
		# copy data to userdata
		userdata.analyze_map_room_sequence_ = room_sequence_planning_action_server_result.checkpoints
		
		return 'list_of_rooms'

# The NextUnprocessedRoom class defines a state machine of smach which basically 
# call the find next unprocessed room action client object and execute the function
# of action client to communicate with action server
class NextUnprocessedRoom(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['next_room', 'move_trolley', 'no_rooms_left'], input_keys=['room_sequence',
																						'room_information_in_meter',
																						'current_clique_index',
																						'current_room_index'],
																			output_keys=['tool_wagon_goal_pose',
																						'navigation_goal',
																						'current_clique_index',
																						'current_room_index'])  # in meter

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		print "len(userdata.room_sequence): ", len(userdata.room_sequence), "   len(userdata.room_sequence[userdata.current_clique_index].room_indices): ", len(userdata.room_sequence[userdata.current_clique_index].room_indices)
		print "userdata.current_clique_index: ", userdata.current_clique_index, "   userdata.current_room_index: ", userdata.current_room_index

		if userdata.current_room_index >= len(userdata.room_sequence[userdata.current_clique_index].room_indices):
			userdata.current_room_index = -1
			userdata.current_clique_index += 1
			
		if userdata.current_clique_index >= len(userdata.room_sequence):
			return 'no_rooms_left'

		if userdata.current_room_index == -1:
			# move trolley to next group
			userdata.current_room_index = 0
			tool_wagon_goal_pose = Pose2D()
			tool_wagon_goal_pose.x = userdata.room_sequence[userdata.current_clique_index].checkpoint_position_in_meter.x
			tool_wagon_goal_pose.y = userdata.room_sequence[userdata.current_clique_index].checkpoint_position_in_meter.y
			tool_wagon_goal_pose.theta = 0.
			userdata.tool_wagon_goal_pose = tool_wagon_goal_pose
			
			print "tool_wagon_goal_pose: ", tool_wagon_goal_pose
			
			return 'move_trolley'
		
		# set room coordinates
		room_index = userdata.room_sequence[userdata.current_clique_index].room_indices[userdata.current_room_index]
		print "room_index: ", room_index
		navigation_goal = Pose2D()
		navigation_goal.x = userdata.room_information_in_meter[room_index].room_center.x
		navigation_goal.y = userdata.room_information_in_meter[room_index].room_center.y
		navigation_goal.theta = 0.
		userdata.navigation_goal = navigation_goal
		userdata.current_room_index += 1
		
		return 'next_room'


# The ComputeNavigationGoals class defines a state machine of smach which basically 
# call the go to room location action client object and execute the function
# of action client to communicate with action server
class ComputeNavigationGoals(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['successful'], input_keys=['navigation_goal', 'map_resolution'],
															output_keys=['goal_poses',
																		 'goal_pose_application',
																		 'new_computation_flag',
																		 'approach_path_accessibility_check'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		# create a list of potential goal poses, in increasing distance to original goal pose
		goal_poses = [userdata.navigation_goal];
		for r in np.arange(0.2, 2.0, 0.2):		# metric radius
			radius = r / userdata.map_resolution
			for v in np.arange(-r, r, 1):
				for u in np.arange(-r, r, 1):
					goal_poses.append(Pose2D(x=userdata.navigation_goal.x+u, y=userdata.navigation_goal.y+v, theta=0))
		userdata.goal_poses = goal_poses
					
		userdata.goal_pose_application = 'use_as_alternatives'
		userdata.new_computation_flag = True
		userdata.approach_path_accessibility_check = False

		return 'successful'


# The InspectRoom class defines a state machine of smach which basically 
# call the inspect room action client object and execute the function
# of action client to communicate with action server   
class InspectRoom(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished'], input_keys=['inspect_room_data_img_in_',
																	'inspect_room_room_number_',  # vector of already visited rooms and current room at back
																	'analyze_map_room_information_in_pixel',
																	'analyze_map_map_resolution_',
																	'analyze_map_map_origin_'],
														output_keys=['inspect_room_img_out_'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
# 		inspect_room_action_server_result_ = inspect_room(userdata.inspect_room_data_img_in_,
# 														userdata.inspect_room_room_number_,
# 														userdata.analyze_map_room_information_in_pixel[inspect_room_room_number_].room_center.x,
# 														userdata.analyze_map_room_information_in_pixel[inspect_room_room_number_].room_center.y,
# 														userdata.analyze_map_room_information_in_pixel[inspect_room_room_number_].room_min_max[0].x,
# 														userdata.analyze_map_room_information_in_pixel[inspect_room_room_number_].room_min_max[1].x,
# 														userdata.analyze_map_room_information_in_pixel[inspect_room_room_number_].room_min_max[0].y,
# 														userdata.analyze_map_room_information_in_pixel[inspect_room_room_number_].room_min_max[1].y,
# 														userdata.analyze_map_map_resolution_,
# 														userdata.analyze_map_map_origin_)
# 		# rospy.sleep(10)
# 		userdata.inspect_room_img_out_ = inspect_room_action_server_result_.output_img

		return 'finished'


# The InspectRoomShowcase class defines a simplified inspection behavior for showcase purposes    
class InspectRoomShowcase(smach.State):
	def __init__(self, inspection_poses=[], inspection_time=0, search_target=''):
		smach.State.__init__(self, outcomes=['finished'])
		self.inspection_poses = inspection_poses
		if inspection_time <= 0:
			self.inspection_time = 2.0
		else:
			self.inspection_time = inspection_time
		self.search_target = search_target
		
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
# 		if self.search_target != '':
# 			sss.say(["I am looking for ", self.search_target], False)
# 		else:
# 			sss.say(["I am searching through the room."], False)
		
		for pose in self.inspection_poses:
			handle_move = sss.move("base", pose[0], mode=pose[1])
			rospy.sleep(self.inspection_time)
		
		return 'finished'



class InspectRoomManual(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished'], input_keys=['inspect_room_data_img_in_',
																	'inspect_room_room_number_',  # vector of already visited rooms and current room at back
																	'analyze_map_room_information_in_pixel',
																	'analyze_map_map_resolution_',
																	'analyze_map_map_origin_'],
														output_keys=['inspect_room_img_out_'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		userdata.inspect_room_img_out_ = userdata.inspect_room_data_img_in_
		raw_input("finished inspection?")
		
		return 'finished'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Move Tool Wagon ---------------------------------------------------------------------------------------
def computeToolWagonPoseFromFiducials(fiducials):
	trROS = tf.TransformerROS(True, rospy.Duration(10.0))
	averaged_tool_wagon_pose = None
	averaged_tool_wagon_markers = 0.0
	for fiducial in fiducials.detections:
		# transform to base pose of tool wagon and then convert to map coordinate system
		offset = None
		if fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_front_l"] or fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_front_c"] or fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_front_r"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['front']
		elif fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_rear_l"] or fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_rear_c"] or fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_rear_r"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['rear']
		elif fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_left"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['left']
		elif fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_right"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['right']
		
		if offset != None:  # i.e. if a tool wagon marker was detected
			# compute tool wagon center
			offset_mat = trROS.fromTranslationRotation((offset.translation.x, offset.translation.y, offset.translation.z), (offset.rotation.x, offset.rotation.y, offset.rotation.z, offset.rotation.w))
			fiducial_pose_mat = trROS.fromTranslationRotation((fiducial.pose.pose.position.x, fiducial.pose.pose.position.y, fiducial.pose.pose.position.z), (fiducial.pose.pose.orientation.x, fiducial.pose.pose.orientation.y, fiducial.pose.pose.orientation.z, fiducial.pose.pose.orientation.w))
			tool_wagon_pose_mat = numpy.dot(fiducial_pose_mat, offset_mat)
			q = quaternion_from_matrix(tool_wagon_pose_mat)
			tool_wagon_pose = PoseStamped(header=fiducial.pose.header, pose=Pose(position=Point(x=tool_wagon_pose_mat[0, 3], y=tool_wagon_pose_mat[1, 3], z=tool_wagon_pose_mat[2, 3]),
									orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
			# print "fidpose, offset: ", fiducial.pose, offset
			# print "twpose: ", tool_wagon_pose
			# print "twpose ypr: ", euler_from_quaternion([tool_wagon_pose.pose.orientation.x, tool_wagon_pose.pose.orientation.y, tool_wagon_pose.pose.orientation.z, tool_wagon_pose.pose.orientation.w], "rzyx")
			
			# transform to map system
			try:
				listener = get_transform_listener()
				listener.waitForTransform('/map', tool_wagon_pose.header.frame_id, tool_wagon_pose.header.stamp, rospy.Duration(2))
				tool_wagon_pose_map = listener.transformPose('/map', tool_wagon_pose)
				# print 'tool_wagon_pose = ', tool_wagon_pose_map
				# print "tool_wagon_pose ypr: ", euler_from_quaternion([tool_wagon_pose_map.pose.orientation.x, tool_wagon_pose_map.pose.orientation.y, tool_wagon_pose_map.pose.orientation.z, tool_wagon_pose_map.pose.orientation.w], "rzyx")
				# average position when multiple detections are present
				if averaged_tool_wagon_pose == None:
					averaged_tool_wagon_pose = tool_wagon_pose_map
					averaged_tool_wagon_markers = 1.0
				else:
					averaged_tool_wagon_pose.pose.position.x = averaged_tool_wagon_pose.pose.position.x + tool_wagon_pose_map.pose.position.x
					averaged_tool_wagon_pose.pose.position.y = averaged_tool_wagon_pose.pose.position.y + tool_wagon_pose_map.pose.position.y
					averaged_tool_wagon_pose.pose.position.z = averaged_tool_wagon_pose.pose.position.z + tool_wagon_pose_map.pose.position.z
					averaged_tool_wagon_pose.pose.orientation.x = averaged_tool_wagon_pose.pose.orientation.x + tool_wagon_pose_map.pose.orientation.x
					averaged_tool_wagon_pose.pose.orientation.y = averaged_tool_wagon_pose.pose.orientation.y + tool_wagon_pose_map.pose.orientation.y
					averaged_tool_wagon_pose.pose.orientation.z = averaged_tool_wagon_pose.pose.orientation.z + tool_wagon_pose_map.pose.orientation.z
					averaged_tool_wagon_pose.pose.orientation.w = averaged_tool_wagon_pose.pose.orientation.w + tool_wagon_pose_map.pose.orientation.w
					averaged_tool_wagon_markers = averaged_tool_wagon_markers + 1.0
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
				print "computeToolWagonPoseFromFiducials: Could not lookup robot pose: %s" % e
				return averaged_tool_wagon_pose
	# finalize averaging and normalizationroslib.load_manifest(PACKAGE)
	if averaged_tool_wagon_markers > 0.0:
		averaged_tool_wagon_pose.pose.position.x = averaged_tool_wagon_pose.pose.position.x / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.position.y = averaged_tool_wagon_pose.pose.position.y / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.position.z = averaged_tool_wagon_pose.pose.position.z / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.x = averaged_tool_wagon_pose.pose.orientation.x / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.y = averaged_tool_wagon_pose.pose.orientation.y / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.z = averaged_tool_wagon_pose.pose.orientation.z / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.w = averaged_tool_wagon_pose.pose.orientation.w / averaged_tool_wagon_markers
		norm = math.sqrt(averaged_tool_wagon_pose.pose.orientation.x * averaged_tool_wagon_pose.pose.orientation.x + averaged_tool_wagon_pose.pose.orientation.y * averaged_tool_wagon_pose.pose.orientation.y + averaged_tool_wagon_pose.pose.orientation.z * averaged_tool_wagon_pose.pose.orientation.z + averaged_tool_wagon_pose.pose.orientation.w * averaged_tool_wagon_pose.pose.orientation.w)
		averaged_tool_wagon_pose.pose.orientation.x = averaged_tool_wagon_pose.pose.orientation.x / norm
		averaged_tool_wagon_pose.pose.orientation.y = averaged_tool_wagon_pose.pose.orientation.y / norm
		averaged_tool_wagon_pose.pose.orientation.z = averaged_tool_wagon_pose.pose.orientation.z / norm
		averaged_tool_wagon_pose.pose.orientation.w = averaged_tool_wagon_pose.pose.orientation.w / norm
		
		return averaged_tool_wagon_pose

def currentRobotPose():
	# read out current robot pose
	try:
		listener = get_transform_listener()
		t = rospy.Time(0)
		listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
		(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
		print "Could not lookup robot pose: %s" % e
		return (None, None, None)
	robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx')  # yields yaw, pitch, roll
	
	return (robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler)

def positionControlLoopLinear(self_tool_wagon_pose, dx, dy, dtheta):
	# move to corrected pose
	tool_wagon_pose_3d = self_tool_wagon_pose.pose
	tool_wagon_pose_3d_euler = tf.transformations.euler_from_quaternion([tool_wagon_pose_3d.orientation.x, tool_wagon_pose_3d.orientation.y, tool_wagon_pose_3d.orientation.z, tool_wagon_pose_3d.orientation.w], 'rzyx')  # yields yaw, pitch, roll
	print "wagon theta: ", tool_wagon_pose_3d_euler[0], dtheta
	tool_wagon_pose = Pose2D(x=tool_wagon_pose_3d.position.x, y=tool_wagon_pose_3d.position.y, theta=tool_wagon_pose_3d_euler[0])
	robot_goal_pose = Pose2D(x=tool_wagon_pose.x + dx * math.cos(tool_wagon_pose.theta) - dy * math.sin(tool_wagon_pose.theta),
							y=tool_wagon_pose.y + dx * math.sin(tool_wagon_pose.theta) + dy * math.cos(tool_wagon_pose.theta),
							theta=tool_wagon_pose.theta - dtheta)
	print "moving to ", [float(robot_goal_pose.x), float(robot_goal_pose.y), float(robot_goal_pose.theta)]
	handle_base = sss.move("base", [float(robot_goal_pose.x), float(robot_goal_pose.y), float(robot_goal_pose.theta)], mode='linear')
	
	# read out current robot pose
	(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = currentRobotPose()
	if (robot_pose_translation == None):
		return False
# 	try:
# 		listener = get_transform_listener()
# 		t = rospy.Time(0)
# 		listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
# 		(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
# 	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
# 		print "Could not lookup robot pose: %s" %e
# 		continue
# 	robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx') # yields yaw, pitch, roll
	
	
	# verify distance to goal pose
	dist = math.sqrt((robot_pose_translation[0] - robot_goal_pose.x) * (robot_pose_translation[0] - robot_goal_pose.x) + (robot_pose_translation[1] - robot_goal_pose.y) * (robot_pose_translation[1] - robot_goal_pose.y))
	delta_theta = robot_pose_rotation_euler[0] - robot_goal_pose.theta
	print "delta_theta first:", delta_theta
	while delta_theta > 1.5 * math.pi:
		delta_theta = delta_theta - 2 * math.pi
	while delta_theta < -1.5 * math.pi:
		delta_theta = delta_theta + 2 * math.pi
	print "(x,y)-dist: ", dist, "  yaw-dist: ", delta_theta
	if dist > 0.04 or abs(delta_theta) > 0.03:  # in m      # rot: 0.02 #trans: 0.03  # josh
		return False
	else:
		return True

class MoveToToolWaggonFront(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'], input_keys=['tool_wagon_pose'])
		self.tool_wagon_pose = None
		self.last_callback_time = rospy.Time.now()

	def fiducial_callback(self, fiducials):
		if (rospy.Time.now() - self.last_callback_time) > rospy.Duration(1.0):
			self.tool_wagon_pose = computeToolWagonPoseFromFiducials(fiducials)
			self.last_callback_time = rospy.Time.now()

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		#hack: turned off for simple simulation
#		sss.move("head", "back", False)
#		sss.move("torso", "back", False)

#		self.tool_wagon_pose = None
#		fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)
		
		print "tool_wagon_pose: ", userdata.tool_wagon_pose
		
		# 1. move to last known position (i.e. move_base to tool_wagon_pose + robot offset)
		robot_offset = Pose2D(x=TOOL_WAGON_ROBOT_OFFSETS["front"].x - 0.2, y=TOOL_WAGON_ROBOT_OFFSETS["front"].y, theta=TOOL_WAGON_ROBOT_OFFSETS["front"].theta)
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		robot_pose = Pose2D(x=userdata.tool_wagon_pose.x + dx * math.cos(userdata.tool_wagon_pose.theta) - dy * math.sin(userdata.tool_wagon_pose.theta),
							y=userdata.tool_wagon_pose.y + dx * math.sin(userdata.tool_wagon_pose.theta) + dy * math.cos(userdata.tool_wagon_pose.theta),
							theta=userdata.tool_wagon_pose.theta - robot_offset.theta)
		handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta], mode='omni')  # hack

		#hack: turned off for simple simulation
# 		# 2. detect fiducials and move to corrected pose
# 		robot_offset = TOOL_WAGON_ROBOT_OFFSETS["front"]
# 		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
# 		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
# 		# wait until wagon is detected
# 		robot_approximately_well_positioned = False
# 		while robot_approximately_well_positioned == False:
# 			while self.tool_wagon_pose == None:
# 				rospy.sleep(0.2)
# 			robot_approximately_well_positioned = positionControlLoopLinear(self.tool_wagon_pose, dx, dy, robot_offset.theta)
# 			if robot_approximately_well_positioned == False:
# 				self.tool_wagon_pose = None
# 		
# 		# 3. unsubscribe to fiducials
# 		fiducials_sub.unregister()
# 		sss.move("torso", "home")
		
		return 'arrived'

class MoveToToolWaggonFrontFar(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'], input_keys=['tool_wagon_pose'])
		self.tool_wagon_pose = None
		self.last_callback_time = rospy.Time.now()

	def fiducial_callback(self, fiducials):
		if (rospy.Time.now() - self.last_callback_time) > rospy.Duration(1.0):
			self.tool_wagon_pose = computeToolWagonPoseFromFiducials(fiducials)
			self.last_callback_time = rospy.Time.now()

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		sss.move("head", "back", False)
		sss.move("torso", "back", False)

		self.tool_wagon_pose = None
		fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)
		
		# 1. move to last known position (i.e. move_base to tool_wagon_pose + robot offset)
		robot_offset = Pose2D(x=TOOL_WAGON_ROBOT_OFFSETS["front_far"].x, y=TOOL_WAGON_ROBOT_OFFSETS["front_far"].y, theta=TOOL_WAGON_ROBOT_OFFSETS["front_far"].theta)
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		robot_pose = Pose2D(x=userdata.tool_wagon_pose.x + dx * math.cos(userdata.tool_wagon_pose.theta) - dy * math.sin(userdata.tool_wagon_pose.theta),
							y=userdata.tool_wagon_pose.y + dx * math.sin(userdata.tool_wagon_pose.theta) + dy * math.cos(userdata.tool_wagon_pose.theta),
							theta=userdata.tool_wagon_pose.theta - robot_offset.theta)
		handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta], mode='omni')
		
		# 2. detect fiducials and move to corrected pose
		robot_offset = TOOL_WAGON_ROBOT_OFFSETS["front_far"]
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		# wait until wagon is detected
		robot_approximately_well_positioned = False
		while robot_approximately_well_positioned == False:
			while self.tool_wagon_pose == None:
				rospy.sleep(0.2)
			robot_approximately_well_positioned = positionControlLoopLinear(self.tool_wagon_pose, dx, dy, robot_offset.theta)
			if robot_approximately_well_positioned == False:
				self.tool_wagon_pose = None
		
		# 3. unsubscribe to fiducials
		fiducials_sub.unregister()
		sss.move("torso", "home")

		return 'arrived'


class MoveToToolWaggonRear(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'], input_keys=['tool_wagon_pose'])
		self.tool_wagon_pose = None

	def fiducial_callback(self, fiducials):
		
		self.tool_wagon_pose = computeToolWagonPoseFromFiducials(fiducials)

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		sss.move("head", "back", False)
		sss.move("torso", "back", False)

		self.tool_wagon_pose = None
		fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)
		
		# 1. move to last known position (i.e. move_base to tool_wagon_pose + robot offset)
		robot_offset = Pose2D(x=TOOL_WAGON_ROBOT_OFFSETS["rear"].x - 0.2, y=TOOL_WAGON_ROBOT_OFFSETS["rear"].y, theta=TOOL_WAGON_ROBOT_OFFSETS["rear"].theta)
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		robot_pose = Pose2D(x=userdata.tool_wagon_pose.x + dx * math.cos(userdata.tool_wagon_pose.theta) - dy * math.sin(userdata.tool_wagon_pose.theta),
							y=userdata.tool_wagon_pose.y + dx * math.sin(userdata.tool_wagon_pose.theta) + dy * math.cos(userdata.tool_wagon_pose.theta),
							theta=userdata.tool_wagon_pose.theta - robot_offset.theta)
		handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta], mode='omni')  # hack
		
		# 2. detect fiducials and move to corrected pose
		robot_offset = TOOL_WAGON_ROBOT_OFFSETS["rear"]
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		# wait until wagon is detected
		robot_approximately_well_positioned = False
		while robot_approximately_well_positioned == False:
			while self.tool_wagon_pose == None:
				rospy.sleep(0.2)
			robot_approximately_well_positioned = positionControlLoopLinear(self.tool_wagon_pose, dx, dy, robot_offset.theta)
			if robot_approximately_well_positioned == False:
				self.tool_wagon_pose = None
		
		# 3. unsubscribe to fiducials
		fiducials_sub.unregister()
		sss.move("torso", "home")

		return 'arrived'

class MoveToToolWaggonFrontFrontalFar(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'], input_keys=['tool_wagon_pose'])
		self.tool_wagon_pose = None
		self.last_callback_time = rospy.Time.now()

	def fiducial_callback(self, fiducials):
		if (rospy.Time.now() - self.last_callback_time) > rospy.Duration(1.0):
			self.tool_wagon_pose = computeToolWagonPoseFromFiducials(fiducials)
			self.last_callback_time = rospy.Time.now()

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		sss.move("head", "front", False)
		sss.move("torso", "front", False)

		self.tool_wagon_pose = None
		fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)
		
		# 1. move to last known position (i.e. move_base to tool_wagon_pose + robot offset)
		robot_offset = Pose2D(x=TOOL_WAGON_ROBOT_OFFSETS["front_frontal_far"].x + 0.0, y=TOOL_WAGON_ROBOT_OFFSETS["front_frontal_far"].y, theta=TOOL_WAGON_ROBOT_OFFSETS["front_frontal_far"].theta)
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		robot_pose = Pose2D(x=userdata.tool_wagon_pose.x + dx * math.cos(userdata.tool_wagon_pose.theta) - dy * math.sin(userdata.tool_wagon_pose.theta),
							y=userdata.tool_wagon_pose.y + dx * math.sin(userdata.tool_wagon_pose.theta) + dy * math.cos(userdata.tool_wagon_pose.theta),
							theta=userdata.tool_wagon_pose.theta - robot_offset.theta)

		d=True
		while d:
			handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta], mode='linear')
			if handle_base.get_error_code() == 0: d=False
		
		# 2. detect fiducials and move to corrected pose
		# wait until wagon is detected
		robot_approximately_well_positioned = False
		while robot_approximately_well_positioned == False:
			while self.tool_wagon_pose == None:
				rospy.sleep(0.2)
			print "tool_wagon_pose", self.tool_wagon_pose
			robot_approximately_well_positioned = positionControlLoopLinear(self.tool_wagon_pose, dx, dy, robot_offset.theta)
			if robot_approximately_well_positioned == False:
				self.tool_wagon_pose = None
		
		# 3. unsubscribe to fiducials
		fiducials_sub.unregister()
		sss.move("torso", "home")
		
		return 'arrived'

class MoveToToolWaggonFrontTrashClearing(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'])  # , input_keys=['tool_wagon_pose'])
#		self.local_costmap_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/local_costmap_node/costmap")
		# self.move_base_local_costmap_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/move_base/local_costmap")
		self.tool_wagon_pose = None
		self.last_callback_time = rospy.Time.now()

	def fiducial_callback(self, fiducials):
		if (rospy.Time.now() - self.last_callback_time) > rospy.Duration(1.0):
			self.tool_wagon_pose = computeToolWagonPoseFromFiducials(fiducials)
			self.last_callback_time = rospy.Time.now()

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		sss.move("head", "back", False)
		sss.move("torso", "back", False)
		
		sss.say(["I will now move closer to the waste container."], False)

		self.tool_wagon_pose = None
		fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)
		
		# 1. adjust base footprint
#		local_config = self.local_costmap_dynamic_reconfigure_client.get_configuration(5.0)
	# 	move_base_local_config = self.move_base_local_costmap_dynamic_reconfigure_client.get_configuration(5.0)
#		self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.3,0.3],[0.3,-0.3],[-0.3,-0.3],[-0.3,0.3]]"})  # [[0.25,-0.25],[-0.25,-0.25],[-0.25,0.25]]#[[0.3,0.3],[0.3,-0.3],[-0.3,-0.3],[-0.3,0.3]]#[[0.1,0.1],[0.1,-0.1],[-0.1,-0.1],[-0.1,0.1]]
	# 	self.move_base_local_costmap_dynamic_reconfigure_client.update_configuration({"inflation_radius": "0.3"})
		# self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.45,0.36],[-0.20,0.16],[-0.20,-0.16],[0.45,-0.36]]"})
		# [[0.56,0.36],[-0.56,0.36],[-0.56,-0.36],[0.56,-0.36]]
		rospy.sleep(0.5)
		rospy.wait_for_service('/update_footprint') 
		try:
			req = rospy.ServiceProxy('/update_footprint', Empty)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to /update_footprint failed: %s" % e
		
# 		# 1. move to last known position (i.e. move_base to tool_wagon_pose + robot offset)
# 		robot_offset = Pose2D(x=TOOL_WAGON_ROBOT_OFFSETS["front_trash_clearing"].x-0.35, y=TOOL_WAGON_ROBOT_OFFSETS["front_trash_clearing"].y, theta=TOOL_WAGON_ROBOT_OFFSETS["front_trash_clearing"].theta)
# 		dx = -(robot_offset.x*math.cos(robot_offset.theta)+robot_offset.y*math.sin(robot_offset.theta))
# 		dy = -(-robot_offset.x*math.sin(robot_offset.theta)+robot_offset.y*math.cos(robot_offset.theta))
# 		robot_pose = Pose2D(x = userdata.tool_wagon_pose.x + dx*math.cos(userdata.tool_wagon_pose.theta)-dy*math.sin(userdata.tool_wagon_pose.theta),
# 							y = userdata.tool_wagon_pose.y + dx*math.sin(userdata.tool_wagon_pose.theta)+dy*math.cos(userdata.tool_wagon_pose.theta),
# 							theta = userdata.tool_wagon_pose.theta - robot_offset.theta)
# 		handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta])
		
		# 2. detect fiducials and move to corrected pose
		robot_offset = TOOL_WAGON_ROBOT_OFFSETS["front_trash_clearing"]
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		# wait until wagon is detected
		robot_approximately_well_positioned = False
		while robot_approximately_well_positioned == False:
			while self.tool_wagon_pose == None:
				rospy.sleep(0.2)
			print "tool_wagon_pose", self.tool_wagon_pose
			robot_approximately_well_positioned = positionControlLoopLinear(self.tool_wagon_pose, dx, dy, robot_offset.theta)
			if robot_approximately_well_positioned == False:
				self.tool_wagon_pose = None
		
		# 3. unsubscribe to fiducials
		fiducials_sub.unregister()
		
		# 4. reset footprint
#		if local_config["footprint"] != None:
#			self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": local_config["footprint"]})
#		else:
#			rospy.logwarn("Could not read previous local footprint configuration of /local_costmap_node/costmap, resetting to standard value: [[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]].")
#			self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]]"})
	# 	if move_base_local_config["inflation_radius"]!=None:
	# 		self.move_base_local_costmap_dynamic_reconfigure_client.update_configuration({"inflation_radius": move_base_local_config["inflation_radius"]})
	# 	else:
	# 		rospy.logwarn("Could not read previous local inflation radius configuration of /move_base/local_costmap, resetting to standard value: 0.55.")
	# 		self.move_base_local_costmap_dynamic_reconfigure_client.update_configuration({"inflation_radius": "0.55"})

		rospy.sleep(0.5)
		rospy.wait_for_service('/update_footprint') 
		try:
			req = rospy.ServiceProxy('/update_footprint', Empty)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to /update_footprint failed: %s" % e

		sss.move("torso", "home")

		return 'arrived'



class GraspHandle(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['grasped'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		#hack: not needed in simulation
		return 'grasped'
		
		# #intermediate_folded2overhandle_position = [2.1915052219741598, -1.0823484823317635, -0.6911678370822745, -1.671606544390089, 3.0001860775932125, -1.3194340079226734, -0.00013962634015954637]
		intermediate1_folded2overhandle_position = [2.4124988118616817, -0.8013330194681565, -0.6911678370822745, -1.6716239976826088, 3.0001860775932125, -1.3194340079226734, -0.00013962634015954637]
		intermediate2_folded2overhandle_position = [2.412481358569162, -0.8013330194681565, -0.6911678370822745, -1.3406223050418844, 2.81319150153454, -1.5754563558977213, -1.4741399928194505]
		overhandle_position = [2.0912709630321253, -1.0194293627973678, -0.6451958645847439, -0.9112713090512793, 2.8221799471823106, -1.6184612686668618, -1.2679642482813605]  # [2.1104870380965832, -0.9793216965865382, -0.6941872566882247, -0.968622828271813, 2.8201902718350373, -1.6184438153743417, -1.288140254434415]
		
		athandle_position = [1.877206330275021, -1.195515631031076, -0.6841167069042173, -0.8776039077803086, 2.978247288895644, -1.5683877724271442, -1.1731230567279887]  # [1.8774855829553403, -1.153348476302893, -0.6841865200742971, -0.928602428523583, 2.9091846103942283, -1.5784583222111517, -1.263129686253336]
		# deepathandle_position = [1.87750303624786, -1.153348476302893, -0.6841865200742971, -0.9696351192379697, 2.9091846103942283, -1.481435469092787, -1.263129686253336]
		
		# 1. move arm in position over wagon
		if JOURNALIST_MODE == False:
			handle_arm = sss.move("arm", [intermediate1_folded2overhandle_position, intermediate2_folded2overhandle_position, overhandle_position])
		else:
			raw_input("Press <Enter>.")
			handle_arm = sss.move("arm", [intermediate1_folded2overhandle_position])
			raw_input("Press <Enter>.")
			handle_arm = sss.move("arm", [intermediate2_folded2overhandle_position])
			raw_input("Press <Enter>.")
			handle_arm = sss.move("arm", [overhandle_position])
			raw_input("Press <Enter>.")
		
		print "handle.get_state()=", handle_arm.get_state()  # 3 = ok
		
		# 2. open hand
		sss.move("sdh", "cylopen")
		if JOURNALIST_MODE == True:
			raw_input("Press <Enter>.")
		
		# 3. lower arm into handle
		handle_arm = sss.move("arm", [athandle_position])
		print "handle.get_state()=", handle_arm.get_state()
		if JOURNALIST_MODE == True:
			raw_input("Press <Enter>.")
		
		# 4. close hand
		sss.move("sdh", "cylclosed")
		if JOURNALIST_MODE == True:
			raw_input("Press <Enter>.")
		
		return 'grasped'


class GoToNextToolWaggonLocation(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'], input_keys=['tool_wagon_goal_pose'], output_keys=['tool_wagon_pose'])
		self.navigation_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/move_base/DWAPlannerROS")

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		# hack: turned off for simulation
		# 1. adjust base movement speeds
#		self.navigation_dynamic_reconfigure_client.update_configuration({"max_vel_y": 0.01, "min_vel_y":-0.01, "max_rot_vel":0.1})
		
		print "tool_wagon_goal_pose: ", userdata.tool_wagon_goal_pose
		
		# 2. move robot to new position (robot offset to tool_wagon_goal_pose)
		robot_offset = TOOL_WAGON_ROBOT_OFFSETS["front"]
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		robot_pose = Pose2D(x=userdata.tool_wagon_goal_pose.x + dx * math.cos(userdata.tool_wagon_goal_pose.theta) - dy * math.sin(userdata.tool_wagon_goal_pose.theta),
							y=userdata.tool_wagon_goal_pose.y + dx * math.sin(userdata.tool_wagon_goal_pose.theta) + dy * math.cos(userdata.tool_wagon_goal_pose.theta),
							theta=userdata.tool_wagon_goal_pose.theta - robot_offset.theta)
		handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta], mode='omni')  # hack
		
		# 3. reset base movement speeds
#		self.navigation_dynamic_reconfigure_client.update_configuration({"max_vel_y": 0.2, "min_vel_y":-0.2, "max_rot_vel":0.6})
		userdata.tool_wagon_pose = userdata.tool_wagon_goal_pose
		
		return 'arrived' 



class ReleaseGrasp(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['released'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		# hack: not implemented in sim
		return 'released'
		
		# #intermediate_folded2overhandle_position = [2.1915052219741598, -1.0823484823317635, -0.6911678370822745, -1.671606544390089, 3.0001860775932125, -1.3194340079226734, -0.00013962634015954637]
		intermediate1_folded2overhandle_position = [2.4124988118616817, -0.8013330194681565, -0.6911678370822745, -1.6716239976826088, 3.0001860775932125, -1.3194340079226734, -0.00013962634015954637]
		intermediate2_folded2overhandle_position = [2.412481358569162, -0.8013330194681565, -0.6911678370822745, -1.3406223050418844, 2.81319150153454, -1.5754563558977213, -1.4741399928194505]
		overhandle_position = [2.0912709630321253, -1.0194293627973678, -0.6451958645847439, -0.9112713090512793, 2.8221799471823106, -1.6184612686668618, -1.2679642482813605]  # [2.1104870380965832, -0.9793216965865382, -0.6941872566882247, -0.968622828271813, 2.8201902718350373, -1.6184438153743417, -1.288140254434415]
		
		if (CONFIRM_MODE == True):
			raw_input("Press <Enter>.")
		
		# 1. open hand
		sss.move("sdh", "cylopen")
		
		# 2. lift arm
		handle_arm = sss.move("arm", [overhandle_position])
		print "handle.get_state()=", handle_arm.get_state()
		
		# 3. hand to home
		sss.move("sdh", "home")
		
		# 4. arm to folded
		handle_arm = sss.move("arm", [intermediate2_folded2overhandle_position, intermediate1_folded2overhandle_position, "folded"])
		print "handle.get_state()=", handle_arm.get_state()
		handle_arm = sss.move("arm", ["folded"])
		print "handle.get_state()=", handle_arm.get_state()
		
		return 'released'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Dirt Detection ----------------------------------------------------------------------------------------



class DirtDetectionOn(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['dirt_detection_on'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		# move torso and head to frontal inspection perspective
#		sss.say(["By bowing my head I can look at the ground to search for dirt spots."], False)
#		sss.move("torso", "front_extreme", False)
#		sss.move("head", "front")

# 		# hack: reset dirt map from previous runs 
# 		rospy.wait_for_service('/dirt_detection/reset_dirt_maps') 
# 		try:
# 			req = rospy.ServiceProxy('/dirt_detection/reset_dirt_maps', Empty)
# 			resp = req()
# 		except rospy.ServiceException, e:
# 			print "Service call failed: %s" % e

		#hack: not in simulation
# 		rospy.wait_for_service('/dirt_detection/activate_dirt_detection') 
# 		try:
# 			req = rospy.ServiceProxy('/dirt_detection/activate_dirt_detection', ActivateDirtDetection)
# 			resp = req()
# 		except rospy.ServiceException, e:
# 			print "Service call failed: %s" % e

		return 'dirt_detection_on'


# The TrashBinDetectionOn class defines a state machine of smach which basically 
# use the ActivateTrashBinDetection service to activate Trash Bin Detection
class TrashBinDetectionOn(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['trash_bin_detection_on'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		# move torso and head to frontal inspection perspective
#		sss.say(["By bowing my head I can search for trash bins with my cameras."], False)
#		sss.move("torso", "front_extreme", False)
#		sss.move("head", "front", False)
		
		#hack: not in simulation
# 		try:
# 			rospy.wait_for_service('/activate_trash_bin_detection_service')
# 			req = rospy.ServiceProxy('/activate_trash_bin_detection_service', ActivateTrashBinDetection)
# 			resp = req()
# 		except rospy.ServiceException, e:
# 			print "Service call failed: %s" % e
# 		except ee:
# 			print "Service call failed: %s" % ee
		
		return 'trash_bin_detection_on'



class DirtDetectionOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['dirt_detection_off'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		# move torso back to normal position
#		sss.move("torso", "home", False)

		# hack: not in simulation
# 		rospy.wait_for_service('/dirt_detection/deactivate_dirt_detection') 
# 		try:
# 			req = rospy.ServiceProxy('/dirt_detection/deactivate_dirt_detection', DeactivateDirtDetection)
# 			resp = req()
# 		except rospy.ServiceException, e:
# 			print "Service call failed: %s" % e

		return 'dirt_detection_off'


# The TrashBinDetectionOff class defines a state machine of smach which basically 
# use the DeactivateTrashBinDetection service to deactivate Trash Bin Detection
class TrashBinDetectionOff(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['trash_can_found', 'trash_can_not_found'], output_keys=['trash_detection_poses', 'sm_trash_bin_counter'])
	
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		# move torso back to normal position
#		sss.move("torso", "home", False)
		
		# hack: not in simulation
# 		try:
# 			rospy.wait_for_service('/deactivate_trash_bin_detection_service')
# 			req = rospy.ServiceProxy('/deactivate_trash_bin_detection_service', DeactivateTrashBinDetection)
# 			resp = req()
# 		except rospy.ServiceException, e:
# 			print "Service call failed: %s" % e

# 		userdata.detected_waste_bin_poses_ = resp.detected_trash_bin_poses.detections
# 		
# 		print "userdata.detected_waste_bin_poses: ", resp.detected_trash_bin_poses.detections
# 		
#  		# kind of hack: reset processed trash bin counter (implies that respective trash bins are always directly cleared after each search)
# 		userdata.sm_trash_bin_counter = 0
# 
# 		if len(resp.detected_trash_bin_poses.detections) == 0:
# 			return 'trash_can_not_found'
# 		else:
# 			return 'trash_can_found'

		return 'trash_can_found'



# The GoToNextUnprocessedWasteBin class defines a state machine of smach which basically 
# give the goal position to go to the next unprocessed trash bin location   
class GoToNextUnprocessedWasteBin(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['go_to_trash_location', 'all_trash_bins_cleared'],
							input_keys=['go_to_next_unprocessed_waste_bin_in_',
										'number_of_unprocessed_trash_bin_in_'],
							output_keys=['go_to_next_unprocessed_waste_bin_out_',
										'number_of_unprocessed_trash_bin_out_'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		if (len(userdata.go_to_next_unprocessed_waste_bin_in_) == 0 or
			userdata.number_of_unprocessed_trash_bin_in_ == len(userdata.go_to_next_unprocessed_waste_bin_in_)):
			rospy.loginfo('Total Number of Trash Bin: %d', len(userdata.go_to_next_unprocessed_waste_bin_in_))
			sss.say(["All found trash bins have been cleared."], False)
			return 'all_trash_bins_cleared'
		else:
			rospy.loginfo('Total Number of Trash Bin: %d', len(userdata.go_to_next_unprocessed_waste_bin_in_))
			rospy.loginfo('Current Trash Bin Number: %d', userdata.number_of_unprocessed_trash_bin_in_ + 1)
			userdata.go_to_next_unprocessed_waste_bin_out_ = userdata.go_to_next_unprocessed_waste_bin_in_[userdata.number_of_unprocessed_trash_bin_in_]
			print "Pose of next trash bin to clear: ", userdata.go_to_next_unprocessed_waste_bin_in_[userdata.number_of_unprocessed_trash_bin_in_]
			userdata.number_of_unprocessed_trash_bin_out_ = userdata.number_of_unprocessed_trash_bin_in_ + 1
			sss.say(["I am going to clear the next trash bin."], False)
			return 'go_to_trash_location'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Clear Waste Bin ---------------------------------------------------------------------------------------


# Here you can use the 'trash_bin_pose_' input key to move the robot 
# to desire trash bin position
class MoveToTrashBinLocation(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['MTTBL_success'], input_keys=['trash_bin_pose_'],
							output_keys=['center', 'radius', 'rotational_sampling_step', 'goal_pose_theta_offset', 'new_computation_flag', 'invalidate_other_poses_radius', 'goal_pose_selection_strategy'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		# try:
			# sm = ApproachPerimeter()
		center = Pose2D()
		center.x = userdata.trash_bin_pose_.pose.pose.position.x
		center.y = userdata.trash_bin_pose_.pose.pose.position.y+0.15 #wmb
		center.theta = 0
		userdata.center = center
		userdata.radius = 1.0  # adjust this for right distance to trash bin
		userdata.goal_pose_theta_offset = 90.0 * math.pi / 180.0  # todo: adjust this rotation angle for the right position relative to the trash bin
		userdata.rotational_sampling_step = 10.0 / 180.0 * math.pi
		userdata.new_computation_flag = True
		userdata.invalidate_other_poses_radius = 1.0  # in meters, radius the current goal covers
		userdata.goal_pose_selection_strategy = 'closest_to_robot'  # 'closest_to_target_gaze_direction', 'closest_to_robot'

		return 'MTTBL_success'

class MoveToTrashBinLocationManual(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['MTTBL_success'], input_keys=['trash_bin_pose_'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		# try:
			# sm = ApproachPerimeter()
		center = Pose2D()
		center.x = userdata.trash_bin_pose_.pose.pose.position.x
		center.y = userdata.trash_bin_pose_.pose.pose.position.y
		center.theta = 0
		print "Trash bin pose:"
		print center
		
		raw_input("Trash bin reached?")
		
		return 'MTTBL_success'

class MoveToTrashBinLocationLinear(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['MTTBL_success'], input_keys=['trash_bin_pose_'],
							output_keys=['center', 'radius', 'rotational_sampling_step', 'goal_pose_theta_offset', 'new_computation_flag', 'invalidate_other_poses_radius', 'goal_pose_selection_strategy'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		# try:
			# sm = ApproachPerimeter()
		center = Pose2D()
		center.x = userdata.trash_bin_pose_.pose.pose.position.x
		center.y = userdata.trash_bin_pose_.pose.pose.position.y
		center.theta = 0
		userdata.center = center
		userdata.radius = 0.6  # adjust this for right distance to trash bin [in m]
		userdata.goal_pose_theta_offset = 95.0 / 180.0 * math.pi  # todo: adjust this rotation angle for the right position relative to the trash bin
		userdata.rotational_sampling_step = 10.0 / 180.0 * math.pi
		userdata.new_computation_flag = True
		userdata.invalidate_other_poses_radius = 1.0  # in meters, radius the current goal covers
		userdata.goal_pose_selection_strategy = 'closest_to_robot'  # 'closest_to_target_gaze_direction', 'closest_to_robot'

		return 'MTTBL_success'

class CheckPositionToTrashBinLocation(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['trash_bin_pose_'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		try:
			listener = get_transform_listener()
			t = rospy.Time(0)
			listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
			(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
			print "Could not lookup robot pose: %s" % e
			return 'failed'
		robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx')  # yields yaw, pitch, roll
		
		dist = math.sqrt((robot_pose_translation[0] - userdata.trash_bin_pose_.pose.pose.position.x) * (robot_pose_translation[0] - userdata.trash_bin_pose_.pose.pose.position.x) + (robot_pose_translation[1] - userdata.trash_bin_pose_.pose.pose.position.y) * (robot_pose_translation[1] - userdata.trash_bin_pose_.pose.pose.position.y))
		print 'xy-dist =', dist
		print 'angle', robot_pose_rotation_euler[0]

		# if dist>0.7 or dist<0.5:
		# 	return 'failed'
		
		return 'success'	

class GraspTrashBin(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['GTB_success', 'failed'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		if (CONFIRM_MODE == True):
			raw_input("positioned correctly?")

		# lwa4d:
		intermediate_folded2overtrashbin_position = [0.8592779506343683, -0.2794097599517722, -0.6911329304972346, -1.6704895336688126, 2.7189611752193663, -0.6486690697962125, -0.00013962634015954637]
		overtrashbin_position = [-0.7597243701006117, -0.29044024082437636, 0.35585518118912385, -1.8774681296628202, 3.024934846386492, -0.23268729587588402, 2.3328594380931804]
		
		intrashbin_position_large = [-0.7597069168080918, -0.7994306105834827, 0.35585518118912385, -1.6194910129255382, 3.024934846386492, -0.02567379329683659, 2.3328594380931804]
		deepintrashbin_position_large = [-0.7846651251116107, -0.9414131452332214, 0.35585518118912385, -1.456459807496748, 3.021932880073062, -0.02567379329683659, 2.475854263709076]
		intrashbin_position_small = [-0.49670325182506625, -0.9567894959432915, 0.15018558213411204, -1.3131333693229736, 3.025004659556572, -0.1595230936322817, 2.374852059896164]  # [-0.4967207051175862, -1.0427120550189724, 0.1498365162837132, -1.2064588454410803, 3.024987206264052, -0.15954054692480166, 2.374869513188684]
		intrashbin_position_smaller = [-0.49658107877742663, -1.2460603661688316, 0.15077899407979012, -0.8043000791965469, 3.0249697529715323, -0.4925493682053197, 2.374869513188684]
		deepintrashbin_position_small = [-0.4967381584101061, -1.0639701653082632, 0.1500459557939525, -1.474611231717489, 3.024917393093972, 0.13828243663551074, 2.374886966481204]  # [-0.49663432530867424, -1.1209673134469444, 0.14990452214815173, -1.428260011571481, 3.0249716446102015, 0.13848445107830495, 2.3748987545970754] #[-0.4967207051175862, -1.223720151743304, 0.1498365162837132, -0.9534384637794623, 3.024987206264052, -0.22654373690886398, 2.374852059896164]
		deepintrashbin_position_smaller = [-0.4965461721923868, -1.2596215744568275, 0.15086626054238986, -0.9657255817135024, 3.025004659556572, -0.278851254591134, 2.374852059896164]  # [-0.49658107877742663, -1.4291454047030367, 0.15676547341413066, -0.5847551125881802, 3.024952299679012, -0.3783524752473308, 2.374886966481204] # [-0.49658107877742663, -1.3030802728314863, 0.15077899407979012, -0.7132986119975625, 3.0249697529715323, -0.4995306852132971, 2.374869513188684]
		
		intermediate1_deep2carry = [-0.3857177646907468, 0.21350612739646632, 0.355820274604084, -2.0331864055257545, 3.021915426780542, -0.43065999292960083, -0.13151055913777274]
		intermediate2_deep2carry = [0.8212821328184517, 0.0005061454830783556, 0.355820274604084, -1.9471765799874738, 2.3328943446782207, -1.2596390277493474, -0.39552651508695497]
		carry_position = [2.022749336598828, -0.9427919553422969, 0.09196139828758122, -1.5927002089074256, 2.4349786526273687, -1.275155004799577, -0.7997796764338816]  # [2.02229555099331, -0.9424428894918981, 0.09210102462774077, -1.5928747418326248, 2.434961199334849, -1.426038718634487, -1.037510973848029] #[2.0313014499336, -0.9694605863127703, 0.2622706266971879, -1.5307061138765867, 2.422900974203568, -1.4726564629552554, -0.407516927048156]
							# [1.9474732859603128, -0.9245881712439961, -0.31063370026995074, -1.9436335616059253, 2.4089034336025734, -0.8307069107792211, -0.8225038632948477]
		
		# 1. arm: folded -> over trash bin
		# lwa4d
		# handle_arm = sss.move("arm",[intermediate_folded2overtrashbin_position, overtrashbin_position])
		if JOURNALIST_MODE == False:
			handle_arm = sss.move("arm", [carry_position, intermediate2_deep2carry, intermediate1_deep2carry, overtrashbin_position])
		else:
			handle_arm = sss.move("arm", [carry_position])
			raw_input("enter")
			handle_arm = sss.move("arm", [intermediate2_deep2carry])
			raw_input("enter")
			handle_arm = sss.move("arm", [intermediate1_deep2carry])
			raw_input("enter")
			handle_arm = sss.move("arm", [overtrashbin_position])
			raw_input("enter")
		
		# lwa
		# handle_arm = sss.move("arm",[[1.2847840785980225, -0.6864653825759888, 2.384225845336914, 1.362023115158081, 0.159407377243042, 0.9801371097564697, -1.39732344150543213]]) # large trash bin
		# handle_arm = sss.move("arm",[[0.6609504818916321, -0.46957021951675415, 2.051220178604126, 1.7225379943847656, 1.0994664430618286, 0.6991068720817566, -3.7607481479644775]])
		# handle_arm = sss.move("arm",[[1.2847840785980225, -0.6864653825759888, 2.384225845336914, 1.362023115158081, 0.159407377243042, 0.9801371097564697, -1.39732344150543213]]) # small trash bin
		# handle_arm = sss.move("arm",[[0.10646567493677139, -0.9334499835968018, 3.0960710048675537, 0.7012139558792114, -0.05262168124318123, 0.738262951374054, -2.813400983810425]]) # small trash bin

		# 2. open hand
		sss.move("sdh", [[0.0, 0.0, 0.0, -1.4, 0.0, -1.4, 0.0]])
		if JOURNALIST_MODE == True:
			raw_input("enter")
		# 3.a) arm: over trash bin -> into trash bin
		# lwa4d
		handle_arm = sss.move("arm", [intrashbin_position_smaller])

		if JOURNALIST_MODE == True:
			raw_input("in trash bin?")
		
		# lwa
		# handle_arm = sss.move("arm",[[0.5458570122718811, -1.0000877380371094, 2.07297420501709, 1.4942553043365479, 1.6078239679336548, 0.7354173064231873, -3.9213883876800537]]) # large trash bin
		# handle_arm = sss.move("arm",[[0.10646567493677139, -1.277030110359192, 3.0960710048675537, 0.5529675483703613, -0.05258183926343918, 0.46139299869537354, -2.8133485317230225]]) # small trash bin

		# 3.b) robot: move left
		handle_base = sss.move_base_rel("base", (0.0, 0.1, 0.0), blocking=True)
		handle_base = sss.move_base_rel("base", (0.0, 0.1, 0.0), blocking=True)
		# rospy.sleep(5)
		# raw_input("moved?")

		# 4. get deeper into trash bin
		handle_arm = sss.move("arm", [deepintrashbin_position_smaller])

		# if JOURNALIST_MODE == True:
# 		raw_input("enter")
		
		# 5. close hand
		# todo: optimize grasp
		# handle_sdh = sss.move("sdh",[[0.20,0,0,0.6,-0.15,0.6,-0.15]])	# large trash bin
		# handle_sdh = sss.move("sdh",[[0.40,0,0,0.6,-0.15,0.6,-0.15]])	# small trash bin
		handle_sdh = sss.move("sdh", [[0.3, -0.2, 0.0, 0.6, -0.3, 0.6, -0.3]])  # smaller trash bin
		# handle_sdh = sss.move("sdh",[[0.47,0,0,0.45,-0,0.45,-0]])	# small trash bin
		if JOURNALIST_MODE == True:
			raw_input("enter")
		
		# 6. arm: lift up
		# lwa4d
		# todo: trajectory very close to robot in between
		if JOURNALIST_MODE == False:
			handle_arm = sss.move("arm", [intermediate1_deep2carry, intermediate2_deep2carry, carry_position])
		else:
			handle_arm = sss.move("arm", [intermediate1_deep2carry])
			raw_input("enter")
			handle_arm = sss.move("arm", [intermediate2_deep2carry])
			raw_input("enter")
			handle_arm = sss.move("arm", [carry_position])
			raw_input("finished grasping")
		
		# lwa
		# handle_arm = sss.move("arm",[[0.6609504818916321, -0.46957021951675415, 2.051220178604126, 1.7225379943847656, 1.0994664430618286, 0.6991068720817566, -3.7607481479644775]])
		# handle_arm = sss.move("arm",[[0.10628669708967209, -0.21421051025390625, 3.096407413482666, 1.2974236011505127, -0.05254769325256348, 0.7705268859863281, -2.813359022140503]]) # small trash bin
		# handle_arm = sss.move("arm",[[1.162199854850769, -0.21367885172367096, 2.4674811363220215, 1.1584975719451904, -0.3269248604774475, 1.0696042776107788, -2.8133485317230225]]) # small trash bin
		# suggested new carry pos
		# handle_arm = sss.move("arm",[[2.1978673934936523, -0.8611428141593933, 2.0929908752441406, 1.7526683807373047, -0.1037636250257492, 0.7569588422775269, -2.8135266304016113]]) # large waste bin
		# handle_arm = sss.move("arm",[[2.5890188217163086, -1.3564121723175049, 2.3780744075775146, 1.9312158823013306, 0.43163323402404785, 0.4533853530883789, -2.814291000366211]]) # small trash bin
		# #handle_arm = sss.move("arm",[[1.7155265808105469, -0.5807472467422485, 2.374333143234253, 0.20792463421821594, -0.19672517478466034, 1.9377082586288452, -2.8133485317230225]])
		
		sss.say(["Lets take the trash can to the waste container."], False)

		return 'GTB_success'



class Turn180(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['arrived'])
#		self.local_costmap_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/local_costmap_node/costmap")

	def checkAngleDiff(self, goal_angle):
		robot_pose_translation = None
		while robot_pose_translation == None:
			(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = currentRobotPose()
		angle_diff = goal_angle - robot_pose_rotation_euler[0]
		while angle_diff < -math.pi:
			angle_diff += 2 * math.pi
		while angle_diff > math.pi:
			angle_diff -= 2 * math.pi
		return angle_diff


	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		# sss.move('base',[3.2, 2.8, -1.2])
		robot_pose_translation = None
		while robot_pose_translation == None:
			(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = currentRobotPose()
		
# 		# 1. adjust base footprint
# 		local_config = self.local_costmap_dynamic_reconfigure_client.get_configuration(5.0)
# 		self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.40,0.36],[-0.40,0.36],[-0.40,-0.36],[0.40,-0.36]]"})

		# 2. turn around
		goal_pose = [robot_pose_translation[0], robot_pose_translation[1], robot_pose_rotation_euler[0] + math.pi]
		sss.move('base', goal_pose, mode='linear')

		# 3. reset footprint
# 		if local_config["footprint"]!=None:
# 			self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": local_config["footprint"]})
# 		else:
# 			rospy.logwarn("Could not read previous local footprint configuration of /local_costmap_node/costmap, resetting to standard value: [[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]].")
# 			self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]]"})

		# check pose
		angle_diff = self.checkAngleDiff(goal_pose[2])
		while abs(angle_diff) > 10.0 / 180.0 * math.pi:
			# correct pose
			sss.move_base_rel('base', (0.0, 0.0, min(max(angle_diff, -10.0 / 180.0 * math.pi), 10.0 / 180.0 * math.pi)), blocking=False)
			angle_diff = self.checkAngleDiff(goal_pose[2])
			rospy.logwarn("Turn180: Linear navigation did not succeed. Using alternative mode.")
		
		# raw_input("turned 180?")

		return 'arrived'



class ClearTrashBinIntoToolWagonPart1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		sss.move("head", "back", False)
		sss.move("torso", "home", False)
		
		# 6. arm: move up, turn aroundintermediate3_carry2clear_position
		if JOURNALIST_MODE == False:
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate1_carry2clear"], ARM_JOINT_CONFIGURATIONS_TRASH["intermediate2_carry2clear"], ARM_JOINT_CONFIGURATIONS_TRASH["intermediate3_carry2clear"]])
		else:
			raw_input("enter")
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate1_carry2clear"]])
			raw_input("enter")
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate2_carry2clear"]])
			raw_input("enter")
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate3_carry2clear"]])
			raw_input("enter")
		
		# up to here: 1,40m distance, then 1,05m
		
		return 'finished'
		
		
class ClearTrashBinIntoToolWagonPart2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['finished'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		if (CONFIRM_MODE == True):
			raw_input("Press key.")
		
		# clear trash bin
		if JOURNALIST_MODE == False:
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate4_carry2clear_small"], ARM_JOINT_CONFIGURATIONS_TRASH["intermediate5_carry2clear_small"],
					ARM_JOINT_CONFIGURATIONS_TRASH["intermediate6_carry2clear_small"], ARM_JOINT_CONFIGURATIONS_TRASH["intermediate7_carry2clear_small"], ARM_JOINT_CONFIGURATIONS_TRASH["clear_small"]])
		else:
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate4_carry2clear_small"]])
			raw_input("enter")
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate5_carry2clear_small"]])
			raw_input("enter")
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate6_carry2clear_small"]])
			raw_input("enter")
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate7_carry2clear_small"]])
			raw_input("enter")
			sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["clear_small"]])
			raw_input("finished trash bin clearing (next steps move back the arm)")


		# move arm back to the side
		sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate7_carry2clear_small"], ARM_JOINT_CONFIGURATIONS_TRASH["intermediate6_carry2clear_small"],
					ARM_JOINT_CONFIGURATIONS_TRASH["intermediate5_carry2clear_small"], ARM_JOINT_CONFIGURATIONS_TRASH["intermediate4_carry2clear_small"]])
		
		# drive forward slightly
		handle_base = sss.move_base_rel("base", (0.1, 0.0, 0.0), blocking=True)
		handle_base = sss.move_base_rel("base", (0.1, 0.0, 0.0), blocking=True)
		handle_base = sss.move_base_rel("base", (0.1, 0.0, 0.0), blocking=True)
		handle_base = sss.move_base_rel("base", (0.1, 0.0, 0.0), blocking=True)
		# todo: check for success!
		
		# move arm back to carry position
		sss.move("arm", [ARM_JOINT_CONFIGURATIONS_TRASH["intermediate3_carry2clear"], ARM_JOINT_CONFIGURATIONS_TRASH["intermediate2_carry2clear"],
					ARM_JOINT_CONFIGURATIONS_TRASH["intermediate1_carry2clear"], ARM_JOINT_CONFIGURATIONS_TRASH["carry"]])
		# handle_arm = sss.move("arm",[[2.5890188217163086, -1.3564121723175049, 2.3780744075775146, 1.9312158823013306, 0.43163323402404785, 0.4533853530883789, -2.814291000366211]]) # small trash bin

		return 'finished'



class MoveToTrashBinPickingLocation(smach.State):
	def __init__(self):
		# smach.State.__init__(self, outcomes=['MTTBPL_done'],input_keys=['trash_bin_pose_'])
		smach.State.__init__(self, outcomes=['MTTBPL_done'], input_keys=['trash_bin_pose_'],
							output_keys=['center', 'radius', 'rotational_sampling_step', 'goal_pose_theta_offset', 'new_computation_flag', 'invalidate_other_poses_radius', 'goal_pose_selection_strategy'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		# try:
		# sm = ApproachPerimeter()
		center = Pose2D(x=userdata.trash_bin_pose_.pose.pose.position.x, y=userdata.trash_bin_pose_.pose.pose.position.y, theta=0)
		print "center: ", center
		userdata.center = center
		userdata.radius = 0.6  # adjust this for right distance to trash bin
		userdata.rotational_sampling_step = 10.0 / 180.0 * math.pi
		userdata.goal_pose_theta_offset = 95.0 / 180.*math.pi  # todo: adjust this rotation angle for the right position relative to the trash bin
		userdata.new_computation_flag = True
		userdata.invalidate_other_poses_radius = 1.0  # in meters, radius the current goal covers
		userdata.goal_pose_selection_strategy = 'closest_to_robot'  # 'closest_to_target_gaze_direction', 'closest_to_robot'          

		 # todo: move back to location where trash bin was grabbed
		 # sss.move('base',[userdata.trash_bin_pose_.x, userdata.trash_bin_pose_.y, userdata.trash_bin_pose_.theta])

		return 'MTTBPL_done'



class ReleaseTrashBin(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['RTB_finished'])

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		if (CONFIRM_MODE == True):
			raw_input("trash bin location ok?")

		# lwa4d:
		intermediate_folded2overtrashbin_position = [0.8592779506343683, -0.2794097599517722, -0.6911329304972346, -1.6704895336688126, 2.7189611752193663, -0.6486690697962125, -0.00013962634015954637]
		overtrashbin_position = [-0.7597243701006117, -0.29044024082437636, 0.35585518118912385, -1.8774681296628202, 3.024934846386492, -0.23268729587588402, 2.3328594380931804]
		
		intrashbin_position_large = [-0.7597069168080918, -0.7994306105834827, 0.35585518118912385, -1.6194910129255382, 3.024934846386492, -0.02567379329683659, 2.3328594380931804]
		deepintrashbin_position_large = [-0.7846651251116107, -0.9414131452332214, 0.35585518118912385, -1.456459807496748, 3.021932880073062, -0.02567379329683659, 2.475854263709076]
		intrashbin_position_small = [-0.49670325182506625, -0.9567894959432915, 0.15018558213411204, -1.3131333693229736, 3.025004659556572, -0.1595230936322817, 2.374852059896164]  # [-0.4967207051175862, -1.0427120550189724, 0.1498365162837132, -1.2064588454410803, 3.024987206264052, -0.15954054692480166, 2.374869513188684]
		intrashbin_position_smaller = [-0.49658107877742663, -1.2460603661688316, 0.15077899407979012, -0.8043000791965469, 3.0249697529715323, -0.4925493682053197, 2.374869513188684]
		deepintrashbin_position_small = [-0.4967381584101061, -1.0639701653082632, 0.1500459557939525, -1.474611231717489, 3.024917393093972, 0.13828243663551074, 2.374886966481204]  # [-0.49663432530867424, -1.1209673134469444, 0.14990452214815173, -1.428260011571481, 3.0249716446102015, 0.13848445107830495, 2.3748987545970754] #[-0.4967207051175862, -1.223720151743304, 0.1498365162837132, -0.9534384637794623, 3.024987206264052, -0.22654373690886398, 2.374852059896164]
		deepintrashbin_position_smaller = [-0.4965461721923868, -1.2596215744568275, 0.15086626054238986, -0.9657255817135024, 3.025004659556572, -0.278851254591134, 2.374852059896164]  # [-0.49658107877742663, -1.4291454047030367, 0.15676547341413066, -0.5847551125881802, 3.024952299679012, -0.3783524752473308, 2.374886966481204] # [-0.49658107877742663, -1.3030802728314863, 0.15077899407979012, -0.7132986119975625, 3.0249697529715323, -0.4995306852132971, 2.374869513188684]
		
		intermediate1_deep2carry = [-0.3857177646907468, 0.21350612739646632, 0.355820274604084, -2.0331864055257545, 3.021915426780542, -0.43065999292960083, -0.13151055913777274]
		intermediate2_deep2carry = [0.8212821328184517, 0.0005061454830783556, 0.355820274604084, -1.9471765799874738, 2.3328943446782207, -1.2596390277493474, -0.39552651508695497]
		carry_position = [2.022749336598828, -0.9427919553422969, 0.09196139828758122, -1.5927002089074256, 2.4349786526273687, -1.275155004799577, -0.7997796764338816]  # [2.02229555099331, -0.9424428894918981, 0.09210102462774077, -1.5928747418326248, 2.434961199334849, -1.426038718634487, -1.037510973848029] #[2.0313014499336, -0.9694605863127703, 0.2622706266971879, -1.5307061138765867, 2.422900974203568, -1.4726564629552554, -0.407516927048156]
							# [1.9474732859603128, -0.9245881712439961, -0.31063370026995074, -1.9436335616059253, 2.4089034336025734, -0.8307069107792211, -0.8225038632948477]
	
		# 8. arm: put down
		handle_arm = sss.move("arm", [intermediate2_deep2carry, intermediate1_deep2carry, intrashbin_position_smaller])
		# lwa
		# handle_arm = sss.move("arm",[[0.10628669708967209, -0.21421051025390625, 3.096407413482666, 1.2974236011505127, -0.05254769325256348, 0.7705268859863281, -2.813359022140503]])
		# handle_arm = sss.move("arm",[[0.10646567493677139, -1.277030110359192, 3.0960710048675537, 0.5529675483703613, -0.05258183926343918, 0.46139299869537354, -2.8133485317230225]])

		# 9. open hand
		sss.move("sdh", "cylopen")

		# 10. arm: trash bin -> over trash bin
		handle_arm = sss.move("arm", [overtrashbin_position])
		# lwa
		# handle_arm = sss.move("arm",[[0.10646567493677139, -0.9334499835968018, 3.0960710048675537, 0.7012139558792114, -0.05262168124318123, 0.738262951374054, -2.813400983810425]])
		# handle_arm = sss.move("arm",[[1.2847840785980225, -0.6864653825759888, 2.384225845336914, 1.362023115158081, 0.159407377243042, 0.9801371097564697, -1.39732344150543213]])

		# 11. close hand
		sss.move("sdh", "home", False)

		# 12. arm: over trash bin -> folded
		handle_arm = sss.move("arm", [intermediate_folded2overtrashbin_position, "folded"])

		return 'RTB_finished'


#----------------------------------------------------------------------------------------------------------------------------------------------------------------
 
 
#--------------------------------------------------------Change Tool Hand --> Vacuum Cleaner---------------------------------------------------------------------

class ChangeToolManual(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['CTM_done'])
		# command line usage:
		# rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital '{uri: "tool_changer_pin2", state: 0}'
		# rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital '{uri: "tool_changer_pin4", state: 0}'
		
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		# move arm with tool facing up (so it cannot fall down on opening)
		handle_arm = sss.move("arm", "pregrasp")
		handle_arm = sss.move("arm", [[1.064633390424021, -1.1901051103498934, 0.6336766915215812, -1.7237046225621198, -1.554041165975751, -1.7535846593562627, -0.00010471975511965978]])
		
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
		
		# move arm with end facing down
		handle_arm = sss.move("arm", [[1.3676400018627566, -0.882106857250454, 0.8536754437354664, -1.6116893911691237, 2.041947958370766, -1.7976018630915598, -0.00010471975511965978]])

		tool_change_successful = ''
		while tool_change_successful != 'yes':
			# wait for confirmation to attach tool
			raw_input("Please attach the tool manually and then press <Enter>.")
			rospy.wait_for_service(service_name) 
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin2', state=1)
				print 'Closing tool changer response: uri=', resp.uri, '  state=', resp.state
				# keep power on closing the changer for safety 
				# rospy.sleep(1.0)
				# resp = req(uri='tool_changer_pin4', state=0)
				# print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			tool_change_successful = raw_input("If the tool was successfully attached type 'yes' and press <Enter>, otherwise just press enter to repeat. >>")
		
		# carrying_position = [1.978714679571011, -0.9163502171745829, 0.08915141819187035, -1.796921184683282, 2.4326209849093216, -1.2165643018101275, 1.2519770323330925] # carrying position
		handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])
		
		print 'Manual tool change successfully completed.'
		
		return 'CTM_done'


class ChangeToolManualPnP(smach.State):
	def __init__(self, release_confirmation_device='keyboard'):  # current_tool='sdh', 
		smach.State.__init__(self, outcomes=['CTM_done_sdh', 'CTM_done_vacuum', 'failed'])
		# command line usage:
		# rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital '{uri: "tool_changer_pin2", state: 0}'
		# rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital '{uri: "tool_changer_pin4", state: 0}'
		
		# self.diagnostics_sub = rospy.Subscriber("/diagnostics_vacuum_cleaner", DiagnosticArray, self.diagnosticCallback)

		if release_confirmation_device == 'joystick':
			self.joystick_sub = rospy.Subscriber("/joy", Joy, self.releaseConfirmationCallback)
		self.release_confirmation = False
		self.release_confirmation_device = release_confirmation_device

		self.attachment_status_sub = rospy.Subscriber("/toolchange_pnp_manager/attachment_status", String, self.attachmentStatusCallback)
# 		self.attachment_status_sub = rospy.Subscriber("/toolchange_pnp_manager/attachment_status", Bool, self.attachmentStatusCallback)

		self.attachment = ""
		self.received_attachment_status = False
		# self.current_tool = current_tool

	def releaseConfirmationCallback(self, msg):
		# check for joystick [0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0] --> buttons 6+8+2 (= both right triggers + lower of the four buttons)
		if msg.buttons[0] == 0 and msg.buttons[1] == 1 and msg.buttons[2] == 0 and msg.buttons[3] == 0 and msg.buttons[4] == 0 and msg.buttons[5] == 1 and msg.buttons[6] == 0 and msg.buttons[7] == 1 and msg.buttons[8] == 0 and msg.buttons[9] == 0 and msg.buttons[10] == 0 and msg.buttons[11] == 0:
			self.release_confirmation = True
	
	def attachmentStatusCallback(self, msg):
		self.attachment = msg.data
		self.received_attachment_status = True
		
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		# move arm with tool facing up (so it cannot fall down on opening)
		# handle_arm = sss.move("arm", "pregrasp")
		# handle_arm = sss.move("arm",[[1.064633390424021, -1.1901051103498934, 0.6336766915215812, -1.7237046225621198, -1.554041165975751, -1.7535846593562627, -0.00010471975511965978]])
		# arm_position = list(ARM_IDLE_POSITION)
		# arm_position[0] = -0.8
		# handle_arm = sss.move("arm",[arm_position])
		
		# ## move arm to release configuration on the backside
		# wait for 10s to read out currently attached device
		attempt = 0;
		while attempt < 100 and self.received_attachment_status == False:
			rospy.sleep(0.1)
			attempt = attempt + 1
		
		TOOLCHANGE_POSITION = [1.3676400018627566, -0.882106857250454, 0.8536754437354664, -1.6116893911691237, 2.041947958370766, -1.7976018630915598, -0.00010471975511965978]
		if self.attachment == 'sdh' or self.attachment == 'none':  # self.current_tool
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"],  # [1.404728248467636, -1.4622368473208494, 0.21975440611860603, -1.7372832841426358, 1.8869103609161093, -1.79756695650652, -0.00013962634015954637],
									TOOLCHANGE_POSITION])
		elif self.attachment == 'vacuum':  # self.current_tool
			handle_arm = sss.move("arm", [TOOLCHANGE_POSITION])
		else:
			sss.say(["I cannot determine the attached device. I will abort."])
			return 'failed'
		if handle_arm.get_error_code() != 0:
			sss.say(["Could not move the arm. I will abort."])
			rospy.logerr("Could not move the arm. I will abort.")
			return 'failed'
			
		
		# ## detach tool
		service_name = '/cob_phidgets_toolchanger/ifk_toolchanger/set_digital'
		tool_change_successful = ''
		while tool_change_successful != 'yes':
			# wait for confirmation to release tool
			sss.say(["Waiting for confirmation to open the tool changer."], False)
			if self.release_confirmation_device == 'joystick':
				# wait for joystick [0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0] --> buttons 6+8+2 (= both right triggers + lower of the four buttons)
				self.release_confirmation = False
				while self.release_confirmation == False:
					rospy.sleep(0.05);
				self.release_confirmation = False
			else:
				raw_input("Please hold the tool tightly with your hands and then press <Enter> and remove the tool quickly.")
			sss.say(["Releasing tool"], False)
			
			# open tool changer
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
			
			while self.attachment != "none":
				rospy.sleep(0.1)
			tool_change_successful = 'yes'  # raw_input("If the tool was successfully removed type 'yes' and press <Enter>, otherwise just press enter to repeat. >>")
		
		# move arm with end facing down
		# handle_arm = sss.move("arm",[[1.3676400018627566, -0.882106857250454, 0.8536754437354664, -1.6116893911691237, 2.041947958370766, -1.7976018630915598, -0.00010471975511965978]])

		print "Device successfully detached."
		sss.say(["Device successfully detached."], False)

		# ## attach new tool
		tool_change_successful = ''
		while tool_change_successful != 'yes':
			# wait for confirmation to attach tool
			# raw_input("Please attach the tool manually and then press <Enter>.")
			while (self.attachment == "none"):
				rospy.sleep(0.1)
			
			rospy.wait_for_service(service_name) 
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin2', state=1)
				print 'Closing tool changer response: uri=', resp.uri, '  state=', resp.state
				# keep power on closing the changer for safety 
				# rospy.sleep(1.0)
				# resp = req(uri='tool_changer_pin4', state=0)
				# print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
				
			try:
				resp = req(uri='tool_changer_pin2', state=0)
				print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			
			tool_change_successful = 'yes'  # raw_input("If the tool was successfully attached type 'yes' and press <Enter>, otherwise just press enter to repeat. >>")
		
		# ## move arm into respective carrying position for the new tool
		if self.attachment == "sdh":
			print 'Manual tool change successfully completed with sdh attached.'
			sss.say(["The hand was connected successfully."], False)
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"], 'folded'])
			if handle_arm.get_error_code() != 0:
				sss.say(["Could not move the arm. I will abort."])
				rospy.logerr("Could not move the arm. I will abort.")
				return 'failed'
			return 'CTM_done_sdh'
		elif self.attachment == "vacuum":
			print 'Manual tool change successfully completed with vacuum cleaner attached.'
			sss.say(["The vacuum cleaner was connected successfully."], False)
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])
			if handle_arm.get_error_code() != 0:
				sss.say(["Could not move the arm. I will abort."])
				rospy.logerr("Could not move the arm. I will abort.")
				return 'failed'
			return 'CTM_done_vacuum'
		
		print 'Manual tool change failed.'
		return 'failed'
			
		# carrying_position = [1.978714679571011, -0.9163502171745829, 0.08915141819187035, -1.796921184683282, 2.4326209849093216, -1.2165643018101275, 1.2519770323330925] # carrying position
# 		handle_arm = sss.move("arm",[ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])
		
# 		print 'Manual tool change successfully completed.'
		
# 		return 'CTM_done'


class ChangeToolManualPnPAttachOnly(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['CTM_done_sdh', 'CTM_done_vacuum', 'failed'])
		# command line usage:
		# rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital '{uri: "tool_changer_pin2", state: 0}'
		# rosservice call /cob_phidgets_toolchanger/ifk_toolchanger/set_digital '{uri: "tool_changer_pin4", state: 0}'
		
		# self.diagnostics_sub = rospy.Subscriber("/diagnostics_vacuum_cleaner", DiagnosticArray, self.diagnosticCallback)
		self.attachment_status_sub = rospy.Subscriber("/toolchange_pnp_manager/attachment_status", String, self.attachmentStatusCallback)
		self.attachment = ""

	def attachmentStatusCallback(self, msg):
		self.attachment = msg.data
		
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		# move arm with tool facing up (so it cannot fall down on opening)
		# handle_arm = sss.move("arm", "pregrasp")
		# handle_arm = sss.move("arm",[[1.064633390424021, -1.1901051103498934, 0.6336766915215812, -1.7237046225621198, -1.554041165975751, -1.7535846593562627, -0.00010471975511965978]])
		# arm_position = list(ARM_IDLE_POSITION)
		# arm_position[0] = -0.8
		# handle_arm = sss.move("arm",[arm_position])
		
		handle_arm = sss.move("arm", [[1.404728248467636, -1.4622368473208494, 0.21975440611860603, -1.7372832841426358, 1.8869103609161093, -1.79756695650652, -0.00013962634015954637],
									[1.3676400018627566, -0.882106857250454, 0.8536754437354664, -1.6116893911691237, 2.041947958370766, -1.7976018630915598, -0.00010471975511965978]])
		
		service_name = '/cob_phidgets_toolchanger/ifk_toolchanger/set_digital'
		tool_change_successful = ''
		while tool_change_successful != 'yes':
			# wait for confirmation to attach tool
			# raw_input("Please attach the tool manually and then press <Enter>.")
			while (self.attachment == ""):
				rospy.sleep(0.2)
			
			rospy.wait_for_service(service_name) 
			try:
				req = rospy.ServiceProxy(service_name, SetDigitalSensor)
				resp = req(uri='tool_changer_pin2', state=1)
				print 'Closing tool changer response: uri=', resp.uri, '  state=', resp.state
				# keep power on closing the changer for safety 
				# rospy.sleep(1.0)
				# resp = req(uri='tool_changer_pin4', state=0)
				# print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
				
			try:
				resp = req(uri='tool_changer_pin2', state=0)
				print 'Resetting tool changer outputs to 0 response: uri=', resp.uri, '  state=', resp.state
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
			
			tool_change_successful = 'yes'  # raw_input("If the tool was successfully attached type 'yes' and press <Enter>, otherwise just press enter to repeat. >>")
		
		# carrying_position = [1.978714679571011, -0.9163502171745829, 0.08915141819187035, -1.796921184683282, 2.4326209849093216, -1.2165643018101275, 1.2519770323330925] # carrying position
		
		if self.attachment == "sdh":
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"], 'folded'])
			print 'Manual tool change successfully completed with sdh attached.'
			return 'CTM_done_sdh'
		elif self.attachment == "vacuum":
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])
			print 'Manual tool change successfully completed with vacuum cleaner attached.'
			return 'CTM_done_vacuum'
		
		print 'Manual tool change failed.'
		return 'failed'


class DetermineAttachedTool(smach.State):
	def __init__(self): 
		smach.State.__init__(self, outcomes=['sdh', 'vacuum', 'none', 'failed'])

		self.attachment_status_sub = rospy.Subscriber("/toolchange_pnp_manager/attachment_status", String, self.attachmentStatusCallback)

		self.attachment = ""
		self.received_attachment_status = False

	def attachmentStatusCallback(self, msg):
		self.attachment = msg.data
		self.received_attachment_status = True
		
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		# reset status flag
		self.received_attachment_status = False
		
		# wait for 10s to read out currently attached device
		attempt = 0;
		while attempt < 100 and self.received_attachment_status == False:
			rospy.sleep(0.1)
			attempt = attempt + 1
		
		if self.attachment == 'sdh':
			sss.say(["I am using my hand now to clear trash bins."], True)
			return 'sdh'
		elif self.attachment == 'vacuum':
			sss.say(["I am using my vacuum cleaner now to clean dirty spots on the ground."], True)
			return 'vacuum'
		elif self.attachment == 'none':
			sss.say(["There is no tool connected to my arm."], True)
			return 'none'
		
		sss.say(["I cannot determine the attached device. I will abort."], True)
		return 'failed'


class GoToToolWagonLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['GTTWL_done'])
             
    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
        return 'GTTWL_done'
    
    
    
class DetectSlotForCurrentTool(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['slot_pose'])
             
    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
        return 'slot_pose'
    
    
    
class MoveArmToSlot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MATS_done'])
             
    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
        return 'MATS_done'
    
    
    
class ReleaseToolChanger(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['RTC_done'])
             
    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
        return 'RTC_done'
    
    
    
class LiftArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['LA_done'])
             
    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
        return 'LA_done'
    
    
    
class DetectSlotForNewDevice(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['DSFND_slot_pose'])
             
    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
        return 'DSFND_slot_pose'
    
    
    
class MoveArmToSlot2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MATS2_done'])
             
    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
        return 'MATS2_done'
      
    
    
class CloseToolChanger(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['CTC_done'])
             
    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
        return 'CTC_done'
    
    
    
class MoveArmToStandardLocation(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MATSL_done'])
             
    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
        return 'MATSL_done'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Get Dirt Map ------------------------------------------------------------------------------------------



class ReceiveDirtMap(smach.State):
	def __init__(self, valid_rectangle_for_dirt_detections=0):
		smach.State.__init__(self, outcomes=['list_of_dirt_location', 'failed'],
							output_keys=['list_of_dirt_locations', 'last_visited_dirt_location'])
		self.valid_rectangle_for_dirt_detections = valid_rectangle_for_dirt_detections

	def agglomerate(self, dirtMap):
		# label individual dirt regions in image/map
		height = dirtMap.info.height
		width = dirtMap.info.width
		groupMap = [0] * width * height
		currentLabel = 1
		for v in range(0, height):
			for u in range(0, width):
				index = v * width + u
				if dirtMap.data[index] > 25 and groupMap[index] == 0:
					# label a new region and find all connected pixels
					print u, v, index, currentLabel
					groupMap[index] = currentLabel
					queue = [ [u, v] ]
					while len(queue) > 0:
						point = queue.pop()
						for dv in range(-1, 2):
							for du in range(-1, 2):
								if dv == 0 and du == 0:
									continue
								newU = point[0] + du
								newV = point[1] + dv
								if newV < 0 or newV >= height or newU < 0 or newU >= width:
									continue
								if dirtMap.data[newV * width + newU] > 25 and groupMap[newV * width + newU] == 0:
									print "added point:", newU, newV
									groupMap[newV * width + newU] = currentLabel
									queue.append([newU, newV])
					currentLabel += 1
		
		# create a list of cell groups that belong to the same cluster
		groups = {}
		for v in range(0, height):
			for u in range(0, width):
				index = v * width + u
				label = groupMap[index]
				if label != 0:
					if label in groups:
						groups[label].append([u, v])
					else:
						groups[label] = [[u, v]]
		
		return groups


	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		rospy.wait_for_service('/dirt_detection/get_dirt_map')
		try:
			req = rospy.ServiceProxy('/dirt_detection/get_dirt_map', GetDirtMap)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call failed: %s" % e
			return 'failed'
		
		# group detections
		dirt_groups = self.agglomerate(resp.dirtMap)

		# convert to metric coordinates
		list_of_dirt_locations = {}
		map_resolution = resp.dirtMap.info.resolution
		map_offset = resp.dirtMap.info.origin
		for group_label in dirt_groups:
			for point in dirt_groups[group_label]:
				x = point[0] * map_resolution + map_offset.position.x
				y = point[1] * map_resolution + map_offset.position.y
				# hack: limit valid space for cleaning
				print "(", x, ", ", y, ")?"
				if (self.valid_rectangle_for_dirt_detections == 0 or
				   (x > valid_rectangle_for_dirt_detections[0] and y > valid_rectangle_for_dirt_detections[1] and
				    x < valid_rectangle_for_dirt_detections[2] and y < valid_rectangle_for_dirt_detections[3])):
					if group_label in list_of_dirt_locations:
						list_of_dirt_locations[group_label].append([x, y])
					else:
						list_of_dirt_locations[group_label] = [[x, y]]
					print "group:", group_label, " --> adding dirt location at (", point[0], ",", point[1] , ")pix = (", x, ",", y, ")m"

		
		# old school: dirt cell by dirt cell
# 		# create list out of map
# 		list_of_dirt_locations = []
# 		map_resolution = resp.dirtMap.info.resolution
# 		map_offset = resp.dirtMap.info.origin
# 		for v in range(0, resp.dirtMap.info.height):
# 			for u in range(0, resp.dirtMap.info.width):
# 				if resp.dirtMap.data[v*resp.dirtMap.info.width + u] > 25:
# 					x = u*map_resolution+map_offset.position.x
# 					y = v*map_resolution+map_offset.position.y
# 					# hack: limit valid space for cleaning
# 					print "(", x, ", ", y, ")?"
# 					if (self.valid_rectangle_for_dirt_detections==0 or
# 					   (x>valid_rectangle_for_dirt_detections[0] and y>valid_rectangle_for_dirt_detections[1] and
# 					    x<valid_rectangle_for_dirt_detections[2] and y<valid_rectangle_for_dirt_detections[3]) ):
# 						list_of_dirt_locations.append([x,y])
# 						print "adding dirt location at (", u, ",", v ,")pix = (", x, ",", y, ")m"
		
		userdata.list_of_dirt_locations = list_of_dirt_locations
		userdata.last_visited_dirt_location = 0  # -1
		
		return 'list_of_dirt_location'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Go To Next Unprocessed dirt Location ------------------------------------------------------------------


class SelectNextUnprocssedDirtSpot(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['selected_next_dirt_location', 'no_dirt_spots_left'],
							input_keys=['list_of_dirt_locations', 'last_visited_dirt_location_in'],
							output_keys=['next_dirt_location', 'last_visited_dirt_location_out'])
	
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		# todo: check whether the next spot is always selected correctly

		print "last_visited_dirt_location =", userdata.last_visited_dirt_location_in
		print "list_of_dirt_locations =", userdata.list_of_dirt_locations
		
		# if (len(userdata.list_of_dirt_locations)==0) or userdata.last_visited_dirt_location_in+1==len(userdata.list_of_dirt_locations):
		if (len(userdata.list_of_dirt_locations) == 0) or userdata.last_visited_dirt_location_in == len(userdata.list_of_dirt_locations):
			sss.say(["All dirt spots have been cleaned."], False)
			return 'no_dirt_spots_left'
		else:
			current_dirt_location = userdata.last_visited_dirt_location_in + 1
			print "current_dirt_location =", current_dirt_location
			userdata.next_dirt_location = userdata.list_of_dirt_locations[current_dirt_location]
			userdata.last_visited_dirt_location_out = current_dirt_location
			print "last_visited_dirt_location =", current_dirt_location
			print "Next dirt location to clean: ", userdata.list_of_dirt_locations[current_dirt_location]
			sss.say(["I am going to clean the next dirt spot."], False)
			return 'selected_next_dirt_location'

		print "Error: the script should never visit this point."
		return 'no_dirt_spots_left'



class MoveLocationPerimeterCleaning(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['movement_prepared'],
							input_keys=['next_dirt_location'],
							output_keys=['center', 'radius', 'rotational_sampling_step', 'goal_pose_theta_offset', 'new_computation_flag', 'invalidate_other_poses_radius', 'goal_pose_selection_strategy'])
	
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		center = Pose2D()
		center.x = userdata.next_dirt_location[0]
		center.y = userdata.next_dirt_location[1]
		center.theta = 0
		userdata.center = center
		userdata.radius = 0.35  # adjust this for right distance to dirt spot
		userdata.goal_pose_theta_offset = 105.0 * math.pi / 180.0  # todo: adjust this rotation angle for the right position relative to the dirt spot
		userdata.rotational_sampling_step = 10.0 / 180.0 * math.pi
		userdata.new_computation_flag = True
		userdata.invalidate_other_poses_radius = 1.0  # in meters, radius the current goal covers
		userdata.goal_pose_selection_strategy = 'closest_to_robot'  # 'closest_to_target_gaze_direction', 'closest_to_robot'
		
		sss.say(["I am now moving to the next dirty spot."], False)
		
		return 'movement_prepared'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Clean -------------------------------------------------------------------------------------------------

class Clean(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['cleaning_done'])
#		self.local_costmap_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/local_costmap_node/costmap")
	
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		# handle_arm = sss.move("arm",[[0.9865473596897948, -1.0831862403727208, 0.8702560716294125, -0.5028991706696462, 1.4975099515036547, -1.6986067879184412, -8.726646259971648e-05]]) # intermediate position with vacuum cleaner in air
		# handle_arm = sss.move("arm",[[]]) #
		# handle_arm = sss.move("arm",[[]]) #
		
		if (CONFIRM_MODE == True):
			raw_input("cleaning position ok?")

		# 1. adjust base footprint
#		local_config = self.local_costmap_dynamic_reconfigure_client.get_configuration(5.0)
#		self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.25,-0.25],[-0.25,-0.25],[-0.25,0.25]]"})  # [[0.25,-0.25],[-0.25,-0.25],[-0.25,0.25]]#[[0.3,0.3],[0.3,-0.3],[-0.3,-0.3],[-0.3,0.3]]
		
		rospy.sleep(0.5)
		rospy.wait_for_service('/update_footprint') 
		try:
			req = rospy.ServiceProxy('/update_footprint', Empty)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to /update_footprint failed: %s" % e

		# 2. clean
		vacuum_init_service_name = '/vacuum_cleaner_controller/init'
		vacuum_recover_service_name = '/vacuum_cleaner_controller/recover'
		vacuum_on_service_name = '/vacuum_cleaner_controller/power_on'
		vacuum_off_service_name = '/vacuum_cleaner_controller/power_off'
		
		# (re-)init vacuum cleaner
 		# rospy.wait_for_service(vacuum_init_service_name) 
		# try:
		# 	req = rospy.ServiceProxy(vacuum_init_service_name, Trigger)
		# 	resp = req()
		# except rospy.ServiceException, e:
		# 	print "Service call failed: %s"%e

# 		carrying_position = [1.978714679571011, -0.9163502171745829, 0.08915141819187035, -1.796921184683282, 2.4326209849093216, -1.2165643018101275, 1.2519770323330925] # carrying position
# 		intermediate1_position = [1.4535276543533975, -0.3381749958664213, -0.07175048554948689, -1.937908881659384, 2.2285221821811047, -1.234576099690709, 1.000527447] # intermediate 1
# 		intermediate2_position = [0.7885223027585182, -0.14316935854109486, -0.07175048554948689, -1.937908881659384, 2.0185241665811468, -0.9095783396768448, 1.000527447] # intermediate 2
# 		above_cleaning_20cm_position = [-0.09950122065619672, -0.19219565722961557, 0.08124507668033604, -2.1109059171170617, 1.7055153453006288, -0.2646093678948603, 1.000527447] # ca. 20cm above cleaning position
# 		above_cleaning_5cm_position = [-0.09944886077863689, -0.7551690607529065, 0.08124507668033604, -1.562907438575882, 1.7055153453006288, -0.2646093678948603, 1.000527447] # just 5cm above cleaning position
# 		cleaning_position = [-0.09923942126839758, -0.909491073214245, 0.08154178265317508, -1.4209598105111834, 1.695622274897531, -0.24858724536155236, 1.066797598696494] #[-0.09944886077863689, -0.9291958404692611, 0.08124507668033604, -1.4179229376127134, 1.7055153453006288, -0.2646093678948603, 1.000527447] # cleaning position 
							# [-0.09943140748611694, -0.8705527776022516, 0.0813497964354557, -1.4487105456178933, 1.6995143591294783, -0.22661355007894374, 0.997525480684839]


		# raw_input("cleaning position ok?")

		# move arm from storage position to cleaning position
		if JOURNALIST_MODE == False:
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate1_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate2_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_20cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["cleaning_position"]])
		else:
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate1_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate2_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_20cm_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["cleaning_position"]])
			raw_input("enter")

		# recover vacuum cleaner (turning on is more reliably thereafter)
		rospy.wait_for_service(vacuum_recover_service_name) 
		try:
			req = rospy.ServiceProxy(vacuum_recover_service_name, Trigger)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to vacuum recover failed: %s" % e
		
# 		# recover vacuum cleaner (turning on is more reliably thereafter)
# 		rospy.wait_for_service(vacuum_recover_service_name) 
# 		try:
# 			req = rospy.ServiceProxy(vacuum_recover_service_name, Trigger)
# 			resp = req()
# 		except rospy.ServiceException, e:
# 			print "Service call to vacuum recover failed: %s"%e

		# turn vacuum cleaner on
		rospy.wait_for_service(vacuum_on_service_name) 
		try:
			req = rospy.ServiceProxy(vacuum_on_service_name, Trigger)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to vacuum on failed: %s" % e
		
		# move base (if necessary)
		# for i in range(0,1):
		# 	for j in range(0,2):
		# 		handle_base = sss.move_base_rel("base", (0.0, -0.1, 0.0), blocking=True)
		# 	for j in range(0,5):
		# 		handle_base = sss.move_base_rel("base", (0.0, 0.1, 0.0), blocking=True)
		# 	for j in range(0,2):
		# 		handle_base = sss.move_base_rel("base", (0.0, -0.1, 0.0), blocking=True)
		for j in range(0, 5):
			handle_base = sss.move_base_rel("base", (0.0, 0.1, 0.0), blocking=True)
			
		
		# if (CONFIRM_MODE==True):
		# raw_input("cleaning finished?")
		
		# turn vacuum cleaner off
		rospy.wait_for_service(vacuum_off_service_name) 
		try:
			req = rospy.ServiceProxy(vacuum_off_service_name, Trigger)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to vacuum off failed: %s" % e
		
		# move arm back to storage position
		handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_20cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate2_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate1_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])
		
		# 4. reset footprint
#		if local_config["footprint"] != None:
#			self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": local_config["footprint"]})
#		else:
#			rospy.logwarn("Could not read previous local footprint configuration of /local_costmap_node/costmap, resetting to standard value: [[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]].")
#			self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]]"})

		rospy.sleep(0.5)
		rospy.wait_for_service('/update_footprint') 
		try:
			req = rospy.ServiceProxy('/update_footprint', Empty)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to /update_footprint failed: %s" % e
	
		return 'cleaning_done'


class CleanCellGroup(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['cleaning_done'], input_keys=['next_dirt_location'])
#		self.local_costmap_dynamic_reconfigure_client = dynamic_reconfigure.client.Client("/local_costmap_node/costmap")
		self.laser_scanner_mode_pub = rospy.Publisher("/laser_top/laser_top_filter_mode/mode", String)
	
	def computeClosestPoint(self, grid_robot, next_point):
		min_dist = 10000000.0
		closest_point = 0
		index_of_closest_point = -1
		for i in range(0, len(grid_robot)):
			point = grid_robot[i]
			dist = math.sqrt((point[0] - next_point[0]) * (point[0] - next_point[0]) + (point[1] - next_point[1]) * (point[1] - next_point[1]))
			if dist < min_dist:
				min_dist = dist
				closest_point = point
				index_of_closest_point = i
		return [closest_point, min_dist, index_of_closest_point]
	
	def computeCleaningPath(self, map_points_to_clean):
		# determine size of dirt spot
		min_x = 10000000.0  # in [m]
		max_x = -10000000.0
		min_y = 10000000.0
		max_y = -10000000.0
		for point in map_points_to_clean:
			if point[0] < min_x:
				min_x = point[0]
			if point[0] > max_x:
				max_x = point[0]
			if point[1] < min_y:
				min_y = point[1]
			if point[1] > max_y:
				max_y = point[1]
		min_x -= 0.1
		max_x += 0.1
		min_y -= 0.1
		max_y += 0.1
		
		# create a regular grid of points to clean
		grid_tool = []
		grid_step = 0.1
		x = min_x
		while x < max_x + 0.9 * grid_step:
			y = min_y
			while y < max_y + 0.9 * grid_step:
				grid_tool.append([x, y])
				y += grid_step
			x += grid_step
		print "min_x:", min_x, "  max_x:", max_x, "  min_y:", min_y, "  max_y:", max_y
		print "grid_tool:", grid_tool, "\n"
		
		# ## check accessibility of several trajectories
		tool_offset_to_base_link = [-0.14, -0.48] #[-0.14, -0.53]  # the x-y-plane offset of the tool center w.r.t. base_link measured in the base_link coordinate system [in m]
		movement_direction = [ [grid_step, 0.0], [0.0, grid_step], [-grid_step, 0.0], [0.0, -grid_step] ]  # potential movement directions of the robot, measured in map coordinates [in m]
		# compute tool offset vectors in the map coordinate system
		tool_offset_vectors_map = []  # (x,y,alpha) tool offset to base_link in map coordinate system for each movement direction [x,y, in m] and angle alpha of movement direction in map coordinate system
		lanes = []  # estimate of the number of lanes to cover the target area while driving along the possible movement directions
		for dir in movement_direction:
			alpha = math.atan2(dir[1], dir[0])
			offset_vector = [ math.cos(alpha) * tool_offset_to_base_link[0] - math.sin(alpha) * tool_offset_to_base_link[1], math.sin(alpha) * tool_offset_to_base_link[0] + math.cos(alpha) * tool_offset_to_base_link[1], alpha ]
			tool_offset_vectors_map.append(offset_vector)
			# ## estimate number of lanes to cover the target area while driving along the current direction
			p1 = []
			p2 = []
			if (x >= 0 and y >= 0) or (x < 0 and y < 0):
				# corner points: (min_x, max_y) and (max_x, min_y)
				p1 = [min_x, max_y]
				p2 = [max_x, min_y]
			else:
				# corner points: (min_x, max_y) and (max_x, min_y)
				p1 = [max_x, max_y]
				p2 = [min_x, min_y]
			# create linear equation in Hesse normal form
			norm = (math.sqrt(dir[0] * dir[0] + dir[1] * dir[1]))
			normal = [dir[1] / norm, -dir[0] / norm]
			d = normal[0] * p1[0] + normal[1] * p1[1]
			if d < 0:
				d *= -1
				normal[0] *= -1
				normal[1] *= -1
			# compute distance between p2 and line
			dist = normal[0] * p2[0] + normal[1] * p2[1] - d
			lanes.append(abs(dist / grid_step))
			print "dir:", dir, "  offset_vector:", offset_vector, "  dist:", dist, "  lanes:", abs(dist / grid_step), "\n"
		# create a grid of robot positions from the grid of tool positions for each movement direction, count the number of accessibile points among them
		number_accessible_points = []  # number of accessible points in each movement direction
		accessible_points = []  # set of accessible points
		for offset_vector in tool_offset_vectors_map:
			grid_robot = []
			for point in grid_tool:
				pose = Pose2D()
				pose.x = point[0] - offset_vector[0]
				pose.y = point[1] - offset_vector[1]
				pose.theta = offset_vector[2]
				grid_robot.append(pose)
			# check accessibility of all grid points
			rospy.wait_for_service('/map_accessibility_analysis/map_points_accessibility_check', 10)
			try:
				get_approach_pose = rospy.ServiceProxy('/map_accessibility_analysis/map_points_accessibility_check', CheckPointAccessibility)
				res = get_approach_pose(grid_robot)
			except rospy.ServiceException, e:
				print "Service call failed: %s" % e
				return 'failed'  # todo
			# count accessible points
			acc = 0
			acc_points = []
			for i in range(0, len(res.accessibility_flags)):
				if res.accessibility_flags[i] == True:
					acc += 1
					acc_points.append(grid_robot[i])
			accessible_points.append(acc_points)
			number_accessible_points.append(acc)
			print "offset_vector:", offset_vector, "   number_accessible_points:", acc, " ( from", len(grid_robot), ")\n"
		
		# ## select best trajectory
		best_trajectories = []
		max_acc = 0
		for i in range(0, len(number_accessible_points)):  # most accessible points
			if number_accessible_points[i] > max_acc:
				max_acc = number_accessible_points[i]
				best_trajectories = [i]
			elif number_accessible_points[i] == max_acc:
				best_trajectories.append(i)
		print "most acc:", max_acc, "  at directions:", best_trajectories, "\n"
		best_trajectories2 = []
		min_lanes = 100000000
		for j in range(0, len(best_trajectories)):  # minimum lanes
			if lanes[best_trajectories[j]] < min_lanes:
				min_lanes = lanes[best_trajectories[j]]
				best_trajectories2 = [best_trajectories[j]]
			elif lanes[best_trajectories[j]] == min_lanes:
				best_trajectories2.append(best_trajectories[j])
		print "min lanes:", min_lanes, "  at directions:", best_trajectories2, "\n"
		best_direction = -1
		trajectory_starting_point = []
		min_dist = 100000000.0
		robot_pose_translation = None
		while robot_pose_translation == None:
			(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = currentRobotPose()
		for i in range(0, len(best_trajectories2)):  # closest point to robot
			starting_point = []
			if movement_direction[best_trajectories2[i]][0] > 0 and movement_direction[best_trajectories2[i]][1] >= 0:
				starting_point = [min_x - tool_offset_vectors_map[best_trajectories2[i]][0], max_y - tool_offset_vectors_map[best_trajectories2[i]][1]]
			elif movement_direction[best_trajectories2[i]][0] >= 0 and movement_direction[best_trajectories2[i]][1] < 0:
				starting_point = [max_x - tool_offset_vectors_map[best_trajectories2[i]][0], max_y - tool_offset_vectors_map[best_trajectories2[i]][1]]
			elif movement_direction[best_trajectories2[i]][0] < 0 and movement_direction[best_trajectories2[i]][1] <= 0:
				starting_point = [max_x - tool_offset_vectors_map[best_trajectories2[i]][0], min_y - tool_offset_vectors_map[best_trajectories2[i]][1]]
			elif movement_direction[best_trajectories2[i]][0] <= 0 and movement_direction[best_trajectories2[i]][1] > 0:
				starting_point = [min_x - tool_offset_vectors_map[best_trajectories2[i]][0], min_y - tool_offset_vectors_map[best_trajectories2[i]][1]]
			dist = math.sqrt((robot_pose_translation[0] - starting_point[0]) * (robot_pose_translation[0] - starting_point[0]) + (robot_pose_translation[1] - starting_point[1]) * (robot_pose_translation[1] - starting_point[1]))
			if dist < min_dist:
				min_dist = dist
				trajectory_starting_point = [starting_point[0], starting_point[1]]
				best_direction = best_trajectories2[i]
		print "min dist:", min_dist, "  at direction:", movement_direction[best_direction], "  with starting point:", trajectory_starting_point, "\n"
		
		# ## compute ordered set of best trajectory way points
		# todo: this simple solution only works properly for axis aligned movement directions
		# todo: it furthermore assumes that all grid points are accessible, inaccessible points will be neglected
		grid_robot = []
		for point in grid_tool:
			grid_robot.append([point[0] - tool_offset_vectors_map[best_direction][0], point[1] - tool_offset_vectors_map[best_direction][1], tool_offset_vectors_map[best_direction][2]])
		print "grid_robot before:", grid_robot, "\n"
		last_point = [trajectory_starting_point[0], trajectory_starting_point[1], tool_offset_vectors_map[best_direction][2]]
		line = [last_point]
		trajectory = []
		[closest_point, min_dist, index_of_closest_point] = self.computeClosestPoint(grid_robot, last_point)
		grid_robot.pop(index_of_closest_point)
		dir = movement_direction[best_direction]
		normal_dir = [dir[1], -dir[0]]
		max_dist = 0.8 * grid_step
		print "walking through grid with dir:", dir, "  normal:", normal_dir, "\n"
		while True:
			next_point = [last_point[0] + dir[0], last_point[1] + dir[1], last_point[2]]
			[closest_point, min_dist, index_of_closest_point] = self.computeClosestPoint(grid_robot, next_point)
			if min_dist < max_dist:  # there was a point in the sampled grid in movement direction from the last point
				line.append(closest_point)
				grid_robot.pop(index_of_closest_point)
				last_point = closest_point
			else:  # no more points in movement direction -> line finished
				first_point_in_line = line[0]
				trajectory.append(line)
				next_point = [first_point_in_line[0] + normal_dir[0], first_point_in_line[1] + normal_dir[1], first_point_in_line[2]]
				[closest_point, min_dist, index_of_closest_point] = self.computeClosestPoint(grid_robot, next_point)
				if min_dist < max_dist:  # start a new line
					line = [closest_point]
					grid_robot.pop(index_of_closest_point)
					last_point = closest_point
				else:  # no more points to sample
					if len(grid_robot) > 0:
						print "grid_robot is not empty:", grid_robot
					break
		
		return trajectory
		
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
# 		if (CONFIRM_MODE==True):
# 			raw_input("cleaning position ok?")

		# compute trajectory
		print "cleaning the following cells:", userdata.next_dirt_location, "\n"
		trajectory = self.computeCleaningPath(userdata.next_dirt_location)
		print "computed trajectory:", trajectory, "\n"
		
		if len(trajectory) == 0:
			print "Error: empty trajectory."
			return 'cleaning_done'
		if len(trajectory[0]) == 0:
			print "Error: empty first line."
			return 'cleaning_done'

		#raw_input("start moving?")

		# move to first point
		sss.move("base", trajectory[0][0], mode='omni')
		sss.move("base", trajectory[0][0], mode='linear')

		# 1. adjust base footprint
#		local_config = self.local_costmap_dynamic_reconfigure_client.get_configuration(5.0)
#		self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.25,-0.25],[-0.25,-0.25],[-0.25,0.25]]"})  # [[0.25,-0.25],[-0.25,-0.25],[-0.25,0.25]]#[[0.3,0.3],[0.3,-0.3],[-0.3,-0.3],[-0.3,0.3]]
		
		rospy.sleep(0.5)
		rospy.wait_for_service('/update_footprint') 
		try:
			req = rospy.ServiceProxy('/update_footprint', Empty)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to /update_footprint failed: %s" % e


		# limit laser scanner range
		self.laser_scanner_mode_pub.publish(String("besen"))


		# 2. clean
		vacuum_init_service_name = '/vacuum_cleaner_controller/init'
		vacuum_recover_service_name = '/vacuum_cleaner_controller/recover'
		vacuum_on_service_name = '/vacuum_cleaner_controller/power_on'
		vacuum_off_service_name = '/vacuum_cleaner_controller/power_off'
		
		# raw_input("cleaning position ok?")

		# move arm from storage position to cleaning position
		if JOURNALIST_MODE == False:
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate1_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate2_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_20cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["cleaning_position"]])
		else:
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate1_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate2_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_20cm_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"]])
			raw_input("enter")
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["cleaning_position"]])
			raw_input("enter")

		# recover vacuum cleaner (turning on is more reliably thereafter)
		rospy.wait_for_service(vacuum_recover_service_name) 
		try:
			req = rospy.ServiceProxy(vacuum_recover_service_name, Trigger)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to vacuum recover failed: %s" % e
		
		# turn vacuum cleaner on
		rospy.wait_for_service(vacuum_on_service_name) 
		try:
			req = rospy.ServiceProxy(vacuum_on_service_name, Trigger)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to vacuum on failed: %s" % e
		
# 		# move base (if necessary)
# 		for j in range(0,5):
# 			handle_base = sss.move_base_rel("base", (0.0, 0.1, 0.0), blocking=True)

		# move along trajectory
		for line in trajectory:
			if len(line) == 0:
				continue
			sss.move("base", line[0], mode='linear')
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["cleaning_position"]])
			for point in line:
				sss.move("base", point, mode='linear')
			for point in reversed(line):
				sss.move("base", point, mode='linear')
			# lift arm
			handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"]])
		
		# if (CONFIRM_MODE==True):
		# raw_input("cleaning finished?")
		
		# turn vacuum cleaner off
		rospy.wait_for_service(vacuum_off_service_name) 
		try:
			req = rospy.ServiceProxy(vacuum_off_service_name, Trigger)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to vacuum off failed: %s" % e
		
		# move arm back to storage position
		handle_arm = sss.move("arm", [ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_5cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["above_cleaning_20cm_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate2_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["intermediate1_position"], ARM_JOINT_CONFIGURATIONS_VACUUM["carrying_position"]])
		
		# 4. reset footprint
#		if local_config["footprint"] != None:
#			self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": local_config["footprint"]})
#		else:
#			rospy.logwarn("Could not read previous local footprint configuration of /local_costmap_node/costmap, resetting to standard value: [[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]].")
#			self.local_costmap_dynamic_reconfigure_client.update_configuration({"footprint": "[[0.45,0.37],[0.45,-0.37],[-0.45,-0.37],[-0.45,0.37]]"})

		rospy.sleep(0.5)
		rospy.wait_for_service('/update_footprint') 
		try:
			req = rospy.ServiceProxy('/update_footprint', Empty)
			resp = req()
		except rospy.ServiceException, e:
			print "Service call to /update_footprint failed: %s" % e
			
		# limit laser scanner range
		self.laser_scanner_mode_pub.publish(String("default"))
	
		return 'cleaning_done'


#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Go To Inspect Location --------------------------------------------------------------------------------



class MoveLocationPerimeterValidation(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['movement_prepared'],
							input_keys=['next_dirt_location'],
							output_keys=['center', 'radius', 'rotational_sampling_step', 'goal_pose_theta_offset', 'new_computation_flag', 'invalidate_other_poses_radius', 'goal_pose_selection_strategy'])
	
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		center = Pose2D()
		center.x = userdata.next_dirt_location[0]
		center.y = userdata.next_dirt_location[1]
		center.theta = 0
		userdata.center = center
		userdata.radius = 1.40  # adjust this for right distance to dirt spot
		userdata.goal_pose_theta_offset = 0.0  # todo: adjust this rotation angle for the right position relative to the dirt spot
		userdata.rotational_sampling_step = 10.0 / 180.0 * math.pi
		userdata.new_computation_flag = True
		userdata.invalidate_other_poses_radius = 1.0  # in meters, radius the current goal covers
		userdata.goal_pose_selection_strategy = 'closest_to_robot'  # 'closest_to_target_gaze_direction', 'closest_to_robot'
		
		return 'movement_prepared'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Verify Cleaning Success -------------------------------------------------------------------------------



class VerifyCleaningProcess(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['verify_cleaning_done'], input_keys=['next_dirt_location'])
	
	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)
		
		sss.say(["I am now checking whether my cleaning was successful. If not, I will inform somebody about the persitent stain."], False)
		
		sss.move("head", "front", False)
		sss.move("torso", "front_extreme")
		
		rospy.sleep(4.0)

		# hack:
	# 	point = Point(x=userdata.next_dirt_location[0], y=userdata.next_dirt_location[1], z=0.0)
	# 	rospy.loginfo('Executing state verify_Cleaning_Process')
	# 	rospy.wait_for_service('/dirt_detection/validate_cleaning_result')
	# 	try:
	# 		req = rospy.ServiceProxy('/dirt_detection/validate_cleaning_result', ValidateCleaningResult)
	# 		resp = req(validationPositions=[point], numberValidationImages=-1)
	# 	except rospy.ServiceException, e:
	# 		print "Service call failed: %s"%e
		
		sss.move("torso", "home", False)
		
		return 'verify_cleaning_done'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------


#-------------------------------------------------------- Process Cleaning Verification Results -----------------------------------------------------------------



class ProcessCleaningVerificationResults(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])

    def execute(self, userdata):
    	sf = ScreenFormat(self.__class__.__name__)
    	
        return 'finished'



#----------------------------------------------------------------------------------------------------------------------------------------------------------------
