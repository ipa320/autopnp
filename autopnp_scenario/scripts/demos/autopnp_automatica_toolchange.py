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
# Supervised by:  
# 
# \date Date of creation: May 2014
#
# \brief
# SMACH representation of AutoPnP Automatica demo for toolchange PnP
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
os.system("rosservice call /say 'I am going to demonstrate the plug and play tool changer.' &")

import roslib; roslib.load_manifest('autopnp_scenario')
import rospy
import smach
import smach_ros

from exploration_detection_cleaning import *


def main(confirm):
	rospy.init_node('exploration_detection_cleaning')
	
	# tool change scenario
	sm_scenario = smach.StateMachine(outcomes=['finished', 'failed'])
	sm_scenario.userdata.sm_trash_bin_counter = 0

	with sm_scenario:

		smach.StateMachine.add('CHANGE_TOOL_MANUAL_IMPLEMENTATION', ChangeToolManualPnP(release_confirmation_device='joystick'),
								transitions={'CTM_done_sdh':'finished', 'CTM_done_vacuum':'finished', 'failed':'failed'})
		
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_scenario, '/START')
	sis.start()
	
	# Execute SMACH plan
	outcome = sm_scenario.execute()
	
	#rospy.spin()
	
	sis.stop()


if __name__ == '__main__':
	try:
#		flag = ""
#		if len(sys.argv)<2:
#			CONFIRM_MODE = True
#		else:
#			flag=sys.argv[1]
#		if len(flag)==0:
#			CONFIRM_MODE = True
#
#		if flag=="auto":
#			CONFIRM_MODE = False  # in auto mode the connection to the control pc may interrupted, the scenario will still work
#			# necessitates that all nodes are started with & and the end, e.g. bringup &
#		else:
#			CONFIRM_MODE = True

		CONFIRM_MODE = False		
		print "CONFIRM_MODE ", CONFIRM_MODE
		main(CONFIRM_MODE)
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
