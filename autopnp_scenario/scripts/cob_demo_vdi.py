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
from std_msgs.msg import Bool
from cob_srvs.srv import Trigger
from std_srvs.srv import Empty
from diagnostic_msgs.msg import DiagnosticArray

from simple_script_server import simple_script_server
sss = simple_script_server()


class GraspCoffee(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['succeeded'])
	
	def execute(self, userdata ):
		sf = ScreenFormat(self.__class__.__name__)
		
		grasp_out = [1.2981758976333824, -0.8947954009124528, 0.654673002423073, -1.8360340132204749, 0.3464478565208744, -1.501436942320642, -0.8940972692116552] #[1.2491146923598218, -0.9378875801441929, 0.6548824419333124, -1.8174113501016953, 0.28647834342234924, -1.501402035735602, -0.894149629089215]
		grasp_in = [1.319172208534874, -1.29580224985067, 0.654655549130553, -0.718045907562987, 0.2054776128372924, -0.7244338126252863, -0.5730963131848581]
		#inter1 = [0.8891405341359913, -0.8558920118854992, 0.737803034695563, -1.9027404972316981, -0.026581364507873642, -0.27817057618285623, -0.9220748971211243]
		#inter2 = [1.0291159401459364, -0.5438969547989929, 0.22078415037728266, -1.7277363331342266, 0.36641442316368955, 1.29580224985067, -0.41905355340383854]
		inter1 = [2.4082053019017757, -0.8928580854427391, 0.013072516097437528, -1.4938622133669865, 1.1684281210401237, -1.4554475165305913, -0.8031132553051908]
		inter2 = [2.5702067630718894, -0.8928406321502192, -1.5809017831639436, -1.4338403403909015, 1.569417516685821, -1.319451461215193, -0.8031132553051908]
		inter3 = [1.697210524516846, 0.021572269554649914, -2.2388734578732863, -1.6852899257257243, 1.5695571430259807, -1.3190849420722743, -1.6451298996373351]
		inter4 = [0.8032179750603105, 0.5935690253107515, -2.353890655579712, -1.6852899257257243, 1.860555889210995, -1.4710856666284606, -1.3791068150483594]
		#overtray = [0.9117774545343575, 0.9998642218825115, -2.7898739027278956, -1.518331729479947, 1.7907601724237416, -1.1541687810513301, -1.4759202286564848]
		overtray = [0.9118472677044375, 1.0442653980532473, -2.7902404218708146, -1.540218158299956, 1.7406866761840245, -1.1087727672069576, -1.4760598549966444]


		sss.move("head", ['back'], False)
		sss.move("torso", ['back'])
		sss.move("tray", ['up'])
		handle_arm = sss.move("arm",['pregrasp', grasp_out])
		sss.move("sdh", ['cylopen'])
		raw_input("move in?")
		handle_arm = sss.move("arm",[grasp_in])
		sss.move("sdh", ['cylclosed'])
		raw_input("grasp ok?")
		sss.move("head", ['front'], False)
		sss.move("torso", ['home'])
		handle_arm = sss.move("arm",[grasp_out, inter1, inter2, inter3, inter4, overtray])
		#raw_input("stop")
		sss.move("sdh", ['cylopen'])
		handle_arm = sss.move("arm",[inter4])
		sss.move("sdh", ['home'])
		handle_arm = sss.move("arm",[inter3, inter2, 'pregrasp','folded'])
		return 'succeeded'

def main_grasp():
	rospy.init_node('grasp')
	
	# clean
	sm_scenario = smach.StateMachine(outcomes=['finished'])
	with sm_scenario:
		smach.StateMachine.add('GRASP_COFFEE', GraspCoffee(),
								transitions={'succeeded':'finished'})
	
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_scenario, '/START')
	sis.start()
	
	# Execute SMACH plan
	outcome = sm_scenario.execute()
	
	sis.stop()

def start_prompt():
	print "---------------------------------------------------------"
	print "plese start script with one of the following arguments:"
	print "grasp - starts the grasp coffee scenario"
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

		if flag=="grasp":
			print "Starting grasping coffee scenario..."
			main_grasp()
		else:
			start_prompt()
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)
