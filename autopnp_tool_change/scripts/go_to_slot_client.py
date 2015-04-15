

#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_tool_change')
import rospy
import actionlib
import autopnp_tool_change.msg 

class GoToSlot:
	
	def go_to_slot_client(self, goal_tool, goal_state):
		go_to_slot = actionlib.SimpleActionClient('go_to_slot_action', autopnp_tool_change.msg.ToolChangeAction)
		go_to_slot.wait_for_server()

		# Creates a goal to send to the action server.
		goal = autopnp_tool_change.msg.ToolChangeGoal()
		goal.state = goal_state
		goal.goal = goal_tool

		# Sends the goal to the action server.
		go_to_slot.send_goal(goal)

		# Waits for the server to finish performing the action.
		finished_before_timeout = go_to_slot.wait_for_result(rospy.Duration(300, 0))

   		if finished_before_timeout:
		   	state = go_to_slot.get_state()
		   	result = go_to_slot.get_result()
		   	
			print "Action go_to_slot finished with state: %s"%state
			print "Action go_to_slot finished with result: %s"%result
		# Prints out the result of executing the action
   		return result # State after waiting for GoToStartPositionAction


