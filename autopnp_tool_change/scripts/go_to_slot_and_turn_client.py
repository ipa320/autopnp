

#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_tool_change')
import rospy
import actionlib
import autopnp_tool_change.msg 

class GoToSlotAndTurn:
	
	def go_to_slot_and_turn_client(self, goal_name):
		go_to_slot_and_turn = actionlib.SimpleActionClient('go_to_slot_and_turn_action', autopnp_tool_change.msg.GoToStartPositionAction)
		go_to_slot_and_turn.wait_for_server()

		# Creates a goal to send to the action server.
		goal = autopnp_tool_change.msg.GoToStartPositionGoal()
		goal.goal = goal_name

		# Sends the goal to the action server.
		go_to_slot_and_turn.send_goal(goal)

		# Waits for the server to finish performing the action.
		finished_before_timeout = go_to_slot_and_turn.wait_for_result(rospy.Duration(300, 0))

   		if finished_before_timeout:
		   	state = go_to_slot_and_turn.get_state()
			print "Action finished: %s"%state
		# Prints out the result of executing the action
   		return state # State after waiting for GoToStartPositionAction


