

#! /usr/bin/env python

import roslib; roslib.load_manifest('autopnp_tool_change')
import rospy
import actionlib
import autopnp_tool_change.msg 

class GoToSlotAndTurn:
	
	def go_to_slot_and_turn_client(self, goal_name, goal_tool):
		go_to_slot_and_turn = actionlib.SimpleActionClient('go_to_slot_and_turn_action', autopnp_tool_change.msg.GoToSlotAndTurnAction)
		go_to_slot_and_turn.wait_for_server()

		# Creates a goal to send to the action server.
		goal = autopnp_tool_change.msg.GoToSlotAndTurnGoal()
		goal.goal = goal_name
		goal.tool = goal_tool

		# Sends the goal to the action server.
		go_to_slot_and_turn.send_goal(goal)

		# Waits for the server to finish performing the action.
		finished_before_timeout = go_to_slot_and_turn.wait_for_result(rospy.Duration(300, 0))

   		if finished_before_timeout:
		   	state = go_to_slot_and_turn.get_state()
		   	result = go_to_slot_and_turn.get_result()
		   	
			print "Action go_to_slot_and_turn finished with state: %s"%state
			print "Action go_to_slot_and_turn finished with result: %s"%result
		# Prints out the result of executing the action
   		return result # State after waiting for GoToStartPositionAction


